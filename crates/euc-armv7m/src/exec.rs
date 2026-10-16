//! Executes one decoded [`Op`], following the ARM ARM (ARMv7-M) "Operation" pseudocode of each
//! instruction in chapter A7 and of the system instructions in B5.2.

// The per-instruction path: forcing these inline runs the interpreter about 10% faster
// (`benches/mips.rs`, `examples/spin.rs`) than leaving the choice to the compiler.
#![allow(clippy::inline_always)]

use euc_core::{Access, Bus, BusFault, BusFaultKind, CoreRegs, Exit, Fault, Size};

use crate::alu::{add_with_carry, shift, shift_c, signed_sat, unsigned_sat};
use crate::op::{BitOp, Dp, Extend, ImmCarry, MulLong, Op, SetFlags, Width, sysm};
use crate::timing::{self, P};
use crate::{Armv7m, CCR_DIV_0_TRP, CCR_UNALIGN_TRP, Stop};

type Step = Result<(), Stop>;

/// A fault that stops the instruction before it retires.
const fn abort(fault: Fault) -> Stop {
    Stop::Abort(Exit::Fault(fault))
}

const fn bus_fault(addr: u32, access: Access, kind: BusFaultKind) -> Stop {
    abort(Fault::Bus(BusFault { addr, access, kind }))
}

/// An access of `n` bytes at `addr` as naturally aligned pieces `(address, bytes)`, as the
/// Cortex-M3 bus interface issues an unaligned access (TRM r2p1, "Unaligned accesses").
fn pieces(addr: u32, n: u32) -> impl Iterator<Item = (u32, u32)> {
    let mut at = addr;
    let end = addr.wrapping_add(n);
    std::iter::from_fn(move || {
        let left = end.wrapping_sub(at);
        if left == 0 {
            return None;
        }
        let size = [4, 2, 1].into_iter().find(|&size| size <= left && at.is_multiple_of(size)).unwrap_or(1);
        let piece = (at, size);
        at = at.wrapping_add(size);
        Some(piece)
    })
}

const fn size_of(n: u32) -> Size {
    match n {
        1 => Size::Byte,
        2 => Size::Half,
        _ => Size::Word,
    }
}

impl Armv7m {
    /// `R[n]`; R15 reads as the current instruction's address plus 4.
    #[inline(always)]
    fn reg(&self, n: u8) -> u32 {
        self.r[usize::from(n & 15)]
    }

    /// `R[n] = value` for R0-R14 (R13 keeps bits 1:0 clear); R15 is `BranchWritePC`
    /// (`ALUWritePC` on ARMv7-M).
    #[inline(always)]
    fn set(&mut self, n: u8, value: u32) {
        if n < 13 {
            self.r[usize::from(n)] = value;
        } else {
            self.set_high(n, value);
        }
    }

    fn set_high(&mut self, n: u8, value: u32) {
        match n & 15 {
            13 => self.r[13] = value & !3,
            14 => self.r[14] = value,
            _ => self.branch(value & !1),
        }
    }

    /// `BranchWritePC`: a taken branch, with the pipeline refill.
    #[inline(always)]
    fn branch(&mut self, target: u32) {
        self.pc = target;
        self.cycles += P;
    }

    /// `BXWritePC` and `LoadWritePC` (A2.3.1): interworking, and an exception return when the
    /// value is `EXC_RETURN` (bits 31:28 all set). The engine owns the mode, so the core does not
    /// check for handler mode; a return attempted in thread mode is the engine's to report.
    fn bx_write_pc(&mut self, value: u32) -> Step {
        if value >> 28 == 0xF {
            return Err(Stop::ExceptionReturn(value));
        }
        self.thumb = value & 1 == 1;
        self.branch(value & !1);
        Ok(())
    }

    /// The destination write of a load: PC loads interwork.
    #[inline(always)]
    fn load_write(&mut self, rt: u8, value: u32) -> Step {
        if rt == 15 {
            self.bx_write_pc(value)
        } else {
            self.set(rt, value);
            Ok(())
        }
    }

    const fn privileged(&self) -> bool {
        self.regs.ipsr() != 0 || self.regs.control & 1 == 0
    }

    const fn setflags(&self, s: SetFlags) -> bool {
        match s {
            SetFlags::No => false,
            SetFlags::Yes => true,
            SetFlags::OutsideIt => self.it == 0,
        }
    }

    #[inline(always)]
    fn set_nz(&mut self, result: u32) {
        self.flags.n = result >> 31 == 1;
        self.flags.z = result == 0;
    }

    // ---- Memory ----

    /// Reads `n` bytes at `addr` with no alignment rule: plain memory directly, the rest through
    /// the bus.
    #[inline(always)]
    fn read(&mut self, bus: &mut dyn Bus, addr: u32, n: u32) -> Result<u32, Stop> {
        for ram in &self.mem.rams {
            if let Some(off) = ram.offset(addr, n) {
                return Ok(ram.read(off, n));
            }
        }
        self.read_slow(bus, addr, n)
    }

    /// MMIO, unmapped, or crossing a region boundary. The bus takes naturally aligned
    /// transfers, so an unaligned access goes out as the Cortex-M3 bus interface splits it:
    /// the largest naturally aligned pieces, lowest address first (a word at offset 1 is a
    /// byte, a halfword and a byte).
    #[cold]
    #[inline(never)]
    fn read_slow(&mut self, bus: &mut dyn Bus, addr: u32, n: u32) -> Result<u32, Stop> {
        let mut value = 0;
        for (at, size) in pieces(addr, n) {
            let piece = match self.mem.find(at, size) {
                Some((r, off)) => self.mem.rams[r].read(off, size),
                None => self.bus_read(bus, at, size)?,
            };
            value |= piece << (8 * at.wrapping_sub(addr));
        }
        Ok(value)
    }

    fn bus_read(&mut self, bus: &mut dyn Bus, addr: u32, n: u32) -> Result<u32, Stop> {
        if !self.mem.is_mmio(addr) {
            return Err(bus_fault(addr, Access::Read, BusFaultKind::Unmapped));
        }
        bus.note_pc(self.r[15].wrapping_sub(4));
        self.bus_touched = true;
        bus.read(addr, size_of(n)).map_err(|fault| abort(Fault::Bus(fault)))
    }

    /// Writes the low `n` bytes of `value` at `addr` with no alignment rule.
    #[inline(always)]
    fn write(&mut self, bus: &mut dyn Bus, addr: u32, n: u32, value: u32) -> Step {
        for (i, ram) in self.mem.rams.iter().enumerate() {
            if let Some(off) = ram.offset(addr, n) {
                if !ram.writable {
                    return Err(bus_fault(addr, Access::Write, BusFaultKind::ReadOnly));
                }
                ram.write(off, n, value);
                let code = &mut self.mem.code[i];
                if code.live && (code.holds_code(off) || code.holds_code(off + n - 1)) {
                    code.clear(off, n);
                }
                return Ok(());
            }
        }
        self.write_slow(bus, addr, n, value)
    }

    #[cold]
    #[inline(never)]
    fn write_slow(&mut self, bus: &mut dyn Bus, addr: u32, n: u32, value: u32) -> Step {
        for (at, size) in pieces(addr, n) {
            let piece = (value >> (8 * at.wrapping_sub(addr))) & Size::mask(size_of(size));
            if self.mem.find(at, size).is_some() {
                self.write(bus, at, size, piece)?;
            } else {
                self.bus_write(bus, at, size, piece)?;
            }
        }
        Ok(())
    }

    fn bus_write(&mut self, bus: &mut dyn Bus, addr: u32, n: u32, value: u32) -> Step {
        if !self.mem.is_mmio(addr) {
            return Err(bus_fault(addr, Access::Write, BusFaultKind::Unmapped));
        }
        bus.note_pc(self.r[15].wrapping_sub(4));
        self.bus_touched = true;
        let size = size_of(n);
        bus.write(addr, size, value & size.mask()).map_err(|fault| abort(Fault::Bus(fault)))
    }

    /// `MemU[addr, size]` load with extension: unaligned is fine unless `CCR.UNALIGN_TRP`.
    #[inline(always)]
    fn load(&mut self, bus: &mut dyn Bus, addr: u32, w: Width) -> Result<u32, Stop> {
        let n = w.bytes();
        if self.ccr & CCR_UNALIGN_TRP != 0 && !addr.is_multiple_of(n) {
            return Err(abort(Fault::Unaligned(addr)));
        }
        let value = self.read(bus, addr, n)?;
        Ok(match w {
            Width::SignedByte => value as u8 as i8 as u32,
            Width::SignedHalf => value as u16 as i16 as u32,
            _ => value,
        })
    }

    /// `MemU[addr, size] = value`.
    #[inline(always)]
    fn store(&mut self, bus: &mut dyn Bus, addr: u32, w: Width, value: u32) -> Step {
        let n = w.bytes();
        if self.ccr & CCR_UNALIGN_TRP != 0 && !addr.is_multiple_of(n) {
            return Err(abort(Fault::Unaligned(addr)));
        }
        self.write(bus, addr, n, value)
    }

    /// `MemA[addr, n]`: an unaligned address is a UsageFault whatever `CCR` says.
    #[inline(always)]
    fn read_aligned(&mut self, bus: &mut dyn Bus, addr: u32, n: u32) -> Result<u32, Stop> {
        if !addr.is_multiple_of(n) {
            return Err(abort(Fault::Unaligned(addr)));
        }
        self.read(bus, addr, n)
    }

    /// `MemA[addr, n] = value`.
    #[inline(always)]
    fn write_aligned(&mut self, bus: &mut dyn Bus, addr: u32, n: u32, value: u32) -> Step {
        if !addr.is_multiple_of(n) {
            return Err(abort(Fault::Unaligned(addr)));
        }
        self.write(bus, addr, n, value)
    }

    // ---- Execution ----

    /// Runs `op`. PC (`self.pc`) already points past it and R15 reads as its address + 4.
    #[inline(always)]
    #[allow(clippy::too_many_lines)]
    pub(crate) fn exec(&mut self, bus: &mut dyn Bus, op: Op) -> Step {
        match op {
            Op::DpImm { op, s, rd, rn, carry, imm } => {
                let carry = match carry {
                    ImmCarry::Keep => self.flags.c,
                    ImmCarry::Clear => false,
                    ImmCarry::Set => true,
                };
                self.data_processing(op, s, rd, self.reg(rn), imm, carry);
            }
            Op::DpReg { op, s, rd, rn, rm, shift, amount } => {
                let (value, carry) = shift_c(self.reg(rm), shift, u32::from(amount), self.flags.c);
                self.data_processing(op, s, rd, self.reg(rn), value, carry);
            }
            Op::ShiftReg { shift, s, rd, rn, rm } => {
                let (result, carry) = shift_c(self.reg(rn), shift, self.reg(rm) & 0xFF, self.flags.c);
                self.set(rd, result);
                if self.setflags(s) {
                    self.set_nz(result);
                    self.flags.c = carry;
                }
            }
            Op::Mul { s, rd, rn, rm } => {
                let result = self.reg(rn).wrapping_mul(self.reg(rm));
                self.set(rd, result);
                // ARMv7: MULS leaves C and V alone.
                if self.setflags(s) {
                    self.set_nz(result);
                }
            }
            Op::Mla { sub, rd, rn, rm, ra } => {
                let product = self.reg(rn).wrapping_mul(self.reg(rm));
                let acc = self.reg(ra);
                self.set(rd, if sub { acc.wrapping_sub(product) } else { acc.wrapping_add(product) });
            }
            Op::MulLong { kind, rdlo, rdhi, rn, rm } => {
                let (n, m) = (self.reg(rn), self.reg(rm));
                let acc = u64::from(self.reg(rdhi)) << 32 | u64::from(self.reg(rdlo));
                let (result, signed, accumulate) = match kind {
                    MulLong::Umull => (u64::from(n) * u64::from(m), false, false),
                    MulLong::Smull => ((i64::from(n as i32) * i64::from(m as i32)) as u64, true, false),
                    MulLong::Umlal => ((u64::from(n) * u64::from(m)).wrapping_add(acc), false, true),
                    MulLong::Smlal => {
                        ((i64::from(n as i32) * i64::from(m as i32)).wrapping_add(acc as i64) as u64, true, true)
                    }
                };
                self.set(rdhi, (result >> 32) as u32);
                self.set(rdlo, result as u32);
                self.cycles += timing::long_multiply(n, m, signed, accumulate) - 1;
            }
            Op::Div { signed, rd, rn, rm } => {
                let (n, m) = (self.reg(rn), self.reg(rm));
                let result = if m == 0 {
                    if self.ccr & CCR_DIV_0_TRP != 0 {
                        return Err(abort(Fault::DivideByZero));
                    }
                    0
                } else if signed {
                    // 0x8000_0000 / -1 overflows to 0x8000_0000 (A7.7.127).
                    (n as i32).wrapping_div(m as i32) as u32
                } else {
                    n / m
                };
                self.set(rd, result);
                let (n, m) = if signed { ((n as i32).unsigned_abs(), (m as i32).unsigned_abs()) } else { (n, m) };
                self.cycles += timing::divide(n, m) - 1;
            }
            Op::Movt { rd, imm } => self.set(rd, u32::from(imm) << 16 | (self.reg(rd) & 0xFFFF)),
            Op::Sat { signed, rd, rn, bits, shift: kind, amount } => {
                let operand = i64::from(shift(self.reg(rn), kind, u32::from(amount), self.flags.c) as i32);
                let (result, saturated) =
                    if signed { signed_sat(operand, u32::from(bits)) } else { unsigned_sat(operand, u32::from(bits)) };
                self.set(rd, result);
                if saturated {
                    self.q = true;
                }
            }
            Op::Bfi { rd, rn, lsb, msb } => {
                let width = u32::from(msb - lsb) + 1;
                let mask = (u32::MAX >> (32 - width)) << lsb;
                let source = if rn == 15 { 0 } else { self.reg(rn) << lsb };
                self.set(rd, (self.reg(rd) & !mask) | (source & mask));
            }
            Op::Bfx { signed, rd, rn, lsb, width } => {
                let field = self.reg(rn) << (32 - u32::from(lsb) - u32::from(width));
                let shift = 32 - u32::from(width);
                self.set(rd, if signed { ((field as i32) >> shift) as u32 } else { field >> shift });
            }
            Op::Bits { op, rd, rm } => {
                let v = self.reg(rm);
                self.set(
                    rd,
                    match op {
                        BitOp::Clz => v.leading_zeros(),
                        BitOp::Rbit => v.reverse_bits(),
                        BitOp::Rev => v.swap_bytes(),
                        BitOp::Rev16 => ((v & 0x00FF_00FF) << 8) | ((v >> 8) & 0x00FF_00FF),
                        BitOp::Revsh => (v as u16).swap_bytes() as i16 as u32,
                    },
                );
            }
            Op::Extend { kind, rd, rm, rotation } => {
                let v = self.reg(rm).rotate_right(u32::from(rotation));
                self.set(
                    rd,
                    match kind {
                        Extend::Sxtb => v as u8 as i8 as u32,
                        Extend::Sxth => v as u16 as i16 as u32,
                        Extend::Uxtb => v & 0xFF,
                        Extend::Uxth => v & 0xFFFF,
                    },
                );
            }

            Op::Load { w, rt, rn, imm } => {
                let value = self.load(bus, self.reg(rn).wrapping_add(imm as u32), w)?;
                return self.load_write(rt, value);
            }
            Op::LoadIdx { w, rt, rn, imm, pre } => {
                let base = self.reg(rn);
                let offset_addr = base.wrapping_add(imm as u32);
                let value = self.load(bus, if pre { offset_addr } else { base }, w)?;
                self.set(rn, offset_addr);
                return self.load_write(rt, value);
            }
            Op::LoadReg { w, rt, rn, rm, lsl } => {
                let addr = self.reg(rn).wrapping_add(self.reg(rm) << lsl);
                let value = self.load(bus, addr, w)?;
                return self.load_write(rt, value);
            }
            Op::LoadLit { w, rt, addr } => {
                let value = self.load(bus, addr, w)?;
                return self.load_write(rt, value);
            }
            Op::Store { w, rt, rn, imm } => {
                self.store(bus, self.reg(rn).wrapping_add(imm as u32), w, self.reg(rt))?;
            }
            Op::StoreIdx { w, rt, rn, imm, pre } => {
                let base = self.reg(rn);
                let offset_addr = base.wrapping_add(imm as u32);
                self.store(bus, if pre { offset_addr } else { base }, w, self.reg(rt))?;
                self.set(rn, offset_addr);
            }
            Op::StoreReg { w, rt, rn, rm, lsl } => {
                let addr = self.reg(rn).wrapping_add(self.reg(rm) << lsl);
                self.store(bus, addr, w, self.reg(rt))?;
            }
            Op::Ldm { rn, list, wback, db } => return self.load_multiple(bus, rn, list, wback, db),
            Op::Stm { rn, list, wback, db } => {
                let count = list.count_ones();
                let base = self.reg(rn);
                let start = if db { base.wrapping_sub(4 * count) } else { base };
                if !start.is_multiple_of(4) {
                    return Err(abort(Fault::Unaligned(start)));
                }
                let mut addr = start;
                let mut bits = list & 0x7FFF;
                while bits != 0 {
                    self.write(bus, addr, 4, self.r[bits.trailing_zeros() as usize])?;
                    addr = addr.wrapping_add(4);
                    bits &= bits - 1;
                }
                if wback {
                    self.set(rn, if db { start } else { base.wrapping_add(4 * count) });
                }
                self.cycles += u64::from(count);
            }
            Op::Dual { load, rt, rt2, rn, imm, index, wback } => {
                let base = self.reg(rn);
                let offset_addr = base.wrapping_add(imm as u32);
                let addr = if index { offset_addr } else { base };
                if load {
                    let first = self.read_aligned(bus, addr, 4)?;
                    let second = self.read_aligned(bus, addr.wrapping_add(4), 4)?;
                    if wback {
                        self.set(rn, offset_addr);
                    }
                    self.set(rt, first);
                    self.set(rt2, second);
                } else {
                    self.write_aligned(bus, addr, 4, self.reg(rt))?;
                    self.write_aligned(bus, addr.wrapping_add(4), 4, self.reg(rt2))?;
                    if wback {
                        self.set(rn, offset_addr);
                    }
                }
            }
            Op::LoadDualLit { rt, rt2, addr } => {
                let first = self.read_aligned(bus, addr, 4)?;
                let second = self.read_aligned(bus, addr.wrapping_add(4), 4)?;
                self.set(rt, first);
                self.set(rt2, second);
            }
            Op::Ldrex { w, rt, rn, imm } => {
                let addr = self.reg(rn).wrapping_add(u32::from(imm));
                let value = self.read_aligned(bus, addr, w.bytes())?;
                self.exclusive = true;
                self.set(rt, value);
            }
            Op::Strex { w, rd, rt, rn, imm } => {
                // ExclusiveMonitorsPass: the alignment check comes first, then the monitor,
                // which the store always leaves open. The Cortex-M3 monitor holds no address.
                let addr = self.reg(rn).wrapping_add(u32::from(imm));
                let n = w.bytes();
                if !addr.is_multiple_of(n) {
                    return Err(abort(Fault::Unaligned(addr)));
                }
                let status = if self.exclusive {
                    self.write(bus, addr, n, self.reg(rt))?;
                    0
                } else {
                    1
                };
                self.exclusive = false;
                self.set(rd, status);
            }
            Op::Clrex => self.exclusive = false,
            Op::Table { half, rn, rm } => {
                let (base, index) = (self.reg(rn), self.reg(rm));
                let offset = if half {
                    self.load(bus, base.wrapping_add(index << 1), Width::Half)?
                } else {
                    self.load(bus, base.wrapping_add(index), Width::Byte)?
                };
                self.branch(self.r[15].wrapping_add(offset << 1));
            }

            Op::B { target } => self.branch(target),
            Op::BCond { cond, target } => {
                if self.flags.passes(cond) {
                    self.branch(target);
                }
            }
            Op::Bl { target } => {
                self.r[14] = self.pc | 1;
                self.branch(target);
            }
            Op::Bx { rm } => return self.bx_write_pc(self.reg(rm)),
            Op::Blx { rm } => {
                // BLXWritePC: interworking, never an exception return.
                let target = self.reg(rm);
                self.r[14] = self.pc | 1;
                self.thumb = target & 1 == 1;
                self.branch(target & !1);
            }
            Op::Cbz { nonzero, rn, target } => {
                if (self.reg(rn) != 0) == nonzero {
                    self.branch(target);
                }
            }

            Op::It { state } => self.it = state,
            Op::Mrs { rd, sysm } => {
                let value = self.read_special(sysm);
                self.set(rd, value);
            }
            Op::Msr { rn, sysm } => self.write_special(sysm, self.reg(rn)),
            Op::Cps { disable, i, f } => {
                if self.privileged() {
                    if i {
                        self.regs.primask = disable;
                    }
                    if f {
                        self.regs.faultmask = disable;
                    }
                }
            }
            Op::Nop | Op::Undecoded | Op::Watch => {}
            Op::Sev => self.event = true,
            Op::Wfe => {
                if self.event {
                    self.event = false;
                } else {
                    return Err(Stop::After(Exit::WaitForEvent));
                }
            }
            Op::Wfi => return Err(Stop::After(Exit::WaitForInterrupt)),
            Op::Svc { imm } => return Err(Stop::After(Exit::Svc(imm))),
            Op::Bkpt { imm } => return Err(Stop::Abort(Exit::Breakpoint(imm))),
            Op::Undefined { raw } | Op::Unpredictable { raw } => return Err(abort(Fault::Undefined(raw))),
        }
        Ok(())
    }

    /// The sixteen data-processing operations (A7.7: AND, BIC, ORR, ..., CMN), with
    /// `shifter_carry` the carry out of the second operand's shift or immediate expansion.
    #[inline(always)]
    fn data_processing(&mut self, op: Dp, s: SetFlags, rd: u8, a: u32, b: u32, shifter_carry: bool) {
        let c = self.flags.c;
        let (result, carry, overflow) = match op {
            Dp::And | Dp::Tst => (a & b, shifter_carry, None),
            Dp::Bic => (a & !b, shifter_carry, None),
            Dp::Orr => (a | b, shifter_carry, None),
            Dp::Orn => (a | !b, shifter_carry, None),
            Dp::Eor | Dp::Teq => (a ^ b, shifter_carry, None),
            Dp::Mov => (b, shifter_carry, None),
            Dp::Mvn => (!b, shifter_carry, None),
            Dp::Add | Dp::Cmn => arith(add_with_carry(a, b, false)),
            Dp::Adc => arith(add_with_carry(a, b, c)),
            Dp::Sbc => arith(add_with_carry(a, !b, c)),
            Dp::Sub | Dp::Cmp => arith(add_with_carry(a, !b, true)),
            Dp::Rsb => arith(add_with_carry(!a, b, true)),
        };
        if !op.is_compare() {
            self.set(rd, result);
        }
        if self.setflags(s) {
            self.set_nz(result);
            self.flags.c = carry;
            if let Some(v) = overflow {
                self.flags.v = v;
            }
        }
    }

    /// `LDM`, `LDMDB`, `POP` (A7.7.40-42, A7.7.99). All loads complete before any register
    /// changes, so a fault leaves the registers as they were; the PC load comes last.
    fn load_multiple(&mut self, bus: &mut dyn Bus, rn: u8, list: u16, wback: bool, db: bool) -> Step {
        let count = list.count_ones();
        let base = self.reg(rn);
        let start = if db { base.wrapping_sub(4 * count) } else { base };
        // Every transfer is word-aligned exactly when the first one is (MemA).
        if !start.is_multiple_of(4) {
            return Err(abort(Fault::Unaligned(start)));
        }
        let mut values = [0_u32; 16];
        let mut addr = start;
        let mut bits = list;
        while bits != 0 {
            values[bits.trailing_zeros() as usize] = self.read(bus, addr, 4)?;
            addr = addr.wrapping_add(4);
            bits &= bits - 1;
        }
        let mut bits = list & 0x7FFF;
        while bits != 0 {
            let i = bits.trailing_zeros() as usize;
            self.set(i as u8, values[i]);
            bits &= bits - 1;
        }
        if wback && list & (1 << rn) == 0 {
            self.set(rn, if db { start } else { base.wrapping_add(4 * count) });
        }
        self.cycles += u64::from(count);
        if list & 0x8000 != 0 {
            return self.bx_write_pc(values[15]);
        }
        Ok(())
    }

    /// `MRS` (B5.2.2). EPSR reads as zero; the stack pointers need privilege.
    fn read_special(&self, sysm: u8) -> u32 {
        match sysm {
            sysm::APSR..=sysm::IEPSR => {
                let mut value = 0;
                if sysm & 1 != 0 {
                    value |= self.regs.xpsr & CoreRegs::XPSR_IPSR;
                }
                if sysm & 4 == 0 {
                    let f = self.flags;
                    value |= u32::from(f.n) << 31
                        | u32::from(f.z) << 30
                        | u32::from(f.c) << 29
                        | u32::from(f.v) << 28
                        | u32::from(self.q) << 27;
                }
                value
            }
            sysm::MSP | sysm::PSP if !self.privileged() => 0,
            sysm::MSP if !self.regs.uses_psp() => self.r[13],
            sysm::MSP => self.regs.msp,
            sysm::PSP if self.regs.uses_psp() => self.r[13],
            sysm::PSP => self.regs.psp,
            sysm::PRIMASK => u32::from(self.regs.primask),
            sysm::BASEPRI | sysm::BASEPRI_MAX => u32::from(self.regs.basepri),
            sysm::FAULTMASK => u32::from(self.regs.faultmask),
            sysm::CONTROL => u32::from(self.regs.control & 3),
            _ => 0,
        }
    }

    /// `MSR` (B5.2.3). Only `APSR_nzcvq` is writable without privilege.
    fn write_special(&mut self, sysm: u8, value: u32) {
        match sysm {
            sysm::APSR..=sysm::XPSR => {
                self.flags.n = value >> 31 & 1 == 1;
                self.flags.z = value >> 30 & 1 == 1;
                self.flags.c = value >> 29 & 1 == 1;
                self.flags.v = value >> 28 & 1 == 1;
                self.q = value >> 27 & 1 == 1;
            }
            _ if !self.privileged() => {}
            sysm::MSP if !self.regs.uses_psp() => self.r[13] = value & !3,
            sysm::MSP => self.regs.msp = value & !3,
            sysm::PSP if self.regs.uses_psp() => self.r[13] = value & !3,
            sysm::PSP => self.regs.psp = value & !3,
            sysm::PRIMASK => self.regs.primask = value & 1 == 1,
            sysm::BASEPRI => self.regs.basepri = value as u8,
            sysm::BASEPRI_MAX => {
                let value = value as u8;
                if value != 0 && (value < self.regs.basepri || self.regs.basepri == 0) {
                    self.regs.basepri = value;
                }
            }
            sysm::FAULTMASK => self.regs.faultmask = value & 1 == 1,
            sysm::CONTROL => {
                // nPRIV always; SPSEL only in thread mode. A SPSEL change switches R13's bank.
                let mut control = (self.regs.control & !1) | (value & 1) as u8;
                if self.regs.ipsr() == 0 {
                    control = (control & !2) | (value & 2) as u8;
                }
                if self.regs.uses_psp() {
                    self.regs.psp = self.r[13];
                } else {
                    self.regs.msp = self.r[13];
                }
                self.regs.control = control;
                self.r[13] = if self.regs.uses_psp() { self.regs.psp } else { self.regs.msp };
            }
            // IPSR, EPSR and IEPSR ignore writes.
            _ => {}
        }
    }
}

/// `AddWithCarry` results as `(result, carry, Some(overflow))`.
#[inline(always)]
const fn arith((result, carry, overflow): (u32, bool, bool)) -> (u32, bool, Option<bool>) {
    (result, carry, Some(overflow))
}
