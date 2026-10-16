//! The Cortex-M exception unit: NVIC, SCB and SysTick in the System Control Space.
//!
//! The engine owns exception logic so that every CPU backend behaves the same at exception
//! boundaries. A backend only reports "a handler returned" ([`crate::Bus::exception_return`]);
//! priorities, pending state, stacking, unstacking and tail-chaining all happen here.
//!
//! The semantics are those of the Python reference harness that boots the stock image, kept
//! exactly so the two produce the same firmware behaviour: `PRIMASK` masks everything
//! (`BASEPRI` and `FAULTMASK` are not modeled), priorities use the implemented high bits only,
//! and system exceptions 14 (PendSV) and 15 (SysTick) are the only ones pended by software.

use crate::{Bus, BusFault, CoreRegs, Size};

/// Offset of the SCS block (`0xE000_E000`) inside the private peripheral bus.
pub const SCS_BASE: u32 = 0xE000_E000;
/// SysTick exception number.
pub const SYSTICK: u16 = 15;
/// PendSV exception number.
pub const PENDSV: u16 = 14;
/// First external interrupt's exception number.
pub const IRQ0: u16 = 16;

/// `EXC_RETURN` that returns to handler mode on the main stack.
const RETURN_TO_HANDLER: u32 = 0xFFFF_FFF1;
/// `EXC_RETURN` that returns to thread mode on the main stack.
const RETURN_TO_THREAD: u32 = 0xFFFF_FFF9;
/// `EXC_RETURN` that returns to thread mode on the process stack.
const RETURN_TO_THREAD_PSP: u32 = 0xFFFF_FFFD;
/// `xPSR` bits of the IT state (`EPSR.ICI/IT`).
const XPSR_IT: u32 = 0x0600_FC00;
/// Stacked `xPSR` bit 9: the frame was realigned to 8 bytes.
const XPSR_ALIGNED: u32 = 1 << 9;
/// `xPSR` bits that are saved in the frame: flags, Q and the IT state.
const XPSR_STACKED: u32 = 0xFE00_FC00;
/// `xPSR` bits a return restores from the frame: flags, Q, Thumb bit, IT state and IPSR.
const XPSR_RESTORED: u32 = 0xF800_0000 | CoreRegs::XPSR_T | XPSR_IT | CoreRegs::XPSR_IPSR;
/// `CCR` offset in the SCS, and its reset value (`STKALIGN`).
const CCR: u32 = 0xD14;
const CCR_RESET: u32 = 0x200;

/// SysTick at `SCS + 0x10`. With `CLKSOURCE` clear it counts HCLK/8, as on STM32.
#[derive(Debug, Clone, Default)]
pub struct SysTick {
    ctrl: u32,
    load: u32,
    t0: u64,
    flag: bool,
    next_event: Option<u64>,
    /// Counter wraps since reset.
    pub wraps: u64,
}

impl SysTick {
    const fn div(&self) -> u64 {
        if self.ctrl & 4 != 0 { 1 } else { 8 }
    }

    const fn period(&self) -> u64 {
        (self.load as u64 + 1) * self.div()
    }

    fn restart(&mut self, now: u64) {
        self.t0 = now;
        self.next_event = (self.ctrl & 1 != 0 && self.load != 0).then(|| now + self.period());
    }

    /// The cycle of the next wrap, if counting.
    pub const fn next_event(&self) -> Option<u64> {
        self.next_event
    }

    /// Handles every wrap due by `now`. Returns true when `TICKINT` asks for the SysTick
    /// exception.
    pub fn on_event(&mut self, now: u64) -> bool {
        let Some(due) = self.next_event.filter(|&due| due <= now) else { return false };
        let period = self.period();
        let wraps = (now - due) / period + 1;
        self.wraps += wraps;
        self.flag = true;
        self.next_event = Some(due + wraps * period);
        self.ctrl & 2 != 0
    }

    fn read(&mut self, off: u32, now: u64) -> (u32, bool) {
        let pend = self.on_event(now);
        let value = match off {
            0x10 => {
                let value = self.ctrl | if self.flag { 0x1_0000 } else { 0 };
                self.flag = false;
                value
            }
            0x14 => self.load,
            0x18 if self.ctrl & 1 == 0 || self.load == 0 => 0,
            0x18 => {
                let elapsed = ((now - self.t0) / self.div()) % (u64::from(self.load) + 1);
                (u64::from(self.load) - elapsed) as u32 & 0x00FF_FFFF
            }
            // CALIB: 9000 ticks per 1 ms at HCLK/8 (72 MHz).
            0x1C => 0x4000_2328,
            _ => 0,
        };
        (value, pend)
    }

    fn write(&mut self, off: u32, value: u32, now: u64) {
        match off {
            0x10 => {
                self.ctrl = value & 7;
                self.restart(now);
            }
            0x14 => {
                self.load = value & 0x00FF_FFFF;
                if self.ctrl & 1 != 0 {
                    self.restart(now);
                }
            }
            0x18 => {
                self.flag = false;
                self.restart(now);
            }
            _ => {}
        }
    }
}

/// Side effects of an SCS write the engine must act on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ScsWrite {
    /// IRQs this write enabled through `ISER` (for the run log).
    pub enabled: u128,
    /// `AIRCR.SYSRESETREQ` was written with the right key.
    pub reset_request: bool,
    /// `CCR` was written: its new value (the CPU backend honours `UNALIGN_TRP` and `DIV_0_TRP`).
    pub ccr: Option<u32>,
}

/// Where an exception entry went.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entry {
    /// The exception taken.
    pub exception: u16,
    /// Its vector-table entry (handler address with the Thumb bit).
    pub vector: u32,
    /// Address of the pushed 8-word frame.
    pub frame: u32,
}

/// NVIC + SCB + SysTick state and the exception entry/return sequences.
#[derive(Debug, Clone)]
pub struct ExceptionUnit {
    irqs: u16,
    priority_mask: u8,
    enabled: u128,
    latched: u128,
    level: u128,
    sys_pending: u16,
    active: Vec<u16>,
    ipr: Vec<u8>,
    shpr: [u8; 16],
    vtor: u32,
    prigroup: u32,
    other: Vec<(u32, u32)>,
    /// The SysTick timer.
    pub systick: SysTick,
    /// Exceptions taken, by exception number (entries and tail-chains).
    pub taken: Vec<u64>,
}

impl ExceptionUnit {
    /// An exception unit for a core with `irqs` external interrupts (at most 128) and
    /// `priority_bits` implemented priority bits (4 on STM32F1).
    ///
    /// # Panics
    ///
    /// If `irqs > 128` or `priority_bits` is not 1 to 8.
    pub fn new(irqs: u16, priority_bits: u8) -> Self {
        assert!(irqs <= 128, "at most 128 external interrupts");
        assert!((1..=8).contains(&priority_bits), "1 to 8 priority bits");
        Self {
            irqs,
            priority_mask: (0xFF_u16 << (8 - priority_bits)) as u8,
            enabled: 0,
            latched: 0,
            level: 0,
            sys_pending: 0,
            active: Vec::new(),
            ipr: vec![0; usize::from(irqs)],
            shpr: [0; 16],
            vtor: 0,
            prigroup: 0,
            other: Vec::new(),
            systick: SysTick::default(),
            taken: vec![0; usize::from(irqs) + usize::from(IRQ0)],
        }
    }

    /// Reset state. Interrupt line levels stay: they belong to the peripherals, which reset
    /// separately and drop their lines themselves.
    pub fn reset(&mut self) {
        let (irqs, mask, level, taken) = (self.irqs, self.priority_mask, self.level, std::mem::take(&mut self.taken));
        *self = Self { priority_mask: mask, level, taken, ..Self::new(irqs, 8) };
    }

    /// Sets the combined level of external interrupt line `irq` (the engine ORs the sources).
    pub fn set_level(&mut self, irq: u16, high: bool) {
        let bit = 1_u128 << irq;
        if high { self.level |= bit } else { self.level &= !bit }
    }

    /// Pends external interrupt `irq` once.
    pub fn pend_irq(&mut self, irq: u16) {
        if irq < self.irqs {
            self.latched |= 1 << irq;
        }
    }

    /// Pends exception `exception` (an IRQ when ≥ 16).
    pub fn pend(&mut self, exception: u16) {
        if exception >= IRQ0 { self.pend_irq(exception - IRQ0) } else { self.sys_pending |= 1 << exception }
    }

    /// Number of external interrupt lines.
    pub const fn irqs(&self) -> u16 {
        self.irqs
    }

    /// Active exceptions, innermost last.
    pub fn active(&self) -> &[u16] {
        &self.active
    }

    /// The vector table base.
    pub const fn vtor(&self) -> u32 {
        self.vtor
    }

    fn priority(&self, exception: u16) -> u8 {
        if exception >= IRQ0 { self.ipr[usize::from(exception - IRQ0)] } else { self.shpr[usize::from(exception)] }
    }

    /// Group priority: the preemption part of a priority under `AIRCR.PRIGROUP`.
    fn group(&self, priority: u8) -> u16 {
        if self.prigroup < 7 { u16::from(priority) & !((2 << self.prigroup) - 1) & 0xFF } else { 0 }
    }

    fn execution_priority(&self) -> u16 {
        self.active.iter().map(|&exc| self.group(self.priority(exc))).min().unwrap_or(256)
    }

    fn is_pending_irq(&self, irq: u16) -> bool {
        let bit = 1_u128 << irq;
        self.latched & bit != 0 || (self.level & bit != 0 && !self.active.contains(&(irq + IRQ0)))
    }

    /// The highest-priority pending exception that may preempt now, if any. Ties go to the lower
    /// exception number.
    pub fn best_pending(&self, primask: bool) -> Option<u16> {
        let limit = if primask { 0 } else { self.execution_priority() };
        let mut best: Option<(u8, u16)> = None;
        let mut enabled = self.enabled;
        while enabled != 0 {
            let irq = enabled.trailing_zeros() as u16;
            enabled &= enabled - 1;
            let priority = self.ipr[usize::from(irq)];
            if self.group(priority) >= limit || best.is_some_and(|(p, _)| priority >= p) {
                continue;
            }
            if self.is_pending_irq(irq) {
                best = Some((priority, irq + IRQ0));
            }
        }
        for exc in [PENDSV, SYSTICK] {
            if self.sys_pending & (1 << exc) == 0 {
                continue;
            }
            let priority = self.shpr[usize::from(exc)];
            if self.group(priority) < limit && best.is_none_or(|(p, e)| priority < p || (priority == p && exc < e)) {
                best = Some((priority, exc));
            }
        }
        best.map(|(_, exc)| exc)
    }

    fn acknowledge(&mut self, exception: u16) {
        if exception >= IRQ0 {
            self.latched &= !(1 << (exception - IRQ0));
        } else {
            self.sys_pending &= !(1 << exception);
        }
        self.active.push(exception);
        if let Some(count) = self.taken.get_mut(usize::from(exception)) {
            *count += 1;
        }
    }

    fn vector(&self, exception: u16, bus: &mut dyn Bus) -> Result<u32, BusFault> {
        bus.read(self.vtor + 4 * u32::from(exception), Size::Word)
    }

    /// Takes `exception`, as `ExceptionTaken` (ARM ARM B1.5.6): pushes the 8-word frame
    /// (realigned to 8 bytes) on the stack in use, marks the exception active, enters handler
    /// mode (`IPSR` = the exception, IT state clear, `CONTROL.SPSEL` = 0, so the main stack) and
    /// points the core at its handler. `PC` gets the handler with bit 0 cleared; `LR` the
    /// `EXC_RETURN` for handler mode, thread mode on the main stack, or thread mode on the
    /// process stack.
    pub fn enter(&mut self, exception: u16, regs: &mut CoreRegs, bus: &mut dyn Bus) -> Result<Entry, BusFault> {
        let sp = regs.sp();
        let nested = !self.active.is_empty();
        let ipsr = self.active.last().copied().map_or(0, u32::from);
        let frame = sp.wrapping_sub(0x20) & !7;
        let aligned = if sp & 4 != 0 { XPSR_ALIGNED } else { 0 };
        let xpsr = (regs.xpsr & XPSR_STACKED) | CoreRegs::XPSR_T | ipsr | aligned;
        let words = [regs.r[0], regs.r[1], regs.r[2], regs.r[3], regs.r[12], regs.lr, regs.pc & !1, xpsr];
        for (i, word) in words.into_iter().enumerate() {
            bus.write(frame + 4 * i as u32, Size::Word, word)?;
        }
        let vector = self.vector(exception, bus)?;
        self.acknowledge(exception);
        // The frame went on the stack in use; set it before the mode changes which one R13 is.
        let process_stack = regs.uses_psp();
        regs.set_sp(frame);
        regs.lr = match (nested, process_stack) {
            (true, _) => RETURN_TO_HANDLER,
            (false, true) => RETURN_TO_THREAD_PSP,
            (false, false) => RETURN_TO_THREAD,
        };
        regs.xpsr = (regs.xpsr & !(XPSR_IT | CoreRegs::XPSR_IPSR)) | u32::from(exception);
        regs.control &= !CoreRegs::CONTROL_SPSEL;
        regs.pc = vector & !1;
        Ok(Entry { exception, vector, frame })
    }

    /// A handler loaded `exc_return` into PC. Tail-chains into the next pending exception (the
    /// frame stays on the stack, `IPSR` becomes the new exception) or, as `ExceptionReturn`
    /// (ARM ARM B1.5.8), unstacks the frame from the stack `exc_return` names and resumes the
    /// interrupted code with its flags, IT state and `IPSR`. Returning to thread mode on the
    /// process stack sets `CONTROL.SPSEL`. Returns the tail-chained entry, if any.
    pub fn exit(&mut self, exc_return: u32, regs: &mut CoreRegs, bus: &mut dyn Bus) -> Result<Option<Entry>, BusFault> {
        self.active.pop();
        let process_stack = exc_return & 0xF == RETURN_TO_THREAD_PSP & 0xF;
        let sp = if process_stack { regs.psp } else { regs.msp };
        if let Some(next) = self.best_pending(regs.primask) {
            let vector = self.vector(next, bus)?;
            self.acknowledge(next);
            regs.lr = exc_return;
            regs.xpsr = (regs.xpsr & !CoreRegs::XPSR_IPSR) | u32::from(next);
            regs.pc = vector & !1;
            return Ok(Some(Entry { exception: next, vector, frame: sp }));
        }
        let mut words = [0_u32; 8];
        for (i, word) in words.iter_mut().enumerate() {
            *word = bus.read(sp + 4 * i as u32, Size::Word)?;
        }
        let [r0, r1, r2, r3, r12, lr, pc, xpsr] = words;
        regs.r[..4].copy_from_slice(&[r0, r1, r2, r3]);
        regs.r[12] = r12;
        regs.lr = lr;
        let popped = sp + 0x20 + if xpsr & XPSR_ALIGNED != 0 { 4 } else { 0 };
        if process_stack {
            regs.psp = popped;
            regs.control |= CoreRegs::CONTROL_SPSEL;
        } else {
            regs.msp = popped;
            regs.control &= !CoreRegs::CONTROL_SPSEL;
        }
        regs.xpsr = (regs.xpsr & !XPSR_RESTORED) | (xpsr & XPSR_RESTORED);
        regs.pc = pc & !1;
        Ok(None)
    }

    /// The `CCR` value.
    pub fn ccr(&self) -> u32 {
        self.stored(CCR).unwrap_or(CCR_RESET)
    }

    /// Reads the SCS register at `off` (from `0xE000_E000`). A SysTick wrap that the read
    /// catches up on pends the SysTick exception, as the hardware would have.
    pub fn read(&mut self, off: u32, now: u64) -> u32 {
        let banked = |bits: u128, base: u32| (bits >> (32 * ((off - base) / 4))) as u32;
        match off {
            0x10..=0x1C => {
                let (value, pend) = self.systick.read(off, now);
                if pend {
                    self.pend(SYSTICK);
                }
                value
            }
            0x100..0x110 => banked(self.enabled, 0x100),
            0x180..0x190 => banked(self.enabled, 0x180),
            0x200..0x210 => banked(self.pending_irqs(), 0x200),
            0x280..0x290 => banked(self.pending_irqs(), 0x280),
            0x300..0x310 => {
                let active = self.active.iter().filter(|&&e| e >= IRQ0).fold(0_u128, |a, &e| a | 1 << (e - IRQ0));
                banked(active, 0x300)
            }
            0x400..0x480 => {
                let i = (off - 0x400) as usize;
                (0..4).fold(0, |word, b| word | u32::from(self.ipr.get(i + b).copied().unwrap_or(0)) << (8 * b))
            }
            0xD00 => 0x411F_C231,
            0xD04 => {
                let mut value = self.active.last().copied().map_or(0, u32::from);
                if self.sys_pending & (1 << SYSTICK) != 0 {
                    value |= 1 << 26;
                }
                if self.sys_pending & (1 << PENDSV) != 0 {
                    value |= 1 << 28;
                }
                value
            }
            0xD08 => self.vtor,
            0xD0C => 0xFA05_0000 | (self.prigroup << 8),
            0xD18..=0xD20 => {
                let i = (off - 0xD18 + 4) as usize;
                u32::from_le_bytes([self.shpr[i], self.shpr[i + 1], self.shpr[i + 2], self.shpr[i + 3]])
            }
            CCR => self.ccr(),
            _ => self.stored(off).unwrap_or(0),
        }
    }

    fn pending_irqs(&self) -> u128 {
        (0..self.irqs).filter(|&irq| self.is_pending_irq(irq)).fold(0, |bits, irq| bits | 1 << irq)
    }

    fn stored(&self, off: u32) -> Option<u32> {
        self.other.iter().find(|(o, _)| *o == off).map(|&(_, v)| v)
    }

    /// Writes the SCS register at `off`: `value` is in its byte lanes and `mask` marks them.
    pub fn write(&mut self, off: u32, value: u32, mask: u32, now: u64) -> ScsWrite {
        let v = value & mask;
        let lanes = |base: u32| u128::from(v) << (32 * ((off - base) / 4));
        let mut out = ScsWrite::default();
        match off {
            0x10..=0x1C => self.systick.write(off, v, now),
            0x100..0x110 => {
                let bits = lanes(0x100) & self.irq_mask();
                self.enabled |= bits;
                out.enabled = bits;
            }
            0x180..0x190 => self.enabled &= !lanes(0x180),
            0x200..0x210 => self.latched |= lanes(0x200) & self.irq_mask(),
            0x280..0x290 => self.latched &= !lanes(0x280),
            0x400..0x480 => {
                let i = (off - 0x400) as usize;
                for b in 0..4 {
                    if mask >> (8 * b) & 0xFF != 0
                        && let Some(slot) = self.ipr.get_mut(i + b)
                    {
                        *slot = (v >> (8 * b)) as u8 & self.priority_mask;
                    }
                }
            }
            0xD04 => {
                if v & 1 << 26 != 0 {
                    self.sys_pending |= 1 << SYSTICK;
                }
                if v & 1 << 25 != 0 {
                    self.sys_pending &= !(1 << SYSTICK);
                }
                if v & 1 << 28 != 0 {
                    self.sys_pending |= 1 << PENDSV;
                }
                if v & 1 << 27 != 0 {
                    self.sys_pending &= !(1 << PENDSV);
                }
            }
            0xD08 => self.vtor = v & 0xFFFF_FF80,
            0xD0C if value >> 16 == 0x05FA => {
                self.prigroup = (value >> 8) & 7;
                out.reset_request = value & 4 != 0;
            }
            0xD0C => {}
            0xD18..=0xD20 => {
                let i = (off - 0xD18 + 4) as usize;
                for b in 0..4 {
                    if mask >> (8 * b) & 0xFF != 0 {
                        self.shpr[i + b] = (v >> (8 * b)) as u8 & self.priority_mask;
                    }
                }
            }
            0xF00 => self.pend_irq((v & 0x1FF) as u16),
            _ => {
                let old = self.stored(off).unwrap_or(if off == CCR { CCR_RESET } else { 0 });
                let new = (old & !mask) | v;
                match self.other.iter_mut().find(|(o, _)| *o == off) {
                    Some(slot) => slot.1 = new,
                    None => self.other.push((off, new)),
                }
                if off == CCR {
                    out.ccr = Some(new);
                }
            }
        }
        out
    }

    const fn irq_mask(&self) -> u128 {
        if self.irqs >= 128 { u128::MAX } else { (1 << self.irqs) - 1 }
    }

    /// Register name at `off` for traces.
    pub fn reg_name(off: u32) -> Option<&'static str> {
        Some(match off {
            0x004 => "ICTR",
            0x010 => "SYST_CSR",
            0x014 => "SYST_RVR",
            0x018 => "SYST_CVR",
            0x01C => "SYST_CALIB",
            0x100..0x110 => "ISER",
            0x180..0x190 => "ICER",
            0x200..0x210 => "ISPR",
            0x280..0x290 => "ICPR",
            0x300..0x310 => "IABR",
            0x400..0x480 => "IPR",
            0xD00 => "CPUID",
            0xD04 => "ICSR",
            0xD08 => "VTOR",
            0xD0C => "AIRCR",
            0xD10 => "SCR",
            0xD14 => "CCR",
            0xD18 => "SHPR1",
            0xD1C => "SHPR2",
            0xD20 => "SHPR3",
            0xD24 => "SHCSR",
            0xD28 => "CFSR",
            0xD2C => "HFSR",
            0xD34 => "MMFAR",
            0xD38 => "BFAR",
            0xF00 => "STIR",
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Access, BusFaultKind, Lane};

    /// 8 KiB of RAM at `0x2000_0000` plus a vector table at 0 where vector n is `0x100 + 4n + 1`.
    struct Mem(Vec<u8>);

    impl Mem {
        fn new() -> Self {
            Self(vec![0; 0x2000])
        }
    }

    impl Bus for Mem {
        fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
            if addr < 0x400 {
                return Ok(0x100 + addr + 1);
            }
            let lane = Lane::new(addr, size);
            let i = lane.word.wrapping_sub(0x2000_0000) as usize;
            let raw =
                self.0.get(i..i + 4).ok_or(BusFault { addr, access: Access::Read, kind: BusFaultKind::Unmapped })?;
            Ok(lane.extract(u32::from_le_bytes(raw.try_into().expect("4"))))
        }
        fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
            let lane = Lane::new(addr, size);
            let i = lane.word.wrapping_sub(0x2000_0000) as usize;
            let raw = self.0.get_mut(i..i + 4).ok_or(BusFault {
                addr,
                access: Access::Write,
                kind: BusFaultKind::Unmapped,
            })?;
            let word = lane.merge(u32::from_le_bytes((&*raw).try_into().expect("4")), value);
            raw.copy_from_slice(&word.to_le_bytes());
            Ok(())
        }
        fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
            Err(BusFault { addr, access: Access::Fetch, kind: BusFaultKind::NoExecute })
        }
        fn exception_return(&mut self, _regs: &mut CoreRegs, _exc_return: u32) {}
    }

    fn unit() -> ExceptionUnit {
        let mut unit = ExceptionUnit::new(68, 4);
        unit.write(0x100, 0xFFFF_FFFF, u32::MAX, 0);
        unit.write(0x104, 0xFFFF_FFFF, u32::MAX, 0);
        unit
    }

    #[test]
    fn levels_pend_until_their_handler_runs() {
        let mut unit = unit();
        assert_eq!(unit.best_pending(false), None);
        unit.set_level(44, true);
        assert_eq!(unit.best_pending(false), Some(60));
        assert_eq!(unit.best_pending(true), None, "PRIMASK masks everything");
        unit.write(0x180 + 4, 1 << 12, u32::MAX, 0);
        assert_eq!(unit.best_pending(false), None, "disabled in ICER");
    }

    #[test]
    fn priorities_and_grouping() {
        let mut unit = unit();
        // PRIGROUP 5: two group bits, two sub bits (of the four implemented).
        unit.write(0xD0C, 0x05FA_0500, u32::MAX, 0);
        unit.write(0x410, 0x40 << 24, 0xFF00_0000, 0); // IRQ19 priority 0x40
        unit.write(0x408, 0x50 << 24, 0xFF00_0000, 0); // IRQ11 priority 0x50 (same group, worse sub)
        unit.set_level(19, true);
        unit.set_level(11, true);
        assert_eq!(unit.best_pending(false), Some(35));
        unit.write(0x408, 0x30 << 24, 0xFF00_0000, 0);
        assert_eq!(unit.best_pending(false), Some(27), "lower priority value wins");
        assert_eq!(unit.read(0x408, 0), 0x30 << 24);
        assert_eq!(unit.read(0xD0C, 0), 0xFA05_0500);
    }

    #[test]
    fn entry_pushes_an_aligned_frame_and_exit_unstacks_it() {
        let mut unit = unit();
        let mut bus = Mem::new();
        let mut regs = CoreRegs::at_reset(0x2000_1EEC, 0x0800_322D);
        regs.r[..4].copy_from_slice(&[1, 2, 3, 4]);
        regs.r[12] = 12;
        regs.lr = 0x0800_1235;
        regs.pc = 0x0800_4000;
        regs.xpsr |= 0x8000_0000;
        let before = regs.clone();

        unit.set_level(44, true);
        let entry = unit.enter(60, &mut regs, &mut bus).unwrap();
        assert_eq!(entry.frame, 0x2000_1EC8, "0x2000_1EEC - 0x20 realigned to 8");
        assert_eq!((regs.sp(), regs.lr, regs.pc), (0x2000_1EC8, 0xFFFF_FFF9, 0x100 + 240));
        assert_eq!(unit.active(), [60]);
        assert_eq!(bus.read(0x2000_1EC8 + 24, Size::Word).unwrap(), 0x0800_4000);
        assert_eq!(bus.read(0x2000_1EC8 + 28, Size::Word).unwrap(), 0x8100_0200, "flags, T, realign bit");

        unit.set_level(44, false);
        regs.r[0] = 99;
        regs.xpsr = CoreRegs::XPSR_T;
        assert_eq!(unit.exit(0xFFFF_FFF9, &mut regs, &mut bus).unwrap(), None);
        assert_eq!(regs, before);
        assert_eq!(unit.active(), []);
        assert_eq!(unit.taken[60], 1);
    }

    #[test]
    fn exit_tail_chains_into_the_next_pending_exception() {
        let mut unit = unit();
        let mut bus = Mem::new();
        let mut regs = CoreRegs::at_reset(0x2000_1000, 1);
        unit.set_level(3, true);
        unit.enter(19, &mut regs, &mut bus).unwrap();
        let frame = regs.sp();
        unit.pend(SYSTICK);
        let chained = unit.exit(0xFFFF_FFF9, &mut regs, &mut bus).unwrap().unwrap();
        assert_eq!(chained.exception, 15);
        assert_eq!((regs.sp(), regs.lr, regs.pc), (frame, 0xFFFF_FFF9, 0x100 + 60));
        assert_eq!(unit.read(0xD04, 0) & 0x1FF, 15);
    }

    #[test]
    fn entry_and_return_track_ipsr() {
        let mut unit = unit();
        let mut bus = Mem::new();
        let mut regs = CoreRegs::at_reset(0x2000_1000, 1);
        unit.set_level(3, true);
        unit.enter(19, &mut regs, &mut bus).unwrap();
        assert_eq!(regs.ipsr(), 19);
        unit.set_level(3, false);
        unit.pend(SYSTICK);
        unit.exit(0xFFFF_FFF9, &mut regs, &mut bus).unwrap();
        assert_eq!(regs.ipsr(), 15, "a tail-chain switches IPSR to the next handler");
        unit.exit(0xFFFF_FFF9, &mut regs, &mut bus).unwrap();
        assert_eq!(regs.ipsr(), 0, "back in thread mode");
    }

    #[test]
    fn thread_mode_on_the_process_stack_returns_to_it() {
        let mut unit = unit();
        let mut bus = Mem::new();
        let mut regs = CoreRegs::at_reset(0x2000_1000, 1);
        regs.psp = 0x2000_0800;
        regs.control = CoreRegs::CONTROL_SPSEL;
        unit.set_level(3, true);
        let entry = unit.enter(19, &mut regs, &mut bus).unwrap();
        assert_eq!(entry.frame, 0x2000_07E0, "the frame goes on the process stack");
        assert_eq!((regs.psp, regs.msp, regs.lr), (0x2000_07E0, 0x2000_1000, 0xFFFF_FFFD));
        assert_eq!((regs.control, regs.sp()), (0, 0x2000_1000), "the handler runs on the main stack");
        unit.set_level(3, false);
        unit.exit(0xFFFF_FFFD, &mut regs, &mut bus).unwrap();
        assert_eq!((regs.psp, regs.control, regs.sp()), (0x2000_0800, CoreRegs::CONTROL_SPSEL, 0x2000_0800));
    }

    #[test]
    fn ccr_writes_are_reported() {
        let mut unit = unit();
        assert_eq!(unit.ccr(), 0x200);
        assert_eq!(unit.write(0xD14, 0x10, 0xFF, 0).ccr, Some(0x210), "DIV_0_TRP set, STKALIGN kept");
        assert_eq!(unit.ccr(), 0x210);
        assert_eq!(unit.write(0xD10, 0, u32::MAX, 0).ccr, None);
    }

    #[test]
    fn nested_entry_returns_to_handler_mode() {
        let mut unit = unit();
        let mut bus = Mem::new();
        let mut regs = CoreRegs::at_reset(0x2000_1000, 1);
        unit.write(0x40C, 0x20, 0xFF, 0); // IRQ12 priority 0x20
        unit.write(0x40C, 0x10 << 8, 0xFF00, 0); // IRQ13 priority 0x10
        unit.set_level(12, true);
        unit.enter(28, &mut regs, &mut bus).unwrap();
        unit.set_level(13, true);
        assert_eq!(unit.best_pending(false), Some(29), "higher priority preempts");
        unit.enter(29, &mut regs, &mut bus).unwrap();
        assert_eq!(regs.lr, 0xFFFF_FFF1);
        assert_eq!(bus.read(regs.sp() + 28, Size::Word).unwrap() & 0x1FF, 28, "stacked IPSR is the preempted handler");
    }

    #[test]
    fn systick_counts_hclk_over_8_and_pends() {
        let mut unit = unit();
        unit.write(0x14, 8999, u32::MAX, 100);
        unit.write(0x10, 0b011, u32::MAX, 100);
        assert_eq!(unit.systick.next_event(), Some(100 + 72_000));
        assert_eq!(unit.read(0x18, 100 + 8 * 1000), 7999);
        assert!(unit.systick.on_event(100 + 72_000 * 3 + 5), "TICKINT pends");
        assert_eq!(unit.systick.wraps, 3);
        assert_eq!(unit.systick.next_event(), Some(100 + 72_000 * 4));
        assert_eq!(unit.read(0x10, 100 + 72_000 * 3 + 6) & 0x1_0000, 0x1_0000, "COUNTFLAG");
        assert_eq!(unit.read(0x10, 100 + 72_000 * 3 + 7) & 0x1_0000, 0, "cleared by the read");
    }

    #[test]
    fn scb_registers() {
        let mut unit = unit();
        unit.write(0xD08, 0x0800_3000, u32::MAX, 0);
        assert_eq!(unit.vtor(), 0x0800_3000);
        assert_eq!(unit.read(0xD14, 0), 0x200, "CCR reset value");
        unit.write(0xD04, 1 << 28, u32::MAX, 0);
        assert_eq!(unit.best_pending(false), Some(14));
        assert!(unit.write(0xD0C, 0x05FA_0004, u32::MAX, 0).reset_request);
        assert!(!unit.write(0xD0C, 0x1234_0004, u32::MAX, 0).reset_request, "wrong key");
        unit.write(0xF00, 7, u32::MAX, 0);
        assert_eq!(unit.read(0x200, 0) & 1 << 7, 1 << 7, "STIR pends");
        assert_eq!(unit.write(0x100, 1 << 3, u32::MAX, 0).enabled, 1 << 3);
    }
}
