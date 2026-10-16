use std::ptr::NonNull;

use thiserror::Error;

use crate::{Bus, BusFault, CoreRegs};

/// One region of the guest address space, as the engine lays it out for a CPU backend.
///
/// Backends execute plain memory in place (no copy, no callback) and send everything else to
/// the [`Bus`]. The engine owns the memory; [`RegionKind::Memory`] only lends a pointer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Region {
    /// First address.
    pub base: u32,
    /// Size in bytes.
    pub size: u32,
    /// What backs it.
    pub kind: RegionKind,
}

/// What backs a [`Region`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegionKind {
    /// Engine-owned bytes the backend reads, writes and executes directly. The engine keeps
    /// them alive and in place for the backend's lifetime and touches them during a run only
    /// from inside the run's [`Bus`] callbacks.
    Memory {
        /// Start of the buffer of `size` bytes.
        ptr: NonNull<u8>,
        /// Stores allowed; a store to read-only memory is a bus fault.
        writable: bool,
    },
    /// Every access goes to the [`Bus`].
    Mmio,
}

/// A CPU backend: our ARMv7-M interpreter, or Unicorn as the test oracle.
///
/// A backend only executes instructions. It knows nothing about the NVIC, priorities or time: it
/// runs until its budget is spent or something needs the engine, and says why with an [`Exit`].
/// The engine owns the exception unit, so exception entry, return, tail-chaining and priority
/// logic exist once and both backends behave the same at every exception boundary.
///
/// Registers are coherent only between calls to [`run`](Self::run). The trait is object safe so
/// the simulator can pick a backend at run time (`Box<dyn Cpu>`).
pub trait Cpu {
    /// Resets the core to [`CoreRegs::at_reset`]`(sp, reset_vector)` and clears execution state
    /// that is not in [`CoreRegs`] (IT block state, the exclusive monitor).
    ///
    /// The engine reads `sp` and `reset_vector` from whichever vector table the wheel boots
    /// through (the bootloader's, or the app's when no bootloader is modeled).
    fn reset(&mut self, sp: u32, reset_vector: u32);

    /// Executes about `max_instr` instructions against `bus`: exactly that many on an exact
    /// backend, or up to the next block boundary on a block-granular one. [`Cpu::retired`]
    /// says how many ran.
    fn run(&mut self, bus: &mut dyn Bus, max_instr: u32) -> Exit;

    /// The register file.
    fn regs(&self) -> &CoreRegs;

    /// The register file, for the exception unit, the inspector and save states.
    fn regs_mut(&mut self) -> &mut CoreRegs;

    /// Instructions retired since the backend was created. It never resets, so the engine
    /// measures a run as a difference.
    fn retired(&self) -> u64;

    /// Reports, through [`Bus::reached`], the first time execution reaches each of `addrs`.
    /// Replaces any earlier list; an empty list stops watching. Watching must cost nothing once
    /// the list is empty, since milestones are only interesting during boot.
    fn watch(&mut self, addrs: &[u32]);

    /// Drops anything decoded or translated from `addr..addr + len`.
    ///
    /// The engine calls it after a write to memory that may hold code (a flash program cycle,
    /// RAM-resident functions), so a stale decode never runs.
    fn invalidate(&mut self, addr: u32, len: u32);

    /// The firmware wrote `CCR` (`0xE000_ED14`), or a reset restored it. A backend that models
    /// `CCR.UNALIGN_TRP` and `CCR.DIV_0_TRP` takes them from here; the engine ends the run right
    /// after the write, so the next instruction already sees the new value. The default ignores
    /// it (the Unicorn backend keeps QEMU's own reset `CCR`).
    fn set_ccr(&mut self, ccr: u32) {
        let _ = ccr;
    }
}

/// Why [`Cpu::run`] returned.
///
/// "Retired" means the instruction completed and PC moved past it. Unless a variant says
/// otherwise, every other register already holds the instruction's results.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Exit {
    /// The instruction budget was used (see [`Cpu::run`]).
    Budget,
    /// The bus asked for the run loop ([`Bus::take_yield`]). Backends that cannot count every
    /// instruction (Unicorn) may have retired some instructions they do not report.
    Yield,
    /// `SVC #imm` retired; PC is the return address the exception frame needs.
    Svc(u8),
    /// `BKPT #imm` did not retire; PC points at it.
    Breakpoint(u8),
    /// `WFI` retired. The engine skips time to the next event or pending interrupt.
    WaitForInterrupt,
    /// `WFE` retired. The engine skips time to the next event, interrupt or `SEV`.
    WaitForEvent,
    /// The instruction at PC faulted and did not retire. The engine escalates it to the matching
    /// fault exception (BusFault, UsageFault or HardFault).
    Fault(Fault),
}

/// A synchronous fault raised by one instruction. Each maps to a `CFSR` status bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum Fault {
    /// A data access or fetch failed (`PRECISERR`, `IBUSERR`).
    #[error(transparent)]
    Bus(#[from] BusFault),
    /// The encoding is undefined or not implemented by ARMv7-M (`UNDEFINSTR`).
    #[error("undefined instruction 0x{0:08X}")]
    Undefined(u32),
    /// An instruction ran with `EPSR.T` clear, e.g. after `BX` to an even address (`INVSTATE`).
    #[error("instruction executed with the Thumb bit clear")]
    InvalidState,
    /// An access that must be aligned was not (`LDM`, `STM`, `LDREX`, or any access with
    /// `CCR.UNALIGN_TRP` set) (`UNALIGNED`).
    #[error("unaligned access at 0x{0:08X}")]
    Unaligned(u32),
    /// `SDIV` or `UDIV` by zero with `CCR.DIV_0_TRP` set (`DIVBYZERO`).
    #[error("division by zero")]
    DivideByZero,
    /// The backend reported an exception it cannot classify (its own number, e.g. Unicorn's).
    #[error("unclassified CPU exception {0}")]
    Unknown(u32),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Access, BusFaultKind, Lane, Size};

    /// 64 KiB of RAM at address 0 that requests a yield on every data write.
    struct Ram {
        bytes: Vec<u8>,
        yield_pending: bool,
    }

    impl Ram {
        fn with_code(halfwords: &[u16]) -> Self {
            let mut bytes = vec![0; 0x1_0000];
            for (i, h) in halfwords.iter().enumerate() {
                bytes[2 * i..2 * i + 2].copy_from_slice(&h.to_le_bytes());
            }
            Self { bytes, yield_pending: false }
        }

        fn word(&self, addr: u32, access: Access) -> Result<u32, BusFault> {
            let i = addr as usize;
            let raw = self.bytes.get(i..i + 4).ok_or(BusFault { addr, access, kind: BusFaultKind::Unmapped })?;
            Ok(u32::from_le_bytes(raw.try_into().expect("4 bytes")))
        }
    }

    impl Bus for Ram {
        fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
            let lane = Lane::new(addr, size);
            Ok(lane.extract(self.word(lane.word, Access::Read)?))
        }

        fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
            let lane = Lane::new(addr, size);
            let merged = lane.merge(self.word(lane.word, Access::Write)?, value);
            self.bytes[lane.word as usize..][..4].copy_from_slice(&merged.to_le_bytes());
            self.yield_pending = true;
            Ok(())
        }

        fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
            let lane = Lane::new(addr, Size::Half);
            Ok(lane.extract(self.word(lane.word, Access::Fetch)?) as u16)
        }

        fn take_yield(&mut self) -> bool {
            std::mem::take(&mut self.yield_pending)
        }

        fn exception_return(&mut self, _regs: &mut CoreRegs, _exc_return: u32) {}
    }

    /// A toy core that knows four encodings, enough to exercise the `Exit` contract.
    #[derive(Default)]
    struct ToyCore {
        regs: CoreRegs,
        retired: u64,
    }

    impl Cpu for ToyCore {
        fn reset(&mut self, sp: u32, reset_vector: u32) {
            self.regs = CoreRegs::at_reset(sp, reset_vector);
        }

        fn run(&mut self, bus: &mut dyn Bus, max_instr: u32) -> Exit {
            for _ in 0..max_instr {
                let pc = self.regs.pc;
                let op = match bus.fetch(pc) {
                    Ok(op) => op,
                    Err(fault) => return Exit::Fault(fault.into()),
                };
                match op {
                    0xBF00 => {} // NOP
                    0xBF30 => {
                        self.regs.pc += 2;
                        self.retired += 1;
                        return Exit::WaitForInterrupt;
                    }
                    0x6008 => {
                        // STR r0, [r1]
                        if let Err(fault) = bus.write(self.regs.r[1], Size::Word, self.regs.r[0]) {
                            return Exit::Fault(fault.into());
                        }
                    }
                    op if op >> 8 == 0xBE => return Exit::Breakpoint(op as u8),
                    op => return Exit::Fault(Fault::Undefined(u32::from(op))),
                }
                self.regs.pc += 2;
                self.retired += 1;
                if bus.take_yield() {
                    return Exit::Yield;
                }
            }
            Exit::Budget
        }

        fn regs(&self) -> &CoreRegs {
            &self.regs
        }

        fn regs_mut(&mut self) -> &mut CoreRegs {
            &mut self.regs
        }

        fn retired(&self) -> u64 {
            self.retired
        }

        fn watch(&mut self, _addrs: &[u32]) {}

        fn invalidate(&mut self, _addr: u32, _len: u32) {}
    }

    #[test]
    fn exit_contract_through_a_trait_object() {
        let mut bus = Ram::with_code(&[0xBF00, 0xBF00, 0x6008, 0xBF00, 0xBF30, 0xBE2A, 0xFFFF]);
        let mut cpu: Box<dyn Cpu> = Box::new(ToyCore::default());
        cpu.reset(0x8000, 0x0001);
        cpu.regs_mut().r[0] = 0xCAFE_F00D;
        cpu.regs_mut().r[1] = 0x4000;

        assert_eq!(cpu.run(&mut bus, 1), Exit::Budget);
        assert_eq!((cpu.regs().pc, cpu.retired()), (2, 1));

        // The store retires, then the bus takes the run loop back.
        assert_eq!(cpu.run(&mut bus, 100), Exit::Yield);
        assert_eq!((cpu.regs().pc, cpu.retired()), (6, 3));
        assert_eq!(bus.read(0x4000, Size::Word), Ok(0xCAFE_F00D));

        assert_eq!(cpu.run(&mut bus, 100), Exit::WaitForInterrupt);
        assert_eq!(cpu.regs().pc, 10);

        // BKPT does not retire: PC stays on it and a second run stops there again.
        assert_eq!(cpu.run(&mut bus, 100), Exit::Breakpoint(0x2A));
        assert_eq!(cpu.run(&mut bus, 100), Exit::Breakpoint(0x2A));
        assert_eq!(cpu.retired(), 5);

        cpu.regs_mut().pc = 12;
        assert_eq!(cpu.run(&mut bus, 100), Exit::Fault(Fault::Undefined(0xFFFF)));
        cpu.regs_mut().pc = 0x2_0000;
        let fault = BusFault { addr: 0x2_0000, access: Access::Fetch, kind: BusFaultKind::Unmapped };
        assert_eq!(cpu.run(&mut bus, 100), Exit::Fault(Fault::Bus(fault)));
    }

    #[test]
    fn fault_messages() {
        assert_eq!(Fault::Undefined(0xDE00).to_string(), "undefined instruction 0x0000DE00");
        let bus = BusFault { addr: 0x4000_7000, access: Access::Write, kind: BusFaultKind::ReadOnly };
        assert_eq!(Fault::from(bus).to_string(), bus.to_string());
    }
}
