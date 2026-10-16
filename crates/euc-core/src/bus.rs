use std::fmt;

use thiserror::Error;

/// Width of one bus transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Size {
    /// 8 bits (`LDRB`, `STRB`).
    Byte = 1,
    /// 16 bits (`LDRH`, `STRH`, a Thumb fetch).
    Half = 2,
    /// 32 bits (`LDR`, `STR`, `LDM`, `STM`, exception stacking).
    Word = 4,
}

impl Size {
    /// Width in bytes.
    pub const fn bytes(self) -> u32 {
        self as u32
    }

    /// Mask of the low bits this width covers.
    pub const fn mask(self) -> u32 {
        match self {
            Self::Byte => 0xFF,
            Self::Half => 0xFFFF,
            Self::Word => 0xFFFF_FFFF,
        }
    }
}

/// Where a byte or halfword access lands inside its aligned 32-bit word.
///
/// Peripheral models keep registers as aligned words, but firmware often stores bytes and
/// halfwords to them (`STRB` to a USART data register, `STRH` to a timer compare register). A
/// lane turns any access into the word address, the value moved into its byte lanes and a mask
/// of those lanes, so a model merges only the bytes the CPU touched and never sees partial words.
/// This is the same contract as the Python prototype (`write(off, val << shift, mask << shift)`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Lane {
    /// Address of the aligned word that holds the access.
    pub word: u32,
    /// Bit position of the access inside that word.
    pub shift: u32,
    /// The bits the access covers, already shifted into place.
    pub mask: u32,
}

impl Lane {
    /// The lane of an access of `size` at `addr`.
    ///
    /// `addr` must be naturally aligned for `size`; CPU backends split unaligned accesses into
    /// aligned ones first, as the Cortex-M3 bus interface does. Checked in debug builds.
    pub const fn new(addr: u32, size: Size) -> Self {
        debug_assert!(addr.is_multiple_of(size.bytes()), "bus access is not naturally aligned");
        let shift = (addr & 3) * 8;
        Self { word: addr & !3, shift, mask: size.mask() << shift }
    }

    /// The value this access reads out of `word`.
    pub const fn extract(self, word: u32) -> u32 {
        (word & self.mask) >> self.shift
    }

    /// `value` moved into this lane, other bits zero.
    pub const fn place(self, value: u32) -> u32 {
        (value << self.shift) & self.mask
    }

    /// `old` with this lane replaced by `value`.
    pub const fn merge(self, old: u32, value: u32) -> u32 {
        (old & !self.mask) | self.place(value)
    }
}

/// Kind of transfer that faulted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Access {
    /// Data load.
    Read,
    /// Data store.
    Write,
    /// Instruction fetch.
    Fetch,
}

/// Why the bus refused a transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BusFaultKind {
    /// Nothing is mapped at the address.
    Unmapped,
    /// A store to memory that only reads (code flash outside the flash controller's program cycle).
    ReadOnly,
    /// A fetch from an execute-never region (peripherals, the private peripheral bus).
    NoExecute,
}

impl fmt::Display for Access {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Read => "read",
            Self::Write => "write",
            Self::Fetch => "fetch",
        })
    }
}

impl fmt::Display for BusFaultKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Unmapped => "nothing is mapped there",
            Self::ReadOnly => "the memory is read-only",
            Self::NoExecute => "the region is execute-never",
        })
    }
}

/// A transfer the bus could not complete. On hardware this raises a BusFault or MemManage fault.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Error)]
#[error("bus fault on {access} at 0x{addr:08X}: {kind}")]
pub struct BusFault {
    /// Address of the faulting transfer.
    pub addr: u32,
    /// What the CPU was doing.
    pub access: Access,
    /// Why it failed.
    pub kind: BusFaultKind,
}

/// The system bus as a CPU backend sees it: memory, peripherals and bit-band aliases behind one
/// address space.
///
/// Every data access goes through [`read`](Self::read) and [`write`](Self::write), already split
/// into naturally aligned transfers (see [`Lane`]). Reads take `&mut self` because peripheral
/// reads have side effects (clear-on-read flags, FIFO pops).
///
/// ```
/// use euc_core::{Access, Bus, BusFault, BusFaultKind, Lane, Size};
///
/// /// 1 KiB of RAM at 0x2000_0000.
/// struct Ram(Vec<u32>);
///
/// impl Ram {
///     fn slot(&mut self, addr: u32, access: Access) -> Result<&mut u32, BusFault> {
///         let index = addr.wrapping_sub(0x2000_0000) as usize / 4;
///         let fault = BusFault { addr, access, kind: BusFaultKind::Unmapped };
///         self.0.get_mut(index).ok_or(fault)
///     }
/// }
///
/// impl Bus for Ram {
///     fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
///         let lane = Lane::new(addr, size);
///         Ok(lane.extract(*self.slot(lane.word, Access::Read)?))
///     }
///     fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
///         let lane = Lane::new(addr, size);
///         let word = self.slot(lane.word, Access::Write)?;
///         *word = lane.merge(*word, value);
///         Ok(())
///     }
///     fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
///         let lane = Lane::new(addr, Size::Half);
///         Ok(lane.extract(*self.slot(lane.word, Access::Fetch)?) as u16)
///     }
///     fn exception_return(&mut self, _regs: &mut euc_core::CoreRegs, _exc_return: u32) {}
/// }
///
/// let mut ram = Ram(vec![0; 256]);
/// ram.write(0x2000_0000, Size::Word, 0x1122_3344)?;
/// ram.write(0x2000_0001, Size::Byte, 0xAA)?;
/// assert_eq!(ram.read(0x2000_0000, Size::Word)?, 0x1122_AA44);
/// assert_eq!(ram.read(0x2000_0002, Size::Half)?, 0x1122);
/// assert!(ram.read(0x3000_0000, Size::Word).is_err());
/// # Ok::<(), BusFault>(())
/// ```
pub trait Bus {
    /// Loads `size` bytes from `addr` (naturally aligned). The value is zero-extended.
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault>;

    /// Stores the low `size` bytes of `value` at `addr` (naturally aligned).
    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault>;

    /// Fetches one Thumb halfword at the even address `addr`.
    ///
    /// Separate from [`read`](Self::read) because a fetch never has side effects, is never traced
    /// as a data access, and faults in execute-never regions where a data read would succeed.
    fn fetch(&mut self, addr: u32) -> Result<u16, BusFault>;

    /// True once per request when the bus needs the run loop back before the next instruction.
    ///
    /// Backends call this after every instruction that made a data access and end
    /// [`Cpu::run`](crate::Cpu::run) with [`Exit::Yield`](crate::Exit::Yield) when it returns
    /// true. A bus requests it when an access needs the engine at once: a detected poll loop to
    /// fast-forward, a reset request, an exception pended by a register write.
    fn take_yield(&mut self) -> bool {
        false
    }

    /// A handler loaded an `EXC_RETURN` value into PC (`BX LR`, `POP {.., PC}`, `LDR PC`).
    ///
    /// The backend calls this in the middle of [`Cpu::run`](crate::Cpu::run), with `regs`
    /// current, and resumes at `regs.pc` afterwards without ending the run: the engine's
    /// exception unit unstacks the frame or tail-chains into the next handler. Handling it inline
    /// keeps quantum accounting identical across backends.
    fn exception_return(&mut self, regs: &mut crate::CoreRegs, exc_return: u32);

    /// The address of the instruction making the next bus access, for traces and diagnostics
    /// (a poll loop's PC). Backends that know it cheaply report it; the default ignores it.
    fn note_pc(&mut self, pc: u32) {
        let _ = pc;
    }

    /// Execution reached an address passed to [`Cpu::watch`](crate::Cpu::watch), for the first
    /// time. Boot milestones use it; the default ignores it.
    fn reached(&mut self, addr: u32) {
        let _ = addr;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes() {
        assert_eq!(Size::Byte.bytes(), 1);
        assert_eq!(Size::Half.bytes(), 2);
        assert_eq!(Size::Word.bytes(), 4);
        assert_eq!(Size::Half.mask(), 0xFFFF);
    }

    #[test]
    fn byte_lanes() {
        for offset in 0..4 {
            let lane = Lane::new(0x4001_3804 + offset, Size::Byte);
            assert_eq!(lane.word, 0x4001_3804);
            assert_eq!(lane.shift, offset * 8);
            assert_eq!(lane.mask, 0xFF << (offset * 8));
            assert_eq!(lane.extract(0x4433_2211), 0x11 * (offset + 1));
        }
    }

    #[test]
    fn halfword_lanes() {
        let high = Lane::new(0x4001_3402, Size::Half);
        assert_eq!((high.word, high.shift, high.mask), (0x4001_3400, 16, 0xFFFF_0000));
        assert_eq!(high.extract(0xBEEF_1234), 0xBEEF);
        assert_eq!(high.merge(0xBEEF_1234, 0xCAFE), 0xCAFE_1234);
    }

    #[test]
    fn word_lane_is_the_whole_word() {
        let lane = Lane::new(0x2000_0010, Size::Word);
        assert_eq!((lane.word, lane.shift, lane.mask), (0x2000_0010, 0, u32::MAX));
        assert_eq!(lane.merge(1, 0xDEAD_BEEF), 0xDEAD_BEEF);
    }

    #[test]
    fn place_drops_bits_outside_the_lane() {
        let lane = Lane::new(0x1001, Size::Byte);
        assert_eq!(lane.place(0x1FF), 0xFF00);
        assert_eq!(lane.merge(0xFFFF_FFFF, 0), 0xFFFF_00FF);
    }

    #[test]
    #[should_panic(expected = "not naturally aligned")]
    #[cfg(debug_assertions)]
    fn unaligned_lane_panics_in_debug() {
        let _ = Lane::new(0x1003, Size::Half);
    }

    #[test]
    fn fault_message_names_the_address() {
        let fault = BusFault { addr: 0x6000_0000, access: Access::Read, kind: BusFaultKind::Unmapped };
        assert_eq!(fault.to_string(), "bus fault on read at 0x60000000: nothing is mapped there");
    }
}
