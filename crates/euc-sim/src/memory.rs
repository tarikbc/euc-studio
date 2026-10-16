//! Plain memory: the code part of flash and SRAM. CPU backends execute from these buffers in
//! place; the engine reads and writes them for exception frames, DMA and reports.

use euc_core::{Access, Bus, BusFault, BusFaultKind, CoreRegs, Lane, Size};

use crate::config::{MemoryMap, Span};

/// Flash below `code_end` (with the image loaded, the rest 0xFF) and SRAM.
pub struct Memory {
    code: Span,
    sram_span: Span,
    /// Flash from its base up to `code_end`.
    pub(crate) flash: Box<[u8]>,
    /// SRAM.
    pub(crate) sram: Box<[u8]>,
}

impl Memory {
    /// Erased flash with `image` at `load`, and zeroed SRAM.
    pub fn new(map: &MemoryMap, image: &[u8], load: u32) -> Result<Self, String> {
        let code = Span { base: map.flash.base, size: map.code_end - map.flash.base };
        let start = load.checked_sub(code.base).filter(|_| code.contains(load)).ok_or_else(|| {
            format!(
                "image base 0x{load:08X} is outside the flash code window 0x{:08X}..0x{:08X}",
                code.base,
                code.end()
            )
        })? as usize;
        let mut flash = vec![0xFF; code.size as usize].into_boxed_slice();
        let end = start + image.len();
        if end > flash.len() {
            return Err(format!("a {} byte image at 0x{load:08X} runs past 0x{:08X}", image.len(), code.end()));
        }
        flash[start..end].copy_from_slice(image);
        Ok(Self { code, sram_span: map.sram, flash, sram: vec![0; map.sram.size as usize].into_boxed_slice() })
    }

    /// The flash code window.
    pub const fn code_span(&self) -> Span {
        self.code
    }

    /// The SRAM span.
    pub const fn sram_span(&self) -> Span {
        self.sram_span
    }

    /// True when `addr` is plain memory.
    pub const fn contains(&self, addr: u32) -> bool {
        self.code.contains(addr) || self.sram_span.contains(addr)
    }

    /// Reads `size` bytes at `addr` (naturally aligned) from flash or SRAM.
    pub fn read(&self, addr: u32, size: Size) -> Option<u32> {
        let (bytes, base) = self.region(addr)?;
        let lane = Lane::new(addr, size);
        let at = (lane.word - base) as usize;
        let word = u32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?);
        Some(lane.extract(word))
    }

    /// Writes `size` bytes at `addr` in SRAM. Flash is read-only here: it only changes through
    /// the flash controller, which the code window never needs.
    pub fn write(&mut self, addr: u32, size: Size, value: u32) -> bool {
        if !self.sram_span.contains(addr) {
            return false;
        }
        let at = (addr - self.sram_span.base) as usize;
        let n = size.bytes() as usize;
        if at + n > self.sram.len() {
            return false;
        }
        self.sram[at..at + n].copy_from_slice(&value.to_le_bytes()[..n]);
        true
    }

    /// `len` bytes starting at `addr`, if they are all in one region.
    pub fn slice(&self, addr: u32, len: usize) -> Option<&[u8]> {
        let (bytes, base) = self.region(addr)?;
        let at = (addr - base) as usize;
        bytes.get(at..at + len)
    }

    /// Clears SRAM (a power cycle with no backup cell).
    pub fn clear_sram(&mut self) {
        self.sram.fill(0);
    }

    fn region(&self, addr: u32) -> Option<(&[u8], u32)> {
        if self.code.contains(addr) {
            Some((&self.flash, self.code.base))
        } else if self.sram_span.contains(addr) {
            Some((&self.sram, self.sram_span.base))
        } else {
            None
        }
    }
}

/// [`Memory`] as a bus, for the exception unit's stacking and vector reads.
pub struct MemoryBus<'a>(pub &'a mut Memory);

impl Bus for MemoryBus<'_> {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        self.0.read(addr, size).ok_or(BusFault { addr, access: Access::Read, kind: BusFaultKind::Unmapped })
    }

    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        if self.0.write(addr, size, value) {
            Ok(())
        } else {
            let kind = if self.0.contains(addr) { BusFaultKind::ReadOnly } else { BusFaultKind::Unmapped };
            Err(BusFault { addr, access: Access::Write, kind })
        }
    }

    fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
        self.0.read(addr, Size::Half).map(|v| v as u16).ok_or(BusFault {
            addr,
            access: Access::Fetch,
            kind: BusFaultKind::Unmapped,
        })
    }

    fn exception_return(&mut self, _regs: &mut CoreRegs, _exc_return: u32) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map() -> MemoryMap {
        MemoryMap {
            flash: Span { base: 0x0800_0000, size: 0x8_0000 },
            code_end: 0x0800_E000,
            sram: Span { base: 0x2000_0000, size: 0x1_0000 },
            sram_bitband: 0x2200_0000,
            periph: Span { base: 0x4000_0000, size: 0x3_0000 },
            periph_bitband: 0x4200_0000,
            ppb: Span { base: 0xE000_0000, size: 0x10_0000 },
        }
    }

    #[test]
    fn image_lands_at_its_load_address_in_erased_flash() {
        let mem = Memory::new(&map(), &[0x11, 0x22, 0x33, 0x44], 0x0800_3000).unwrap();
        assert_eq!(mem.read(0x0800_3000, Size::Word), Some(0x4433_2211));
        assert_eq!(mem.read(0x0800_2FFC, Size::Word), Some(0xFFFF_FFFF), "no bootloader: erased");
        assert_eq!(mem.read(0x0800_E000, Size::Byte), None, "data flash is not plain memory");
        assert!(Memory::new(&map(), &[0; 16], 0x0800_DFF8).is_err());
    }

    #[test]
    fn only_sram_is_writable() {
        let mut mem = Memory::new(&map(), &[], 0x0800_3000).unwrap();
        assert!(mem.write(0x2000_0010, Size::Half, 0xBEEF));
        assert_eq!(mem.read(0x2000_0010, Size::Word), Some(0xBEEF));
        let mut bus = MemoryBus(&mut mem);
        let fault = bus.write(0x0800_3000, Size::Word, 0).unwrap_err();
        assert_eq!(fault.kind, BusFaultKind::ReadOnly);
        assert_eq!(bus.read(0x3000_0000, Size::Word).unwrap_err().kind, BusFaultKind::Unmapped);
    }
}
