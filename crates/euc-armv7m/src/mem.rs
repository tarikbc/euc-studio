//! The guest address space as the core sees it: plain memory it reads, writes and executes in
//! place, MMIO windows it sends to the [`euc_core::Bus`], and a decode cache per plain region.
//!
//! This is the crate's only `unsafe` code. The engine lends plain memory as raw pointers
//! ([`RegionKind::Memory`]); the core reads and writes them with unaligned raw-pointer accesses
//! after a bounds check, and never holds a Rust reference into them, so the engine can touch the
//! same bytes from inside bus callbacks (exception stacking, DMA) without aliasing a borrow.

#![allow(unsafe_code)]
// Region lookups run on every load, store and fetch; see the note in `exec`.
#![allow(clippy::inline_always)]

use std::ptr::NonNull;

use euc_core::{Region, RegionKind};
use thiserror::Error;

use crate::op::Decoded;

/// Bytes per bit of [`Code::blocks`], the granularity of self-modifying-code detection.
const BLOCK_SHIFT: u32 = 8;

/// The memory layout cannot be used.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum LayoutError {
    /// A region is empty, smaller than a word, or not word aligned.
    #[error("region at 0x{0:08X} must be at least 4 bytes, word aligned in base and size")]
    BadRegion(u32),
    /// A region runs past the end of the 4 GiB address space.
    #[error("region at 0x{0:08X} wraps past 0xFFFFFFFF")]
    Wraps(u32),
    /// Two regions overlap.
    #[error("regions at 0x{0:08X} and 0x{1:08X} overlap")]
    Overlap(u32, u32),
}

/// One plain-memory region.
pub(crate) struct Ram {
    pub(crate) base: u32,
    pub(crate) size: u32,
    ptr: NonNull<u8>,
    pub(crate) writable: bool,
}

impl Ram {
    /// Offset of an `n`-byte access at `addr` if it lies entirely inside this region.
    #[inline(always)]
    pub(crate) const fn offset(&self, addr: u32, n: u32) -> Option<u32> {
        let off = addr.wrapping_sub(self.base);
        if off < self.size && n <= self.size - off { Some(off) } else { None }
    }

    /// Reads `n` (1, 2 or 4) bytes at `off`, little-endian, zero-extended.
    #[inline(always)]
    pub(crate) fn read(&self, off: u32, n: u32) -> u32 {
        debug_assert!(off.checked_add(n).is_some_and(|end| end <= self.size));
        let p = self.ptr.as_ptr();
        // SAFETY: `off + n <= size` (every caller gets `off` from `Ram::offset(addr, n)`), and
        // `Armv7m::new`'s contract keeps `size` bytes at `ptr` valid for the core's lifetime.
        // Raw unaligned reads create no reference, so engine accesses between them are fine.
        unsafe {
            match n {
                1 => u32::from(p.add(off as usize).read()),
                2 => u32::from(u16::from_le(p.add(off as usize).cast::<u16>().read_unaligned())),
                _ => u32::from_le(p.add(off as usize).cast::<u32>().read_unaligned()),
            }
        }
    }

    /// Writes the low `n` (1, 2 or 4) bytes of `value` at `off`, little-endian.
    #[inline(always)]
    pub(crate) fn write(&self, off: u32, n: u32, value: u32) {
        debug_assert!(off.checked_add(n).is_some_and(|end| end <= self.size));
        let p = self.ptr.as_ptr();
        // SAFETY: as in `read`; the region is writable (callers check `writable` first) and
        // the engine's contract allows the core to write it during a run.
        unsafe {
            match n {
                1 => p.add(off as usize).write(value as u8),
                2 => p.add(off as usize).cast::<u16>().write_unaligned((value as u16).to_le()),
                _ => p.add(off as usize).cast::<u32>().write_unaligned(value.to_le()),
            }
        }
    }
}

/// The decode cache of one plain region: one slot per halfword, allocated on first execution.
#[derive(Default)]
pub(crate) struct Code {
    pub(crate) slots: Box<[Decoded]>,
    /// One bit per 256-byte block that has held a decoded slot since the last full flush.
    blocks: Vec<u64>,
    /// True once anything was decoded here; stores skip the block check until then.
    pub(crate) live: bool,
}

impl Code {
    /// Allocates the slots for a region of `size` bytes.
    fn allocate(&mut self, size: u32) {
        self.slots = vec![Decoded::EMPTY; (size / 2) as usize].into_boxed_slice();
        self.blocks = vec![0; (size >> BLOCK_SHIFT) as usize / 64 + 1];
    }

    /// Marks the block holding offset `off` as holding code.
    pub(crate) fn mark(&mut self, off: u32) {
        let block = (off >> BLOCK_SHIFT) as usize;
        self.blocks[block / 64] |= 1 << (block % 64);
        self.live = true;
    }

    /// True when the block holding `off` may hold decoded slots.
    #[inline]
    pub(crate) fn holds_code(&self, off: u32) -> bool {
        let block = (off >> BLOCK_SHIFT) as usize;
        self.blocks.get(block / 64).is_some_and(|word| word >> (block % 64) & 1 == 1)
    }

    /// Empties the slots of `off..off + len` (and the halfword before, which may be the first half
    /// of a 32-bit instruction that overlaps the range).
    pub(crate) fn clear(&mut self, off: u32, len: u32) {
        if self.slots.is_empty() {
            return;
        }
        let first = (off.saturating_sub(2) / 2) as usize;
        let last = ((u64::from(off) + u64::from(len)).div_ceil(2) as usize).min(self.slots.len());
        if first < last {
            self.slots[first..last].fill(Decoded::EMPTY);
        }
    }
}

/// Plain regions, MMIO windows, and the decode caches.
pub(crate) struct Memory {
    pub(crate) rams: Vec<Ram>,
    pub(crate) code: Vec<Code>,
    mmio: Vec<(u32, u32)>,
}

impl Memory {
    /// The layout of `regions`, checked.
    ///
    /// # Safety
    ///
    /// As for [`crate::Armv7m::new`]: every [`RegionKind::Memory`] pointer is valid for reads
    /// and writes of its region's bytes for as long as the returned value lives.
    pub(crate) unsafe fn new(regions: &[Region]) -> Result<Self, LayoutError> {
        let mut spans: Vec<(u64, u64, u32)> = Vec::new();
        for region in regions {
            if region.size < 4 || region.base % 4 != 0 || region.size % 4 != 0 {
                return Err(LayoutError::BadRegion(region.base));
            }
            let end = u64::from(region.base) + u64::from(region.size);
            if end > 1 << 32 {
                return Err(LayoutError::Wraps(region.base));
            }
            if let Some(&(_, _, other)) = spans.iter().find(|&&(b, e, _)| u64::from(region.base) < e && b < end) {
                return Err(LayoutError::Overlap(other, region.base));
            }
            spans.push((u64::from(region.base), end, region.base));
        }
        let mut rams = Vec::new();
        let mut mmio = Vec::new();
        for region in regions {
            match region.kind {
                RegionKind::Memory { ptr, writable } => {
                    rams.push(Ram { base: region.base, size: region.size, ptr, writable });
                }
                RegionKind::Mmio => mmio.push((region.base, region.size)),
            }
        }
        // Data accesses try regions in order, and most of them hit SRAM.
        rams.sort_by_key(|ram| !ram.writable);
        let code = rams.iter().map(|_| Code::default()).collect();
        Ok(Self { rams, code, mmio })
    }

    /// The plain region holding an `n`-byte access at `addr`, with its offset.
    #[inline(always)]
    pub(crate) fn find(&self, addr: u32, n: u32) -> Option<(usize, u32)> {
        self.rams.iter().enumerate().find_map(|(i, ram)| ram.offset(addr, n).map(|off| (i, off)))
    }

    /// True when `addr` is in an MMIO window.
    pub(crate) fn is_mmio(&self, addr: u32) -> bool {
        self.mmio.iter().any(|&(base, size)| addr.wrapping_sub(base) < size)
    }

    /// The decode cache of region `i`, allocated on first use.
    pub(crate) fn code(&mut self, i: usize) -> &mut Code {
        let code = &mut self.code[i];
        if code.slots.is_empty() {
            code.allocate(self.rams[i].size);
        }
        code
    }

    /// Empties every decode slot that may hold bytes of `addr..addr + len`.
    pub(crate) fn invalidate(&mut self, addr: u32, len: u32) {
        let (start, end) = (u64::from(addr), u64::from(addr) + u64::from(len));
        for (ram, code) in self.rams.iter().zip(&mut self.code) {
            let (base, top) = (u64::from(ram.base), u64::from(ram.base) + u64::from(ram.size));
            if start < top && base < end {
                let off = start.max(base) - base;
                code.clear(off as u32, (end.min(top) - base - off) as u32);
            }
        }
    }
}
