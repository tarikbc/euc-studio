//! Embedded flash: the memory interface registers (RM0008 §3.3.3, programming rules from PM0075)
//! and the flash array they program.

use euc_core::{Build, ConfigError, Ctx, DeviceId, Peripheral, ResetKind};
use serde::Deserialize;

use crate::util::merge;

const ACR: u32 = 0x00;
const KEYR: u32 = 0x04;
const OPTKEYR: u32 = 0x08;
const SR: u32 = 0x0C;
const CR: u32 = 0x10;
const AR: u32 = 0x14;
const OBR: u32 = 0x1C;
const WRPR: u32 = 0x20;

/// Unlock keys, in order.
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

// SR
const PGERR: u32 = 1 << 2;
const WRPRTERR: u32 = 1 << 4;
const EOP: u32 = 1 << 5;
// CR
const PG: u32 = 1 << 0;
const PER: u32 = 1 << 1;
const MER: u32 = 1 << 2;
const STRT: u32 = 1 << 6;
const LOCK: u32 = 1 << 7;
const OPTWRE: u32 = 1 << 9;
const ERRIE: u32 = 1 << 10;
const EOPIE: u32 = 1 << 12;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CtlConfig {
    /// The flash array instance this interface programs.
    array: String,
    /// Erase page size in bytes: 2 KiB on high-density parts.
    #[serde(default = "default_page")]
    page_size: u32,
}

const fn default_page() -> u32 {
    0x800
}

/// The flash memory interface (FPEC): wait states, unlock, page erase, half-word programming.
///
/// `KEYR` takes the two keys in order to clear `LOCK`; a wrong key locks the interface until the
/// next reset, as on hardware, and is reported. With `PER` set, `STRT` erases the page that
/// contains `AR` in the flash array (pages in the plain code window are not erasable here and are
/// reported); `MER` (mass erase) is reported and ignored. With `PG` set, half-word stores to the
/// array program it (see [`FlashArray`]). Operations complete at once: `BSY` stays 0 and `EOP`
/// sets. `EOP`, `PGERR` and `WRPRTERR` clear by writing 1. Interrupt role `irq` (`EOPIE`, `ERRIE`).
///
/// Not modeled: option bytes (`OBR` reads "no protection", `WRPR` all ones, `OPTKEYR` only sets
/// `OPTWRE`), read protection, wait-state timing, the busy time of operations.
#[derive(Debug, Clone)]
pub struct FlashCtl {
    array: DeviceId,
    page_size: u32,
    irq: Option<(u16, bool)>,
    acr: u32,
    sr: u32,
    cr: u32,
    ar: u32,
    /// Keys accepted so far in the unlock sequence.
    keys: u8,
    /// A wrong key locked the interface until reset.
    locked_out: bool,
    opt_keys: u8,
}

impl FlashCtl {
    /// The interface after reset (locked), programming `array`.
    pub fn new(array: DeviceId, page_size: u32, irq: Option<u16>) -> Self {
        Self {
            array,
            page_size,
            irq: irq.map(|irq| (irq, false)),
            acr: 0x30,
            sr: 0,
            cr: LOCK,
            ar: 0,
            keys: 0,
            locked_out: false,
            opt_keys: 0,
        }
    }

    /// Builds the `stm32f1.flash` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: CtlConfig = build.params()?;
        if !config.page_size.is_power_of_two() {
            return Err(build.error(format!("page_size 0x{:X} is not a power of two", config.page_size)));
        }
        Ok(Box::new(Self::new(build.id_of(&config.array)?, config.page_size, build.irq("irq").ok())))
    }

    /// True while a half-word store to the array programs it (`PG` set, unlocked).
    pub const fn programming(&self) -> bool {
        self.cr & (PG | LOCK) == PG
    }

    /// Records the outcome of a program operation the array carried out.
    fn finish(&mut self, ok: bool) {
        self.sr |= if ok { EOP } else { PGERR };
    }

    fn write_keyr(&mut self, key: u32, ctx: &mut Ctx<'_>) {
        if self.locked_out {
            return;
        }
        match (self.keys, key) {
            (0, KEY1) => self.keys = 1,
            (1, KEY2) => {
                self.keys = 0;
                self.cr &= !LOCK;
                ctx.log("FLASH unlocked");
            }
            _ => {
                self.locked_out = true;
                self.cr |= LOCK;
                ctx.blocker("flash-key", format!("wrong FLASH_KEYR key 0x{key:08X}: locked until reset"), false);
            }
        }
    }

    fn write_cr(&mut self, new: u32, ctx: &mut Ctx<'_>) {
        if self.cr & LOCK != 0 {
            if new & !LOCK & 0x1677 != 0 {
                ctx.blocker("flash-locked", format!("FLASH_CR write 0x{new:X} while locked (ignored)"), false);
            }
            return;
        }
        let mut cr = new & (PG | PER | MER | 1 << 4 | 1 << 5 | LOCK | ERRIE | EOPIE) | self.cr & OPTWRE;
        if new & STRT != 0 {
            if cr & PER != 0 {
                self.erase_page(ctx);
            } else if cr & MER != 0 {
                ctx.blocker("flash-mass-erase", "FLASH mass erase requested (ignored)", false);
            }
            self.sr |= EOP;
            cr &= !STRT;
        }
        if cr & LOCK != 0 {
            cr &= !OPTWRE;
        }
        self.cr = cr;
    }

    fn erase_page(&mut self, ctx: &mut Ctx<'_>) {
        let page = self.ar & !(self.page_size - 1);
        let page_size = self.page_size;
        let erased = ctx.peer::<FlashArray>(self.array).is_some_and(|array| array.erase(page, page_size));
        if erased {
            ctx.log(format!("FLASH page erase 0x{page:08X}"));
        } else {
            ctx.blocker(
                "flash-erase-outside-array",
                format!("page erase at 0x{page:08X} is outside the flash array (ignored)"),
                false,
            );
        }
    }

    fn update_irq(&mut self, ctx: &mut Ctx<'_>) {
        let Some((irq, level)) = &mut self.irq else { return };
        let high =
            self.sr & EOP != 0 && self.cr & EOPIE != 0 || self.sr & (PGERR | WRPRTERR) != 0 && self.cr & ERRIE != 0;
        if high != *level {
            *level = high;
            ctx.set_irq(*irq, high);
        }
    }
}

impl Peripheral for FlashCtl {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        *self = Self::new(self.array, self.page_size, self.irq.map(|(irq, _)| irq));
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        match off {
            // PRFTBS (bit 5) mirrors PRFTBE (bit 4).
            ACR => self.acr & !0x20 | (self.acr & 0x10) << 1,
            SR => self.sr,
            CR => self.cr,
            AR => self.ar,
            OBR => 0x03FF_FFFC,
            WRPR => u32::MAX,
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        match off {
            ACR => self.acr = merge(self.acr, value, mask) & 0x1F,
            KEYR => self.write_keyr(value & mask, ctx),
            OPTKEYR => {
                let key = value & mask;
                self.opt_keys = match (self.opt_keys, key) {
                    (0, KEY1) => 1,
                    (1, KEY2) if self.cr & LOCK == 0 => {
                        self.cr |= OPTWRE;
                        0
                    }
                    _ => 0,
                };
            }
            SR => self.sr &= !(value & mask & (PGERR | WRPRTERR | EOP)),
            CR => self.write_cr(merge(self.cr, value, mask), ctx),
            AR => self.ar = merge(self.ar, value, mask),
            _ => return,
        }
        self.update_irq(ctx);
    }

    fn reg_name(&self, off: u32) -> Option<&'static str> {
        Some(match off {
            ACR => "ACR",
            KEYR => "KEYR",
            OPTKEYR => "OPTKEYR",
            SR => "SR",
            CR => "CR",
            AR => "AR",
            OBR => "OBR",
            WRPR => "WRPR",
            _ => return None,
        })
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ArrayConfig {
    /// The memory interface (`stm32f1.flash`) whose `PG` bit gates programming.
    controller: String,
    /// Contents to place before power-on (calibration, settings), as little-endian half-words.
    #[serde(default)]
    seed: Vec<Seed>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Seed {
    addr: u32,
    halfwords: Vec<u16>,
}

/// The part of the flash array that firmware programs at run time (settings, calibration, the
/// bootloader request page), mapped above the plain code window.
///
/// Reads return the stored bytes; erased flash reads `0xFF`. A store programs the array only
/// while the memory interface has `PG` set, and only as a half-word: other sizes and stores over a
/// cell that is neither erased (`0xFFFF`) nor being cleared to `0x0000` set `PGERR` and change
/// nothing; a store without `PG` is reported and ignored (STM32F1 rules, PM0075 §2.3). Contents
/// survive every reset.
///
/// The interface's `EOP`/`PGERR` are updated through a peer call, so a program operation does not
/// raise the FLASH interrupt by itself (the Begode Extreme polls instead).
#[derive(Debug, Clone)]
pub struct FlashArray {
    controller: DeviceId,
    base: u32,
    bytes: Vec<u8>,
    programs: u64,
}

impl FlashArray {
    /// An erased array of `size` bytes at `base`, programmed through `controller`.
    pub fn new(controller: DeviceId, base: u32, size: u32) -> Self {
        Self { controller, base, bytes: vec![0xFF; size as usize], programs: 0 }
    }

    /// Builds the `stm32f1.flash-array` kind.
    pub fn build(build: &Build<'_>) -> Result<Box<dyn Peripheral>, ConfigError> {
        let config: ArrayConfig = build.params()?;
        let spec = build.spec();
        let base = spec.base.ok_or_else(|| build.error("a flash array needs a base address"))?;
        let mut array = Self::new(build.id_of(&config.controller)?, base, spec.size);
        for seed in &config.seed {
            for (i, &halfword) in seed.halfwords.iter().enumerate() {
                let addr = seed.addr + 2 * i as u32;
                let slot = array
                    .slot(addr, 2)
                    .ok_or_else(|| build.error(format!("seed address 0x{addr:08X} is outside the array")))?;
                array.bytes[slot..slot + 2].copy_from_slice(&halfword.to_le_bytes());
            }
        }
        Ok(Box::new(array))
    }

    /// The array's base address.
    pub const fn base(&self) -> u32 {
        self.base
    }

    /// The stored bytes, from [`base`](Self::base).
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Half-words programmed since creation.
    pub const fn programs(&self) -> u64 {
        self.programs
    }

    /// Byte index of `len` bytes at `addr`, if they are all inside the array.
    fn slot(&self, addr: u32, len: u32) -> Option<usize> {
        let start = addr.checked_sub(self.base)? as usize;
        (start + len as usize <= self.bytes.len()).then_some(start)
    }

    /// Erases `len` bytes at `addr` to `0xFF`. False when the range is not inside the array.
    pub fn erase(&mut self, addr: u32, len: u32) -> bool {
        let Some(start) = self.slot(addr, len) else { return false };
        self.bytes[start..start + len as usize].fill(0xFF);
        true
    }
}

impl Peripheral for FlashArray {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {}

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        let start = (off & !3) as usize;
        self.bytes
            .get(start..start + 4)
            .map_or(0xFFFF_FFFF, |word| u32::from_le_bytes([word[0], word[1], word[2], word[3]]))
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, ctx: &mut Ctx<'_>) {
        let addr = self.base + off + mask.trailing_zeros() / 8;
        let programming = ctx.peer::<FlashCtl>(self.controller).is_some_and(|ctl| ctl.programming());
        if !programming {
            ctx.blocker(
                "flash-write-no-pg",
                format!("store to flash 0x{addr:08X} with FLASH_CR.PG clear (ignored)"),
                false,
            );
            return;
        }
        let shift = mask.trailing_zeros();
        let ok = match (mask >> shift, self.slot(addr, 2)) {
            (0xFFFF, Some(i)) => {
                let new = (value >> shift) as u16;
                let old = u16::from_le_bytes([self.bytes[i], self.bytes[i + 1]]);
                let ok = old == 0xFFFF || new == 0;
                if ok {
                    self.bytes[i..i + 2].copy_from_slice(&new.to_le_bytes());
                    self.programs += 1;
                }
                ok
            }
            _ => false,
        };
        if let Some(ctl) = ctx.peer::<FlashCtl>(self.controller) {
            ctl.finish(ok);
        }
    }
}

#[cfg(test)]
mod tests {
    use euc_core::testing::TestHost;

    use super::*;

    const CTL: DeviceId = 1;
    const ARRAY: DeviceId = 2;
    const BASE: u32 = 0x0800_E000;

    fn unlocked() -> FlashCtl {
        let mut host = TestHost::new(72_000_000);
        let mut ctl = FlashCtl::new(ARRAY, 0x800, Some(4));
        ctl.write(KEYR, KEY1, u32::MAX, &mut host.ctx());
        ctl.write(KEYR, KEY2, u32::MAX, &mut host.ctx());
        ctl
    }

    fn ctl(host: &mut TestHost) -> &mut FlashCtl {
        (host.peers.get_mut(&CTL).unwrap().as_mut() as &mut dyn std::any::Any).downcast_mut().unwrap()
    }

    /// The array under test with an unlocked interface as its peer.
    fn setup(cr: u32) -> (TestHost, FlashArray) {
        let mut host = TestHost::new(72_000_000);
        let mut ctl = unlocked();
        ctl.write(CR, cr, u32::MAX, &mut TestHost::new(72_000_000).ctx());
        host.peers.insert(CTL, Box::new(ctl));
        (host, FlashArray::new(CTL, BASE, 0x7_2000))
    }

    #[test]
    fn unlock_sequence_and_lockout() {
        let mut host = TestHost::new(72_000_000);
        let mut ctl = FlashCtl::new(ARRAY, 0x800, None);
        assert_eq!(ctl.read(CR, u32::MAX, &mut host.ctx()), LOCK);
        ctl.write(CR, PG, u32::MAX, &mut host.ctx());
        assert_eq!(host.blockers[0].0, "flash-locked");
        let unlocked = unlocked();
        assert_eq!(unlocked.cr, 0);
        ctl.write(KEYR, KEY2, u32::MAX, &mut host.ctx());
        ctl.write(KEYR, KEY1, u32::MAX, &mut host.ctx());
        ctl.write(KEYR, KEY2, u32::MAX, &mut host.ctx());
        assert_eq!(ctl.read(CR, u32::MAX, &mut host.ctx()), LOCK, "a wrong first key locks until reset");
        ctl.reset(ResetKind::System, &mut host.ctx());
        ctl.write(KEYR, KEY1, u32::MAX, &mut host.ctx());
        ctl.write(KEYR, KEY2, u32::MAX, &mut host.ctx());
        assert_eq!(ctl.read(CR, u32::MAX, &mut host.ctx()), 0);
        assert_eq!(ctl.read(ACR, u32::MAX, &mut host.ctx()), 0x30, "prefetch enabled and active");
    }

    #[test]
    fn half_word_programming_rules() {
        let (mut host, mut array) = setup(PG);
        // FLASH_ProgramHalfWord(0x0800F800, 0x0055), then 0x00AA.
        array.write(0x1800, 0x0055, 0xFFFF, &mut host.ctx());
        array.write(0x1800, 0x00AA << 16, 0xFFFF_0000, &mut host.ctx());
        assert_eq!(array.read(0x1800, u32::MAX, &mut host.ctx()), 0x00AA_0055);
        assert_eq!(ctl(&mut host).sr, EOP);
        array.write(0x1800, 0x1234, 0xFFFF, &mut host.ctx());
        assert_eq!(ctl(&mut host).sr & PGERR, PGERR, "not erased");
        array.write(0x1800, 0, 0xFFFF, &mut host.ctx());
        assert_eq!(array.read(0x1800, u32::MAX, &mut host.ctx()) & 0xFFFF, 0, "clearing to 0 is allowed");
        array.write(0x1000, 0x12, 0xFF, &mut host.ctx());
        assert_eq!(array.read(0x1000, u32::MAX, &mut host.ctx()), u32::MAX, "byte stores do not program");
        assert_eq!(array.programs(), 3);
    }

    #[test]
    fn stores_without_pg_are_ignored() {
        let (mut host, mut array) = setup(0);
        array.write(0x10, 0, u32::MAX, &mut host.ctx());
        assert_eq!(array.read(0x10, u32::MAX, &mut host.ctx()), u32::MAX);
        assert_eq!(host.blockers[0].0, "flash-write-no-pg");
    }

    #[test]
    fn page_erase_through_the_interface() {
        let mut host = TestHost::new(72_000_000);
        let mut array = FlashArray::new(CTL, BASE, 0x7_2000);
        array.bytes[0x1800..0x1804].copy_from_slice(&[0x55, 0, 0xAA, 0]);
        array.bytes[0x2000] = 0;
        host.peers.insert(ARRAY, Box::new(array));
        let mut ctl = unlocked();
        ctl.write(CR, PER, u32::MAX, &mut host.ctx());
        ctl.write(AR, 0x0800_F800, u32::MAX, &mut host.ctx());
        ctl.write(CR, PER | STRT, u32::MAX, &mut host.ctx());
        assert_eq!(ctl.read(CR, u32::MAX, &mut host.ctx()), PER, "STRT clears");
        assert_eq!(ctl.read(SR, u32::MAX, &mut host.ctx()), EOP);
        let array: &mut FlashArray =
            (host.peers.get_mut(&ARRAY).unwrap().as_mut() as &mut dyn std::any::Any).downcast_mut().unwrap();
        assert!(array.bytes()[0x1800..0x2000].iter().all(|&b| b == 0xFF));
        assert_eq!(array.bytes()[0x2000], 0, "the next page is untouched");
        assert_eq!(host.log, ["FLASH page erase 0x0800F800"]);
        ctl.write(AR, 0x0800_3000, u32::MAX, &mut host.ctx());
        ctl.write(CR, PER | STRT, u32::MAX, &mut host.ctx());
        assert_eq!(host.blockers[0].0, "flash-erase-outside-array");
        ctl.write(SR, EOP, u32::MAX, &mut host.ctx());
        assert_eq!(ctl.read(SR, u32::MAX, &mut host.ctx()), 0, "EOP clears by writing 1");
    }

    #[test]
    fn lock_bit_relocks() {
        let mut host = TestHost::new(72_000_000);
        let mut ctl = unlocked();
        ctl.write(CR, LOCK, u32::MAX, &mut host.ctx());
        assert!(!ctl.programming());
        ctl.write(CR, PG, u32::MAX, &mut host.ctx());
        assert_eq!(ctl.read(CR, u32::MAX, &mut host.ctx()), LOCK);
    }
}
