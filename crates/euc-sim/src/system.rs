//! The machine minus the CPU: memory, components, interrupts, wiring and diagnostics.
//!
//! [`System`] is the [`Bus`] the CPU backend runs against and the [`Host`] every component talks
//! to. Its access path, poll detector and stuck-loop logic are ports of the Python reference
//! harness (`Machine.access` and `_poll_hit`), kept exact so both make the same decisions.

use std::collections::{HashMap, VecDeque};

use euc_core::{
    Access, Bus, BusFault, BusFaultKind, CoreRegs, Ctx, DeviceId, ExceptionUnit, Host, Lane, Peripheral, PlantView,
    PollDetector, ResetKind, SYSTICK, Signal, Size,
};

use crate::config::{MemoryMap, Span};
use crate::memory::{Memory, MemoryBus};
use crate::plant::Plant;

/// Granularity of the MMIO page table.
const PAGE_SHIFT: u32 = 10;
/// The System Control Space inside the PPB.
const SCS: Span = Span { base: 0xE000_E000, size: 0x1000 };
/// Name the SCS has in statistics.
pub(crate) const SCS_NAME: &str = "SCS";
/// Key of the SCS in [`Diagnostics::accesses`].
pub(crate) const SCS_KEY: DeviceId = DeviceId::MAX;

/// A component and its pending events.
pub(crate) struct Slot {
    pub(crate) name: String,
    pub(crate) base: Option<u32>,
    pub(crate) size: u32,
    pub(crate) device: Option<Box<dyn Peripheral>>,
    /// `(tag, cycle)` of each pending event.
    pub(crate) timers: Vec<(u32, u64)>,
}

/// What an MMIO page belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Target {
    Device(DeviceId),
    Scs,
}

/// Who started a bus access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Master {
    Cpu,
    /// The CPU's read half of a bit-band read-modify-write: counted, but no poll tracking.
    BitBand,
    Dma,
}

enum Pending {
    Signal { from: DeviceId, port: u8, signal: Signal },
    Dma { from: DeviceId, line: u8 },
}

/// A problem the run hit, deduplicated by kind and text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Blocker {
    /// Short kind (`infinite-poll`, `unmapped`).
    pub kind: String,
    /// What happened, with PC and registers.
    pub text: String,
    /// Cycle of the first occurrence.
    pub cycle: u64,
    /// Occurrences.
    pub count: u64,
    /// The run stopped on it.
    pub fatal: bool,
}

/// What the run logged and counted.
#[derive(Debug, Default)]
pub struct Diagnostics {
    /// Timeline events `(cycle, text)`.
    pub events: Vec<(u64, String)>,
    /// Problems.
    pub blockers: Vec<Blocker>,
    /// Why the run stopped for good, if it did.
    pub halted: Option<String>,
    /// Cycles skipped by poll fast-forward, by `(pc, address)`, with the register name.
    pub poll_skips: HashMap<(u32, u32), (u64, String)>,
    /// MMIO accesses (CPU, DMA and bit-band).
    pub mmio_count: u64,
    /// Accesses by `(component, aligned offset, is_write)`; the SCS uses `DeviceId::MAX`.
    pub accesses: HashMap<(DeviceId, u32, bool), u64>,
    /// First cycle each watched address was reached.
    pub reached: Vec<(u32, u64)>,
    /// Lowest exception frame address seen.
    pub min_frame: Option<u32>,
}

/// Everything except the CPU.
pub struct System {
    pub(crate) hclk: u64,
    /// Cycle the current quantum started at.
    pub(crate) chunk_start: u64,
    /// Cycles a poll fast-forward skipped inside the current quantum.
    pub(crate) skip: u64,
    /// A poll fast-forward ended the current quantum early.
    pub(crate) early: bool,
    /// End of the current `run` call; caps fast-forwards.
    pub(crate) run_end: Option<u64>,
    pub(crate) mem: Memory,
    map: MemoryMap,
    pub(crate) slots: Vec<Slot>,
    pages: PageTable,
    pub(crate) exc: ExceptionUnit,
    irq_sources: Vec<u128>,
    wires: HashMap<(DeviceId, u8), Vec<(DeviceId, u8)>>,
    dma_routes: HashMap<(DeviceId, u8), (DeviceId, u8)>,
    queue: VecDeque<Pending>,
    pub(crate) plant: Plant,
    poll: PollDetector,
    stuck: Option<Vec<(u32, u32)>>,
    stuck_skip: u64,
    pc: u32,
    yield_pending: bool,
    pub(crate) reset_pending: Option<(ResetKind, String)>,
    pub(crate) irq_names: Vec<String>,
    marker_names: HashMap<u32, String>,
    pub(crate) diag: Diagnostics,
}

impl System {
    /// A system with `slots` mapped; `irq_names` index external interrupts.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        hclk: u64,
        mem: Memory,
        map: MemoryMap,
        slots: Vec<Slot>,
        exc: ExceptionUnit,
        plant: Plant,
        irq_names: Vec<String>,
        marker_names: HashMap<u32, String>,
    ) -> Self {
        let mut pages = PageTable::new(&[map.periph, map.ppb, map.flash]);
        pages.set(SCS, Target::Scs);
        let irqs = usize::from(exc.irqs());
        let mut sys = Self {
            hclk,
            chunk_start: 0,
            skip: 0,
            early: false,
            run_end: None,
            mem,
            map,
            slots: Vec::new(),
            pages,
            exc,
            irq_sources: vec![0; irqs],
            wires: HashMap::new(),
            dma_routes: HashMap::new(),
            queue: VecDeque::new(),
            plant,
            poll: PollDetector::default(),
            stuck: None,
            stuck_skip: 0,
            pc: 0,
            yield_pending: false,
            reset_pending: None,
            irq_names,
            marker_names,
            diag: Diagnostics::default(),
        };
        for slot in slots {
            sys.attach(slot);
        }
        sys
    }

    fn attach(&mut self, slot: Slot) -> DeviceId {
        let id = self.slots.len() as DeviceId;
        if let Some(base) = slot.base {
            self.pages.set(Span { base, size: slot.size }, Target::Device(id));
        }
        self.slots.push(slot);
        id
    }

    /// Connects output `from` to input `to`.
    pub(crate) fn wire(&mut self, from: (DeviceId, u8), to: (DeviceId, u8)) {
        self.wires.entry(from).or_default().push(to);
    }

    /// Routes DMA request `request` to controller channel `channel`.
    pub(crate) fn route_dma(&mut self, request: (DeviceId, u8), channel: (DeviceId, u8)) {
        self.dma_routes.insert(request, channel);
    }

    /// The current cycle as components see it.
    pub(crate) const fn now(&self) -> u64 {
        self.chunk_start + self.skip
    }

    /// The earliest pending event: SysTick or any component's.
    pub(crate) fn next_event(&self) -> Option<u64> {
        let timers = self.slots.iter().flat_map(|slot| slot.timers.iter().map(|&(_, at)| at));
        timers.chain(self.exc.systick.next_event()).min()
    }

    /// Fires everything due at `now`: SysTick first, then each component in board order, each
    /// once, as the reference harness does.
    pub(crate) fn fire_due(&mut self, now: u64) {
        if self.exc.systick.next_event().is_some_and(|at| at <= now) && self.exc.systick.on_event(now) {
            self.exc.pend(SYSTICK);
        }
        for index in 0..self.slots.len() {
            let mut due: Vec<u32> = Vec::new();
            self.slots[index].timers.retain(|&(tag, at)| {
                let fire = at <= now;
                if fire {
                    due.push(tag);
                }
                !fire
            });
            for tag in due {
                self.call(index as DeviceId, |device, ctx| device.on_event(tag, ctx));
                self.drain();
            }
        }
    }

    /// Calls component `id` with a context, if it is not already busy (a component never
    /// reaches itself through the bus). Wires and DMA requests it raises are queued.
    pub(crate) fn call<R>(
        &mut self,
        id: DeviceId,
        f: impl FnOnce(&mut dyn Peripheral, &mut Ctx<'_>) -> R,
    ) -> Option<R> {
        let mut device = self.slots.get_mut(usize::from(id))?.device.take()?;
        let now = self.now();
        let result = f(device.as_mut(), &mut Ctx::new(now, id, self));
        self.slots[usize::from(id)].device = Some(device);
        Some(result)
    }

    /// Delivers queued signals and DMA requests until none are left.
    pub(crate) fn drain(&mut self) {
        while let Some(pending) = self.queue.pop_front() {
            match pending {
                Pending::Signal { from, port, signal } => {
                    let count = self.wires.get(&(from, port)).map_or(0, Vec::len);
                    for i in 0..count {
                        let (to, input) = self.wires[&(from, port)][i];
                        self.call(to, |device, ctx| device.input(input, signal, ctx));
                    }
                }
                Pending::Dma { from, line } => {
                    if let Some(&(dma, channel)) = self.dma_routes.get(&(from, line)) {
                        self.call(dma, |device, ctx| device.dma_request(channel, ctx));
                    }
                }
            }
        }
    }

    /// Resets every component and the exception unit (the CPU is the machine's job).
    pub(crate) fn reset_components(&mut self, kind: ResetKind) {
        for slot in &mut self.slots {
            slot.timers.clear();
        }
        for irq in 0..self.irq_sources.len() {
            self.irq_sources[irq] = 0;
            self.exc.set_level(irq as u16, false);
        }
        self.exc.reset();
        if kind == ResetKind::Power {
            self.mem.clear_sram();
        }
        for id in 0..self.slots.len() {
            self.call(id as DeviceId, |device, ctx| device.reset(kind, ctx));
        }
        self.drain();
    }

    /// Records a blocker; a fatal one halts the run after the current instruction.
    pub(crate) fn blocker(&mut self, kind: &str, text: String, fatal: bool) {
        if let Some(known) = self.diag.blockers.iter_mut().find(|b| b.kind == kind && b.text == text) {
            known.count += 1;
            return;
        }
        let now = self.now();
        self.diag.events.push((now, format!("BLOCKER {kind}: {text}")));
        if fatal {
            self.diag.halted = Some(format!("{kind}: {text}"));
            self.yield_pending = true;
        }
        self.diag.blockers.push(Blocker { kind: kind.to_owned(), text, cycle: now, count: 1, fatal });
    }

    /// `"USART1.SR"` for an address, for messages.
    fn describe(&self, addr: u32) -> String {
        match self.pages.get(addr) {
            Some(Target::Scs) => {
                let reg = ExceptionUnit::reg_name((addr - SCS.base) & !3);
                format!("{SCS_NAME}.{}", reg.map_or_else(|| format!("+{:03X}", (addr - SCS.base) & !3), str::to_owned))
            }
            Some(Target::Device(id)) => {
                let slot = &self.slots[usize::from(id)];
                let off = (addr - slot.base.unwrap_or(0)) & !3;
                let reg = slot.device.as_ref().and_then(|d| d.reg_name(off));
                format!("{}.{}", slot.name, reg.map_or_else(|| format!("+{off:03X}"), str::to_owned))
            }
            None => format!("0x{addr:08X}"),
        }
    }

    /// The MMIO target of `addr`, creating a read-back placeholder for an unmapped page of the
    /// peripheral or private peripheral region (unmodeled registers are traced, not fatal).
    fn target(&mut self, addr: u32) -> Option<Target> {
        if let Some(target) = self.pages.get(addr) {
            return Some(target);
        }
        if !self.map.periph.contains(addr) && !self.map.ppb.contains(addr) {
            return None;
        }
        let base = addr & !((1 << PAGE_SHIFT) - 1);
        let id = self.attach(Slot {
            name: format!("UNMODELED_{base:08X}"),
            base: Some(base),
            size: 1 << PAGE_SHIFT,
            device: Some(Box::new(Unmodeled::default())),
            timers: Vec::new(),
        });
        Some(Target::Device(id))
    }

    /// One access to MMIO space (peripherals, PPB, data flash, bit-band aliases).
    fn mmio(&mut self, addr: u32, size: Size, write: Option<u32>, master: Master) -> Result<u32, BusFault> {
        let access = if write.is_some() { Access::Write } else { Access::Read };
        let fault = BusFault { addr, access, kind: BusFaultKind::Unmapped };
        if let Some((word, bit)) = bitband(addr, self.map.sram_bitband, self.map.sram) {
            return self.sram_bitband(word, bit, write).ok_or(fault);
        }
        if let Some((word, bit)) = bitband(addr, self.map.periph_bitband, self.map.periph) {
            return self.periph_bitband(word, bit, write, master);
        }
        let target = self.target(addr).ok_or(fault)?;
        let data_flash = addr >= self.map.code_end && self.map.flash.contains(addr);
        let base = match target {
            Target::Scs => SCS.base,
            Target::Device(id) => self.slots[usize::from(id)].base.unwrap_or(0),
        };
        let off = addr - base;
        let lane = Lane::new(off, size);
        let now = self.now();
        let value = match (target, write) {
            (Target::Scs, Some(value)) => {
                let effect = self.exc.write(lane.word, lane.place(value), lane.mask, now);
                self.scs_effects(effect.enabled, effect.reset_request);
                0
            }
            (Target::Scs, None) => lane.extract(self.exc.read(lane.word, now)),
            (Target::Device(id), Some(value)) => {
                self.call(id, |device, ctx| device.write(lane.word, lane.place(value), lane.mask, ctx)).ok_or(fault)?;
                0
            }
            (Target::Device(id), None) => {
                lane.extract(self.call(id, |device, ctx| device.read(lane.word, lane.mask, ctx)).ok_or(fault)?)
            }
        };
        let key = match target {
            Target::Scs => SCS_KEY,
            Target::Device(id) => id,
        };
        *self.diag.accesses.entry((key, lane.word, write.is_some())).or_default() += 1;
        if data_flash {
            return Ok(value);
        }
        self.diag.mmio_count += 1;
        if master == Master::Cpu {
            self.track_polls(addr, value, write.is_some());
        }
        Ok(value)
    }

    fn scs_effects(&mut self, enabled: u128, reset_request: bool) {
        if enabled != 0 {
            let names: Vec<String> = (0..128)
                .filter(|irq| enabled >> irq & 1 == 1)
                .map(|irq| self.irq_names.get(irq).cloned().unwrap_or_else(|| irq.to_string()))
                .collect();
            let now = self.now();
            self.diag.events.push((now, format!("NVIC enable {}", names.join(", "))));
        }
        if reset_request {
            self.request_reset(ResetKind::System, format!("SYSRESETREQ (AIRCR) from PC 0x{:08X}", self.pc));
        }
    }

    fn request_reset(&mut self, kind: ResetKind, why: String) {
        self.reset_pending = Some((kind, why));
        self.yield_pending = true;
    }

    fn sram_bitband(&mut self, word: u32, bit: u32, write: Option<u32>) -> Option<u32> {
        let current = self.mem.read(word, Size::Word)?;
        match write {
            None => Some((current >> bit) & 1),
            Some(value) => {
                let new = if value & 1 == 1 { current | 1 << bit } else { current & !(1 << bit) };
                self.mem.write(word, Size::Word, new).then_some(0)
            }
        }
    }

    fn periph_bitband(&mut self, word: u32, bit: u32, write: Option<u32>, master: Master) -> Result<u32, BusFault> {
        match write {
            None => Ok((self.mmio(word, Size::Word, None, master)? >> bit) & 1),
            Some(value) => {
                let current = self.mmio(word, Size::Word, None, Master::BitBand)?;
                let new = if value & 1 == 1 { current | 1 << bit } else { current & !(1 << bit) };
                self.mmio(word, Size::Word, Some(new), master)
            }
        }
    }

    /// The reference harness's poll and stuck-loop bookkeeping for one CPU access.
    fn track_polls(&mut self, addr: u32, value: u32, is_write: bool) {
        let thread_mode = self.exc.active().is_empty();
        if thread_mode && self.stuck.as_ref().is_some_and(|keys| is_write || !keys.contains(&(addr, value))) {
            self.stuck = None;
            self.stuck_skip = 0;
        }
        if is_write {
            self.poll.reset();
        } else if thread_mode && self.poll.observe(addr, value) {
            self.poll_hit(addr, value);
        }
    }

    /// A poll loop: skip time to the next event that could change the value, or report a wait
    /// that nothing can end.
    fn poll_hit(&mut self, addr: u32, value: u32) {
        if self.early {
            return;
        }
        let now = self.now();
        let Some(next) = self.next_event() else {
            let mut keys = self.poll.keys().to_vec();
            keys.sort_unstable();
            let regs: Vec<String> = keys.iter().map(|&(a, v)| format!("{}=0x{v:X}", self.describe(a))).collect();
            let text =
                format!("PC 0x{:08X} spins reading {}; no scheduled event can change it", self.pc, regs.join(", "));
            self.blocker("infinite-poll", text, true);
            return;
        };
        let next = self.run_end.map_or(next, |end| next.min(end));
        if next > now {
            let skipped = next - now;
            self.skip += skipped;
            let reg = self.describe(addr);
            let entry = self.diag.poll_skips.entry((self.pc, addr)).or_insert((0, reg));
            entry.0 += skipped;
            if self.stuck.as_ref().is_some_and(|keys| keys.contains(&(addr, value))) {
                self.stuck_skip += skipped;
                if self.stuck_skip > 2 * self.hclk {
                    let mut keys = self.stuck.clone().unwrap_or_default();
                    keys.sort_unstable();
                    let regs: Vec<String> =
                        keys.iter().map(|&(a, v)| format!("{}=0x{v:X}", self.describe(a))).collect();
                    let text = format!(
                        "PC 0x{:08X} loop waits on {} for {:.1} s of sim time with no other thread activity",
                        self.pc,
                        regs.join(", "),
                        self.stuck_skip as f64 / self.hclk as f64
                    );
                    self.blocker("stuck-poll", text, true);
                }
            } else {
                self.stuck = Some(self.poll.keys().to_vec());
                self.stuck_skip = skipped;
            }
        }
        self.early = true;
        self.poll.reset();
        self.yield_pending = true;
    }

    /// Starts a quantum at `start`.
    pub(crate) fn begin_quantum(&mut self, start: u64) {
        self.chunk_start = start;
        self.skip = 0;
        self.early = false;
        self.poll.reset();
    }

    /// The stock-map name of a watched address.
    pub(crate) fn marker_name(&self, addr: u32) -> String {
        self.marker_names.get(&addr).cloned().unwrap_or_else(|| format!("0x{addr:08X}"))
    }

    /// Every watched address with its name.
    pub(crate) fn marker_names(&self) -> impl Iterator<Item = (u32, &String)> {
        self.marker_names.iter().map(|(&addr, name)| (addr, name))
    }

    /// `("USART1", "SR")` for an access-statistics key.
    pub(crate) fn register_label(&self, key: DeviceId, off: u32) -> (String, String) {
        if key == SCS_KEY {
            let reg = ExceptionUnit::reg_name(off).map_or_else(|| format!("+{off:03X}"), str::to_owned);
            return (SCS_NAME.to_owned(), reg);
        }
        let slot = &self.slots[usize::from(key)];
        let reg = slot.device.as_ref().and_then(|d| d.reg_name(off));
        (slot.name.clone(), reg.map_or_else(|| format!("+{off:03X}"), str::to_owned))
    }
}

/// MMIO page owners for the windows that hold registers (peripherals, PPB, flash), one slot
/// per 1 KiB page: the bus looks up every access here.
struct PageTable {
    windows: Vec<(u32, Vec<Option<Target>>)>,
}

impl PageTable {
    fn new(windows: &[Span]) -> Self {
        let windows = windows.iter().map(|w| (w.base, vec![None; (w.size >> PAGE_SHIFT) as usize])).collect();
        Self { windows }
    }

    fn slot(&mut self, addr: u32) -> Option<&mut Option<Target>> {
        self.windows
            .iter_mut()
            .find_map(|(base, pages)| pages.get_mut((addr.checked_sub(*base)? >> PAGE_SHIFT) as usize))
    }

    fn get(&self, addr: u32) -> Option<Target> {
        self.windows.iter().find_map(|(base, pages)| *pages.get((addr.checked_sub(*base)? >> PAGE_SHIFT) as usize)?)
    }

    /// Gives `span` to `target`. Pages outside every window are ignored.
    fn set(&mut self, span: Span, target: Target) {
        for page in (span.base..span.base.saturating_add(span.size)).step_by(1 << PAGE_SHIFT) {
            if let Some(slot) = self.slot(page) {
                *slot = Some(target);
            }
        }
    }
}

/// `(word, bit)` of a bit-band alias address, if `addr` is in the alias of `target`.
fn bitband(addr: u32, alias: u32, target: Span) -> Option<(u32, u32)> {
    let off = addr.checked_sub(alias)?;
    if off >= target.size.checked_mul(32)? {
        return None;
    }
    Some((target.base + ((off >> 5) & !3), ((off >> 5) & 3) * 8 + ((off >> 2) & 7)))
}

impl Bus for System {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        let result = match self.mem.read(addr, size) {
            Some(value) => Ok(value),
            None => self.mmio(addr, size, None, Master::Cpu),
        };
        self.drain();
        result
    }

    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        let result = if self.mem.sram_span().contains(addr) {
            self.mem.write(addr, size, value);
            Ok(())
        } else if self.mem.code_span().contains(addr) {
            Err(BusFault { addr, access: Access::Write, kind: BusFaultKind::ReadOnly })
        } else {
            self.mmio(addr, size, Some(value), Master::Cpu).map(|_| ())
        };
        self.drain();
        result
    }

    fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
        MemoryBus(&mut self.mem).fetch(addr)
    }

    fn take_yield(&mut self) -> bool {
        std::mem::take(&mut self.yield_pending)
    }

    fn exception_return(&mut self, regs: &mut CoreRegs, exc_return: u32) {
        if self.exc.active().is_empty() {
            let text = format!("QEMU exception 8 at PC 0x{:08X} (HardFault/UsageFault on HW)", exc_return & !1);
            self.blocker("cpu-exception", text, true);
            return;
        }
        if let Err(fault) = self.exc.exit(exc_return, regs, &mut MemoryBus(&mut self.mem)) {
            self.blocker("cpu-exception", format!("exception return: {fault}"), true);
        }
    }

    fn note_pc(&mut self, pc: u32) {
        self.pc = pc;
    }

    fn reached(&mut self, addr: u32) {
        let now = self.now();
        if !self.diag.reached.iter().any(|&(a, _)| a == addr) {
            self.diag.reached.push((addr, now));
            let name = self.marker_name(addr);
            self.diag.events.push((now, format!("reached {name} (0x{addr:08X})")));
        }
    }
}

impl Host for System {
    fn hclk(&self) -> u64 {
        self.hclk
    }

    fn schedule(&mut self, dev: DeviceId, at: u64, tag: u32) {
        let timers = &mut self.slots[usize::from(dev)].timers;
        match timers.iter_mut().find(|(t, _)| *t == tag) {
            Some(timer) => timer.1 = at,
            None => timers.push((tag, at)),
        }
    }

    fn cancel(&mut self, dev: DeviceId, tag: u32) {
        self.slots[usize::from(dev)].timers.retain(|&(t, _)| t != tag);
    }

    fn set_irq(&mut self, dev: DeviceId, irq: u16, level: bool) {
        let Some(sources) = self.irq_sources.get_mut(usize::from(irq)) else { return };
        let bit = 1_u128 << (dev % 128);
        if level {
            *sources |= bit;
        } else {
            *sources &= !bit;
        }
        let high = *sources != 0;
        self.exc.set_level(irq, high);
    }

    fn pulse_irq(&mut self, irq: u16) {
        self.exc.pend_irq(irq);
    }

    fn dma_request(&mut self, dev: DeviceId, line: u8) {
        self.queue.push_back(Pending::Dma { from: dev, line });
    }

    fn emit(&mut self, dev: DeviceId, port: u8, signal: Signal) {
        self.queue.push_back(Pending::Signal { from: dev, port, signal });
    }

    fn bus_read(&mut self, _dev: DeviceId, addr: u32, size: Size) -> Result<u32, BusFault> {
        match self.mem.read(addr, size) {
            Some(value) => Ok(value),
            None => self.mmio(addr, size, None, Master::Dma),
        }
    }

    fn bus_write(&mut self, _dev: DeviceId, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        if self.mem.write(addr, size, value) {
            return Ok(());
        }
        if self.mem.contains(addr) {
            return Err(BusFault { addr, access: Access::Write, kind: BusFaultKind::ReadOnly });
        }
        self.mmio(addr, size, Some(value), Master::Dma).map(|_| ())
    }

    fn peer(&mut self, id: DeviceId) -> Option<&mut dyn Peripheral> {
        self.slots.get_mut(usize::from(id))?.device.as_deref_mut()
    }

    fn plant(&self) -> &dyn PlantView {
        &self.plant
    }

    fn log(&mut self, _dev: DeviceId, text: String) {
        let now = self.now();
        self.diag.events.push((now, text));
    }

    fn blocker(&mut self, _dev: DeviceId, kind: &'static str, text: String, fatal: bool) {
        System::blocker(self, kind, text, fatal);
    }

    fn request_reset(&mut self, _dev: DeviceId, kind: ResetKind, why: String) {
        System::request_reset(self, kind, why);
    }
}

/// A register window nobody models: reads return what was written.
#[derive(Default)]
struct Unmodeled {
    regs: HashMap<u32, u32>,
}

impl Peripheral for Unmodeled {
    fn reset(&mut self, _kind: ResetKind, _ctx: &mut Ctx<'_>) {
        self.regs.clear();
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        self.regs.get(&off).copied().unwrap_or(0)
    }

    fn write(&mut self, off: u32, value: u32, mask: u32, _ctx: &mut Ctx<'_>) {
        let reg = self.regs.entry(off).or_default();
        *reg = (*reg & !mask) | (value & mask);
    }
}
