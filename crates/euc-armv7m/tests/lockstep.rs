//! Whole-firmware lock-step: the stock Begode Extreme image boots on Unicorn while our core
//! shadows every quantum, from boot through 30 simulated seconds.
//!
//! Unicorn (block quanta) drives the real machine. Before each quantum the shadow copies the
//! machine's memory and registers; Unicorn runs and every bus call is recorded, with any bytes
//! the engine wrote to plain memory during the call (DMA, for example). Our core then runs exactly
//! the instructions Unicorn retired, on the copy, against a bus that replays the recording: reads
//! return what Unicorn read, writes and exception returns must match what Unicorn did. After the
//! quantum the registers and all plain memory must be identical.
//!
//! The image is never in the repo: the test reads it from the firmware cache (`euc fetch
//! begode-extreme`, or `EUC_CACHE_DIR`) and passes with a printed reason when it is not there.
//! `EUC_LOCKSTEP_SECONDS` changes the simulated time (default 30).
//!
//! ```text
//! cargo test -p euc-armv7m --features unicorn --release --test lockstep -- --nocapture
//! ```

#![allow(unsafe_code)]

use std::cell::RefCell;
use std::fmt::Write as _;
use std::ptr::NonNull;
use std::rc::Rc;
use std::time::Instant;

use euc_armv7m::Armv7m;
use euc_core::{Bus, BusFault, CoreRegs, Cpu, Exit, Region, RegionKind, Size};
use euc_unicorn::UnicornCpu;

mod common;

/// Bytes the engine changed in plain memory during one bus call: `(region, offset, bytes)`.
type Delta = Vec<(usize, usize, Vec<u8>)>;

/// One bus call of the reference run.
#[derive(Debug, Clone)]
enum Call {
    Read { addr: u32, size: Size, result: Result<u32, BusFault>, delta: Delta },
    Write { addr: u32, size: Size, value: u32, result: Result<(), BusFault>, delta: Delta },
    Return { exc_return: u32, before: CoreRegs, after: CoreRegs, delta: Delta },
}

/// A plain-memory region of the machine and its shadow copy.
struct Shadowed {
    real: NonNull<u8>,
    copy: Box<[u8]>,
    /// The real bytes before the bus call being recorded.
    before: Box<[u8]>,
}

impl Shadowed {
    fn real(&self) -> &[u8] {
        // SAFETY: `real` points at a region of `copy.len()` bytes the machine keeps alive and in
        // place for the CPU's lifetime; the engine only touches it inside bus calls, and this
        // test reads it between them or inside its own bus wrapper, on the same thread.
        unsafe { std::slice::from_raw_parts(self.real.as_ptr(), self.copy.len()) }
    }
}

/// What the lock-step saw, shared with the test.
#[derive(Default)]
struct Stats {
    quanta: u64,
    instructions: u64,
    bus_calls: u64,
    returns: u64,
    engine_writes: u64,
    divergences: u64,
    first: Option<String>,
    own_seconds: f64,
}

/// Unicorn drives the machine; our core shadows each quantum.
struct LockStep {
    reference: UnicornCpu,
    own: Armv7m,
    memory: Vec<Shadowed>,
    log: Vec<Call>,
    stats: Rc<RefCell<Stats>>,
}

/// Records the reference run's bus calls and what the engine wrote to memory during each.
struct Recorder<'a> {
    bus: &'a mut dyn Bus,
    memory: &'a mut [Shadowed],
    log: &'a mut Vec<Call>,
}

impl Recorder<'_> {
    fn snapshot(&mut self) {
        for region in self.memory.iter_mut() {
            // SAFETY: as in `Shadowed::real`; copying out creates no lasting reference.
            let real = unsafe { std::slice::from_raw_parts(region.real.as_ptr(), region.copy.len()) };
            region.before.copy_from_slice(real);
        }
    }

    fn delta(&self) -> Delta {
        let mut delta = Vec::new();
        for (i, region) in self.memory.iter().enumerate() {
            let real = region.real();
            if real == &region.before[..] {
                continue;
            }
            let mut at = 0;
            while at < real.len() {
                if real[at] == region.before[at] {
                    at += 1;
                    continue;
                }
                let start = at;
                while at < real.len() && real[at] != region.before[at] {
                    at += 1;
                }
                delta.push((i, start, real[start..at].to_vec()));
            }
        }
        delta
    }
}

impl Bus for Recorder<'_> {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        self.snapshot();
        let result = self.bus.read(addr, size);
        let delta = self.delta();
        self.log.push(Call::Read { addr, size, result, delta });
        result
    }

    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        self.snapshot();
        let result = self.bus.write(addr, size, value);
        let delta = self.delta();
        self.log.push(Call::Write { addr, size, value: value & size.mask(), result, delta });
        result
    }

    fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
        self.bus.fetch(addr)
    }

    fn take_yield(&mut self) -> bool {
        self.bus.take_yield()
    }

    fn exception_return(&mut self, regs: &mut CoreRegs, exc_return: u32) {
        self.snapshot();
        let before = regs.clone();
        self.bus.exception_return(regs, exc_return);
        let delta = self.delta();
        self.log.push(Call::Return { exc_return, before, after: regs.clone(), delta });
    }

    fn note_pc(&mut self, pc: u32) {
        self.bus.note_pc(pc);
    }

    fn reached(&mut self, addr: u32) {
        self.bus.reached(addr);
    }
}

/// Feeds our core the reference run's bus traffic and checks it asks for the same.
struct Replayer<'a> {
    log: &'a [Call],
    next: usize,
    memory: &'a mut [Shadowed],
    mismatch: Option<String>,
}

impl Replayer<'_> {
    fn apply(&mut self, delta: &Delta) {
        for (region, at, bytes) in delta {
            self.memory[*region].copy[*at..at + bytes.len()].copy_from_slice(bytes);
        }
    }

    fn take(&mut self, what: &str) -> Option<Call> {
        let call = self.log.get(self.next).cloned();
        self.next += 1;
        if call.is_none() && self.mismatch.is_none() {
            self.mismatch = Some(format!("our core made an extra bus call: {what}"));
        }
        call
    }

    fn differ(&mut self, text: String) {
        if self.mismatch.is_none() {
            self.mismatch = Some(text);
        }
    }
}

impl Bus for Replayer<'_> {
    fn read(&mut self, addr: u32, size: Size) -> Result<u32, BusFault> {
        match self.take(&format!("read {size:?} 0x{addr:08X}")) {
            Some(Call::Read { addr: a, size: s, result, delta }) if a == addr && s == size => {
                self.apply(&delta);
                result
            }
            Some(other) => {
                self.differ(format!("our core read {size:?} 0x{addr:08X}, Unicorn did {other:?}"));
                Ok(0)
            }
            None => Ok(0),
        }
    }

    fn write(&mut self, addr: u32, size: Size, value: u32) -> Result<(), BusFault> {
        let value = value & size.mask();
        match self.take(&format!("write {size:?} 0x{addr:08X} = 0x{value:X}")) {
            Some(Call::Write { addr: a, size: s, value: v, result, delta }) if (a, s, v) == (addr, size, value) => {
                self.apply(&delta);
                result
            }
            Some(other) => {
                self.differ(format!("our core wrote {size:?} 0x{addr:08X} = 0x{value:X}, Unicorn did {other:?}"));
                Ok(())
            }
            None => Ok(()),
        }
    }

    fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
        self.differ(format!("our core fetched from MMIO at 0x{addr:08X}"));
        Ok(0xBF00)
    }

    fn exception_return(&mut self, regs: &mut CoreRegs, exc_return: u32) {
        match self.take(&format!("exception return 0x{exc_return:08X}")) {
            Some(Call::Return { exc_return: v, before, after, delta }) if v | 1 == exc_return | 1 => {
                if let Some(field) = differs(&before, regs, false) {
                    self.differ(format!("at exception return {field}"));
                }
                *regs = after;
                self.apply(&delta);
            }
            Some(other) => self.differ(format!("our core returned 0x{exc_return:08X}, Unicorn did {other:?}")),
            None => {}
        }
    }
}

/// The xPSR bits a core owns: flags, Thumb bit, IT state.
const OWNED: u32 = 0xF800_0000 | CoreRegs::XPSR_T | 0x0600_FC00;

/// The first register that differs, if any. PC only when `pc` is set.
fn differs(want: &CoreRegs, got: &CoreRegs, pc: bool) -> Option<String> {
    for n in 0..15 {
        if want.reg(n) != got.reg(n) {
            return Some(format!("r{n}: Unicorn 0x{:08X}, our core 0x{:08X}", want.reg(n), got.reg(n)));
        }
    }
    if pc && want.pc != got.pc {
        return Some(format!("pc: Unicorn 0x{:08X}, our core 0x{:08X}", want.pc, got.pc));
    }
    if want.xpsr & OWNED != got.xpsr & OWNED {
        return Some(format!("xpsr: Unicorn 0x{:08X}, our core 0x{:08X}", want.xpsr, got.xpsr));
    }
    let masks = |r: &CoreRegs| (r.primask, r.faultmask, r.basepri, r.control);
    if masks(want) != masks(got) {
        return Some(format!("masks: Unicorn {:?}, our core {:?}", masks(want), masks(got)));
    }
    None
}

impl LockStep {
    /// # Safety
    ///
    /// As for `UnicornCpu::new`.
    unsafe fn new(regions: &[Region], stats: Rc<RefCell<Stats>>) -> Result<Self, String> {
        // SAFETY: forwarded from this function's contract.
        let reference = unsafe { UnicornCpu::new(regions) }.map_err(|e| e.to_string())?;
        let mut memory = Vec::new();
        let mut shadow_regions = Vec::new();
        for region in regions {
            let kind = match region.kind {
                RegionKind::Memory { ptr, writable } => {
                    let mut copy = vec![0; region.size as usize].into_boxed_slice();
                    let ptr_copy = NonNull::from(&mut copy[0]);
                    let before = copy.clone();
                    memory.push(Shadowed { real: ptr, copy, before });
                    RegionKind::Memory { ptr: ptr_copy, writable }
                }
                RegionKind::Mmio => RegionKind::Mmio,
            };
            shadow_regions.push(Region { kind, ..*region });
        }
        // SAFETY: the copies are boxed (they never move) and live in `self.memory` next to the
        // core; they are only touched between runs or by the replaying bus during a run.
        let own = unsafe { Armv7m::new(&shadow_regions) }.map_err(|e| e.to_string())?;
        Ok(Self { reference, own, memory, log: Vec::new(), stats })
    }
}

impl Cpu for LockStep {
    fn reset(&mut self, sp: u32, reset_vector: u32) {
        self.reference.reset(sp, reset_vector);
        self.own.reset(sp, reset_vector);
    }

    fn run(&mut self, bus: &mut dyn Bus, max_instr: u32) -> Exit {
        // The shadow starts from the machine's state: memory, and registers the engine may
        // have changed (exception entry, reset).
        for region in &mut self.memory {
            let real = region.real().to_vec();
            region.copy.copy_from_slice(&real);
        }
        *self.own.regs_mut() = self.reference.regs().clone();
        let start_pc = self.reference.regs().pc;

        self.log.clear();
        let before = self.reference.retired();
        let exit = {
            let mut recorder = Recorder { bus, memory: &mut self.memory, log: &mut self.log };
            self.reference.run(&mut recorder, max_instr)
        };
        let executed = self.reference.retired() - before;

        let started = Instant::now();
        let mut replayer = Replayer { log: &self.log, next: 0, memory: &mut self.memory, mismatch: None };
        let mut own_exit = Exit::Budget;
        let mut left = executed;
        while left > 0 {
            let chunk = left.min(u64::from(u32::MAX)) as u32;
            own_exit = self.own.run(&mut replayer, chunk);
            left -= u64::from(chunk);
            if own_exit != Exit::Budget {
                break;
            }
        }
        let own_seconds = started.elapsed().as_secs_f64();
        let mut mismatch = replayer.mismatch.take();
        let consumed = replayer.next;

        let mut stats = self.stats.borrow_mut();
        stats.quanta += 1;
        stats.instructions += executed;
        stats.bus_calls += self.log.len() as u64;
        stats.returns += self.log.iter().filter(|c| matches!(c, Call::Return { .. })).count() as u64;
        stats.engine_writes += self
            .log
            .iter()
            .filter(|c| match c {
                Call::Read { delta, .. } | Call::Write { delta, .. } | Call::Return { delta, .. } => !delta.is_empty(),
            })
            .count() as u64;
        stats.own_seconds += own_seconds;
        if mismatch.is_none() && consumed < self.log.len() {
            mismatch = Some(format!("our core made {consumed} of Unicorn's {} bus calls", self.log.len()));
        }
        let faulted = |e: Exit| matches!(e, Exit::Fault(_) | Exit::Breakpoint(_) | Exit::Svc(_));
        if mismatch.is_none() && faulted(exit) != faulted(own_exit) {
            mismatch = Some(format!("exit: Unicorn {exit:?}, our core {own_exit:?}"));
        }
        if mismatch.is_none() && !faulted(exit) {
            mismatch = differs(self.reference.regs(), self.own.regs(), true);
        }
        if mismatch.is_none() {
            for (i, region) in self.memory.iter().enumerate() {
                if let Some(at) = region.real().iter().zip(region.copy.iter()).position(|(a, b)| a != b) {
                    mismatch = Some(format!("memory region {i} differs at offset 0x{at:X}"));
                    break;
                }
            }
        }
        if let Some(text) = mismatch {
            stats.divergences += 1;
            if stats.first.is_none() {
                let mut first =
                    format!("quantum {} ({} instructions from PC 0x{:08X}): {text}", stats.quanta, executed, start_pc);
                let _ = write!(first, "\n  Unicorn {:08X?}\n  our     {:08X?}", self.reference.regs(), self.own.regs());
                stats.first = Some(first);
            }
        }
        exit
    }

    fn regs(&self) -> &CoreRegs {
        self.reference.regs()
    }

    fn regs_mut(&mut self) -> &mut CoreRegs {
        self.reference.regs_mut()
    }

    fn retired(&self) -> u64 {
        self.reference.retired()
    }

    fn watch(&mut self, addrs: &[u32]) {
        self.reference.watch(addrs);
    }

    fn invalidate(&mut self, addr: u32, len: u32) {
        self.reference.invalidate(addr, len);
        self.own.invalidate(addr, len);
    }
}

#[test]
fn stock_image_runs_in_lockstep_with_unicorn() {
    let Some((wheel, image)) = common::stock() else { return };
    let seconds = common::seconds("EUC_LOCKSTEP_SECONDS", 30.0);
    let stats = Rc::new(RefCell::new(Stats::default()));
    let shared = Rc::clone(&stats);
    let cpu = Box::new(move |regions: &[Region]| {
        // SAFETY: `Machine::new` owns the memory behind every region in boxed buffers that never
        // move and outlive the CPU, and touches it during a run only from bus callbacks.
        let cpu = unsafe { LockStep::new(regions, shared) }?;
        Ok(Box::new(cpu) as Box<dyn Cpu>)
    });
    let common::Run { report, wall, .. } = common::boot(&wheel, &image, seconds, cpu);
    let stats = stats.borrow();
    eprintln!(
        "lockstep: {seconds} s simulated in {wall:.1} s: {} quanta, {} instructions, {} bus calls, {} exception \
         returns, {} calls during which the engine wrote memory; our core alone {:.2} s",
        stats.quanta, stats.instructions, stats.bus_calls, stats.returns, stats.engine_writes, stats.own_seconds
    );
    assert_eq!(report.halted, None, "{:#?}", report.blockers);
    assert_eq!(
        stats.divergences,
        0,
        "{} quanta diverged; first: {}",
        stats.divergences,
        stats.first.as_deref().unwrap_or("")
    );
    assert!(report.ble.frames > 0, "the firmware sends telemetry");
}
