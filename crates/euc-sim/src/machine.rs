//! The machine: a CPU backend plus the [`System`], and the run loop.
//!
//! The loop is a port of the Python reference harness's `Machine.run`: quanta of at most
//! `max_quantum` instructions that never cross the next event, interrupts taken at quantum
//! boundaries (and tail-chained at exception return), `cpi` cycles per instruction, and a
//! quantum cut short by a poll fast-forward, a reset or a halt costing one cycle plus the skip.
//! A quantum's cycles come from the instructions the backend retired, which an exact backend
//! makes equal to the budget and a block-granular one makes at most one block more.

use std::collections::HashMap;

use euc_core::{
    BusFaultKind, Cpu, DeviceId, ExceptionUnit, Exit, Fault, IRQ0, Region, RegionKind, Registry, ResetKind,
};
use thiserror::Error;

use crate::board::{Action, BoardLogic};
use crate::config::{BoardFile, Span, StockMap};
use crate::memory::{Memory, MemoryBus};
use crate::plant::{Live, Plant, PlantState};
use crate::system::{Slot, System};

/// Name of the simulator's board-logic component in wiring.
pub const BOARD: &str = "board";
/// Name of the plant component in wiring (inverter legs in, hall lines out).
pub const PLANT: &str = "plant";

/// xPSR bits holding the IT state; an interrupt is never taken inside an IT block.
const XPSR_IT: u32 = 0x0600_FC00;

/// Everything needed to build a machine.
pub struct MachineConfig<'a> {
    /// The board file.
    pub board: &'a BoardFile,
    /// Named addresses of the firmware, for markers (stock builds only).
    pub stockmap: Option<&'a StockMap>,
    /// Firmware image (raw flash bytes).
    pub image: &'a [u8],
    /// Where the image loads (`wheel.toml` `board.app.base`).
    pub load_addr: u32,
    /// Core clock in Hz.
    pub hclk: u64,
    /// Sensor values of a static plant; for a live plant, the values it does not model.
    pub plant: PlantState,
    /// BLE bytes to send, with the simulated second to send them at.
    pub injections: Vec<(f64, Vec<u8>)>,
    /// Simulated seconds at which the battery is switched off and on.
    pub power_cycles: Vec<f64>,
    /// HCLK cycles per instruction (the Unicorn backend has no timing model).
    pub cpi: f64,
    /// Most instructions in one quantum.
    pub max_quantum: u32,
    /// Markers are watched for this many simulated seconds (watching costs speed).
    pub marker_window_s: f64,
}

/// Why a machine could not be built.
#[derive(Debug, Error)]
pub enum BuildError {
    /// The board or image is inconsistent.
    #[error("{0}")]
    Board(String),
    /// A component could not be built.
    #[error(transparent)]
    Config(#[from] euc_core::ConfigError),
    /// The CPU backend refused the memory layout.
    #[error("CPU backend: {0}")]
    Cpu(String),
}

/// A CPU backend factory: given the memory layout, returns the CPU.
pub type CpuFactory<'f> = dyn FnOnce(&[Region]) -> Result<Box<dyn Cpu>, String> + 'f;

/// A simulated controller.
pub struct Machine {
    cpu: Box<dyn Cpu>,
    pub(crate) sys: System,
    board: DeviceId,
    cpi: f64,
    max_quantum: u32,
    reset_sp: u32,
    reset_vector: u32,
    default_handler: u32,
    code: Span,
    marker_deadline: u64,
    watching: bool,
    /// Current cycle.
    pub cycles: u64,
    /// Instructions retired in full quanta.
    pub instr: u64,
    /// Quanta run.
    pub quanta: u64,
    /// Quanta cut short (poll fast-forward, reset, halt).
    pub early_quanta: u64,
    /// Resets after power-on.
    pub resets: u32,
}

impl Machine {
    /// Builds the machine with a static plant and powers it on. `registry` must know every kind
    /// in the board file except the simulator's own `plant` and `board` logic, which are added
    /// last.
    pub fn new(config: &MachineConfig<'_>, registry: &Registry, cpu: Box<CpuFactory<'_>>) -> Result<Self, BuildError> {
        Self::build(config, registry, cpu, None)
    }

    /// Builds the machine with a live plant (electrical plant and body physics) and powers it on.
    pub fn new_live(
        config: &MachineConfig<'_>,
        registry: &Registry,
        cpu: Box<CpuFactory<'_>>,
        live: Live,
    ) -> Result<Self, BuildError> {
        Self::build(config, registry, cpu, Some(live))
    }

    fn build(
        config: &MachineConfig<'_>,
        registry: &Registry,
        cpu: Box<CpuFactory<'_>>,
        live: Option<Live>,
    ) -> Result<Self, BuildError> {
        let file = config.board;
        let map = &file.memory;
        let mem = Memory::new(map, config.image, config.load_addr).map_err(BuildError::Board)?;
        let vector = |i: u32| {
            mem.read(config.load_addr + 4 * i, euc_core::Size::Word)
                .ok_or_else(|| BuildError::Board("the image has no vector table".to_owned()))
        };
        let (reset_sp, reset_vector) = (vector(0)?, vector(1)?);
        let default_handler = default_handler(&mem, config.load_addr);

        let mut names: Vec<String> = file.instances.iter().map(|i| i.name.clone()).collect();
        names.push(PLANT.to_owned());
        names.push(BOARD.to_owned());
        let ids = |name: &str| names.iter().position(|n| n == name).map(|i| i as DeviceId);
        let mut slots = Vec::new();
        for instance in &file.instances {
            let spec = instance.spec();
            let device = registry.build(&euc_core::Build::new(&spec, &ids))?;
            slots.push(Slot {
                name: spec.name,
                base: spec.base,
                size: spec.size,
                device: Some(device),
                timers: Vec::new(),
            });
        }
        let plant_id = slots.len() as DeviceId;
        slots.extend(own_slots(config, live));

        let mcu = &file.mcu;
        let exc = ExceptionUnit::new(mcu.irqs, mcu.priority_bits);
        let markers: Vec<(u32, String)> = config
            .stockmap
            .map(|map| map.entries.iter().filter(|e| e.marker).map(|e| (e.addr, e.name.clone())).collect())
            .unwrap_or_default();
        let mut sys = System::new(
            config.hclk,
            mem,
            map.clone(),
            slots,
            exc,
            Some(plant_id),
            mcu.irq_names.clone(),
            markers.iter().cloned().collect::<HashMap<_, _>>(),
        );
        for wire in &file.wires {
            let from = resolve(&sys, &names, &wire.from)?;
            let to = resolve(&sys, &names, &wire.to)?;
            sys.wire(from, to);
        }
        for route in &file.dma {
            let request = resolve(&sys, &names, &route.request)?;
            let (controller, channel) = route
                .channel
                .split_once('.')
                .and_then(|(name, ch)| Some((ids(name)?, ch.parse::<u8>().ok()?)))
                .ok_or_else(|| BuildError::Board(format!("DMA channel \"{}\" is not CONTROLLER.N", route.channel)))?;
            sys.route_dma(request, (controller, channel));
        }

        let code = sys.mem.code_span();
        let regions = layout(&mut sys, file);
        let mut cpu = cpu(&regions).map_err(BuildError::Cpu)?;
        let board = (names.len() - 1) as DeviceId;
        sys.reset_components(ResetKind::Power);
        cpu.reset(reset_sp, reset_vector);
        let marker_addrs: Vec<u32> = markers.iter().map(|&(addr, _)| addr).collect();
        cpu.watch(&marker_addrs);
        Ok(Self {
            cpu,
            sys,
            board,
            cpi: config.cpi,
            max_quantum: config.max_quantum,
            reset_sp,
            reset_vector,
            default_handler,
            code,
            marker_deadline: seconds(config.marker_window_s, config.hclk),
            watching: !marker_addrs.is_empty(),
            cycles: 0,
            instr: 0,
            quanta: 0,
            early_quanta: 0,
            resets: 0,
        })
    }

    /// Core clock in Hz.
    pub const fn hclk(&self) -> u64 {
        self.sys.hclk
    }

    /// Runs `cycles` more HCLK cycles (or until a fatal blocker). Returns the halt reason.
    pub fn run(&mut self, cycles: u64) -> Option<&str> {
        let end = self.cycles + cycles;
        self.sys.run_end = Some(end);
        while self.cycles < end && self.sys.diag.halted.is_none() {
            let now = self.cycles;
            self.sys.chunk_start = now;
            self.sys.skip = 0;
            self.sys.fire_due(now);
            if let Some((kind, why)) = self.sys.reset_pending.take() {
                self.system_reset(kind, &why);
            }
            if self.watching && now >= self.marker_deadline {
                self.cpu.watch(&[]);
                self.watching = false;
            }
            self.take_interrupt();
            let next = self.sys.next_event().map_or(end, |at| at.min(end));
            let budget = ((next as f64 - self.cycles as f64) / self.cpi) as i64;
            let budget = budget.clamp(1, i64::from(self.max_quantum)) as u32;
            self.sys.begin_quantum(self.cycles);
            let before = self.cpu.retired();
            let exit = self.cpu.run(&mut self.sys, budget);
            let executed = self.cpu.retired() - before;
            self.quanta += 1;
            self.handle(exit);
            if self.sys.early || self.sys.reset_pending.is_some() || self.sys.diag.halted.is_some() {
                self.early_quanta += 1;
                self.cycles = self.sys.chunk_start + self.sys.skip + 1;
            } else {
                self.instr += executed;
                self.cycles = self.sys.chunk_start + (executed as f64 * self.cpi) as u64 + self.sys.skip;
            }
        }
        self.sys.chunk_start = self.cycles;
        self.sys.skip = 0;
        self.sys.diag.halted.as_deref()
    }

    fn handle(&mut self, exit: Exit) {
        let pc = self.cpu.regs().pc;
        let (kind, text) = match exit {
            Exit::Budget | Exit::Yield => return,
            Exit::Fault(Fault::Bus(fault)) if fault.kind == BusFaultKind::ReadOnly => {
                ("write-code-flash", format!("store to code flash 0x{:08X} from PC 0x{pc:08X} (ignored)", fault.addr))
            }
            Exit::Fault(Fault::Bus(fault)) => {
                ("unmapped", format!("{} at 0x{:08X} from PC 0x{pc:08X}", fault.access, fault.addr))
            }
            Exit::Fault(fault) => ("cpu-exception", format!("{fault} at PC 0x{pc:08X} (HardFault/UsageFault on HW)")),
            other => ("cpu-exception", format!("{other:?} at PC 0x{pc:08X} (not modeled)")),
        };
        self.sys.blocker(kind, text, true);
    }

    /// Takes the best pending exception, after stepping out of an IT block.
    fn take_interrupt(&mut self) {
        let primask = self.cpu.regs().primask;
        let Some(exception) = self.sys.exc.best_pending(primask) else { return };
        for _ in 0..4 {
            if self.cpu.regs().xpsr & XPSR_IT == 0 {
                break;
            }
            let exit = self.cpu.run(&mut self.sys, 1);
            self.handle(exit);
            self.cycles += (self.cpi as u64).max(1);
            self.instr += 1;
        }
        let entered = self.sys.exc.enter(exception, self.cpu.regs_mut(), &mut MemoryBus(&mut self.sys.mem));
        let entry = match entered {
            Ok(entry) => entry,
            Err(fault) => {
                self.sys.blocker("cpu-exception", format!("exception entry: {fault}"), true);
                return;
            }
        };
        let lowest = self.sys.diag.min_frame.get_or_insert(entry.frame);
        *lowest = (*lowest).min(entry.frame);
        let name = self.exception_name(exception);
        if entry.vector & !1 == self.default_handler {
            self.sys.blocker("default-handler", format!("{name} taken but its vector is Default_Handler (b .)"), true);
        }
        if entry.vector == 0 || entry.vector == u32::MAX || !self.code.contains(entry.vector) {
            let text = format!("{name} vector = 0x{:08X} (VTOR 0x{:08X})", entry.vector, self.sys.exc.vtor());
            self.sys.blocker("bad-vector", text, true);
        }
    }

    fn system_reset(&mut self, kind: ResetKind, why: &str) {
        self.resets += 1;
        let what = if kind == ResetKind::Power { "POWER CYCLE" } else { "SYSTEM RESET" };
        let now = self.sys.now();
        self.sys.diag.events.push((now, format!("{what} #{}: {why}", self.resets)));
        self.sys.reset_components(kind);
        self.cpu.reset(self.reset_sp, self.reset_vector);
    }

    /// `HardFault`, `SysTick`, `TIM8_UP`, `IRQ61`...
    pub fn exception_name(&self, exception: u16) -> String {
        exception_name(exception, &self.sys.irq_names)
    }

    /// The board logic (BLE capture, button, latch).
    ///
    /// # Panics
    ///
    /// Never: the board logic is the last component from construction on and is idle between
    /// runs.
    pub fn board(&self) -> &BoardLogic {
        let device =
            self.sys.slots[usize::from(self.board)].device.as_deref().expect("board logic is idle between runs");
        (device as &dyn std::any::Any).downcast_ref().expect("the board slot holds BoardLogic")
    }

    /// Why the run stopped for good, if it did (a fatal blocker).
    pub fn halted(&self) -> Option<&str> {
        self.sys.diag.halted.as_deref()
    }

    /// The plant (static values, or the electrical plant and body physics).
    ///
    /// # Panics
    ///
    /// Never: the plant component exists from construction on and is idle between runs.
    pub fn plant(&self) -> &Plant {
        self.sys.plant_ref().expect("the plant is idle between runs")
    }

    /// The plant, to steer the rider or the stand between runs.
    ///
    /// # Panics
    ///
    /// Never, as for [`Machine::plant`].
    pub fn plant_mut(&mut self) -> &mut Plant {
        self.sys.plant_mut().expect("the plant is idle between runs")
    }

    /// The CPU backend.
    pub fn cpu(&self) -> &dyn Cpu {
        self.cpu.as_ref()
    }

    /// `len` bytes of plain memory at `addr` (SRAM or the flash code window).
    pub fn memory(&self, addr: u32, len: usize) -> Option<&[u8]> {
        self.sys.mem.slice(addr, len)
    }

    /// Exceptions taken, by exception number.
    pub fn exceptions_taken(&self) -> &[u64] {
        &self.sys.exc.taken
    }

    /// The component named `name` in the board file (`"display"`, `"buzzer"`, `"dma1"`), if it
    /// is a `T`. For viewers that mirror device state (display digits, LEDs, buzzer tone).
    pub fn component<T: euc_core::Peripheral>(&self, name: &str) -> Option<&T> {
        let slot = self.sys.slots.iter().find(|s| s.name == name)?;
        let device: &dyn std::any::Any = slot.device.as_deref()?;
        device.downcast_ref()
    }

    /// The component named `name`, if it is a `T`, to change it between runs (fault injection).
    pub fn component_mut<T: euc_core::Peripheral>(&mut self, name: &str) -> Option<&mut T> {
        let slot = self.sys.slots.iter_mut().find(|s| s.name == name)?;
        let device: &mut dyn std::any::Any = slot.device.as_deref_mut()?;
        device.downcast_mut()
    }

    /// The names of every component that is a `T`, in board order.
    pub fn component_names<T: euc_core::Peripheral>(&self) -> Vec<String> {
        self.sys
            .slots
            .iter()
            .filter(|slot| slot.device.as_deref().is_some_and(|d| (d as &dyn std::any::Any).is::<T>()))
            .map(|slot| slot.name.clone())
            .collect()
    }

    /// Takes the bytes the controller has sent over BLE since the last call (or since power-on),
    /// with the cycle each started on the wire. A long-running viewer drains them so they do not
    /// pile up; a report reads them all through [`Machine::board`] instead.
    pub fn take_ble_tx(&mut self) -> Vec<(u64, u8)> {
        let board = usize::from(self.board);
        let Some(device) = self.sys.slots[board].device.as_deref_mut() else { return Vec::new() };
        (device as &mut dyn std::any::Any)
            .downcast_mut::<BoardLogic>()
            .map(|logic| std::mem::take(&mut logic.ble_tx))
            .unwrap_or_default()
    }

    /// Sends `bytes` to the controller's BLE input now, as a phone would, while the machine runs
    /// (the configured injections are only for times known before the run).
    pub fn inject_ble(&mut self, bytes: &[u8]) {
        let now = self.cycles;
        let board = usize::from(self.board);
        let slot = &mut self.sys.slots[board];
        let Some(device) = slot.device.as_deref_mut() else { return };
        let Some(logic) = (device as &mut dyn std::any::Any).downcast_mut::<BoardLogic>() else { return };
        logic.at(now, Action::Inject(bytes.to_vec()));
        // Fire the board's timeline at the next quantum boundary.
        match slot.timers.iter_mut().find(|(tag, _)| *tag == crate::board::TIMELINE) {
            Some(timer) => timer.1 = timer.1.min(now),
            None => slot.timers.push((crate::board::TIMELINE, now)),
        }
    }
}

/// The simulator's own components, after the board file's: the plant, then the board logic.
fn own_slots(config: &MachineConfig<'_>, live: Option<Live>) -> [Slot; 2] {
    let file = config.board;
    let plant = match live {
        Some(live) => Plant::live(config.plant.clone(), file.analog.clone(), live),
        None => Plant::new(config.plant.clone(), file.analog.clone()),
    };
    let release_after = (file.board.release_after_latch_s * config.hclk as f64) as u64;
    let mut board = BoardLogic::new(file.board.button_active_low, release_after);
    for (at, bytes) in &config.injections {
        board.at(seconds(*at, config.hclk), Action::Inject(bytes.clone()));
    }
    for at in &config.power_cycles {
        board.at(seconds(*at, config.hclk), Action::PowerCycle);
    }
    let slot = |name: &str, device: Box<dyn euc_core::Peripheral>| Slot {
        name: name.to_owned(),
        base: None,
        size: 0,
        device: Some(device),
        timers: Vec::new(),
    };
    [slot(PLANT, Box::new(plant)), slot(BOARD, Box::new(board))]
}

/// Exception name for reports and messages.
pub fn exception_name(exception: u16, irq_names: &[String]) -> String {
    let system = match exception {
        1 => "Reset",
        2 => "NMI",
        3 => "HardFault",
        4 => "MemManage",
        5 => "BusFault",
        6 => "UsageFault",
        11 => "SVCall",
        12 => "DebugMon",
        14 => "PendSV",
        15 => "SysTick",
        e if e >= IRQ0 => {
            let irq = e - IRQ0;
            return irq_names.get(usize::from(irq)).cloned().unwrap_or_else(|| format!("IRQ{irq}"));
        }
        e => return format!("exception {e}"),
    };
    system.to_owned()
}

fn seconds(s: f64, hclk: u64) -> u64 {
    (s * hclk as f64) as u64
}

/// The handler most IRQ slots of the image's vector table share: the firmware's catch-all
/// `b .` loop. Taking it means an interrupt fired that the firmware never expected.
fn default_handler(mem: &Memory, load: u32) -> u32 {
    let vectors: Vec<u32> = (16..76).filter_map(|i| mem.read(load + 4 * i, euc_core::Size::Word)).collect();
    let mut counts: Vec<(u32, usize)> = Vec::new();
    for &vector in &vectors {
        match counts.iter_mut().find(|(v, _)| *v == vector) {
            Some((_, n)) => *n += 1,
            None => counts.push((vector, 1)),
        }
    }
    let best = counts.iter().map(|&(_, n)| n).max().unwrap_or(0);
    counts.iter().find(|&&(_, n)| n == best).map_or(0, |&(v, _)| v & !1)
}

/// `"USART3.rx"` as `(component, port)`.
fn resolve(sys: &System, names: &[String], end: &str) -> Result<(DeviceId, u8), BuildError> {
    let bad = |why: &str| BuildError::Board(format!("wire end \"{end}\": {why}"));
    let (name, port) = end.split_once('.').ok_or_else(|| bad("expected INSTANCE.PORT"))?;
    let id = names.iter().position(|n| n == name).ok_or_else(|| bad("unknown instance"))?;
    let device = sys.slots[id].device.as_deref().ok_or_else(|| bad("instance not built"))?;
    let port = device.port(port).ok_or_else(|| bad("no such port"))?;
    Ok((id as DeviceId, port))
}

/// The memory layout a CPU backend maps: plain memory by pointer, everything else MMIO.
fn layout(sys: &mut System, file: &BoardFile) -> Vec<Region> {
    let map = &file.memory;
    let flash = std::ptr::NonNull::from(&mut sys.mem.flash[0]);
    let sram = std::ptr::NonNull::from(&mut sys.mem.sram[0]);
    let code = sys.mem.code_span();
    let mmio = |base: u32, size: u32| Region { base, size, kind: RegionKind::Mmio };
    vec![
        Region { base: code.base, size: code.size, kind: RegionKind::Memory { ptr: flash, writable: false } },
        mmio(map.code_end, map.flash.end() - map.code_end),
        Region { base: map.sram.base, size: map.sram.size, kind: RegionKind::Memory { ptr: sram, writable: true } },
        mmio(map.sram_bitband, map.sram.size * 32),
        mmio(map.periph.base, map.periph.size),
        mmio(map.periph_bitband, map.periph.size * 32),
        mmio(map.ppb.base, map.ppb.size),
    ]
}

#[cfg(test)]
mod tests {
    use euc_core::{Bus, CoreRegs};

    use super::*;

    /// A CPU that never runs: enough to build machines and check their wiring.
    #[derive(Default)]
    struct Idle(CoreRegs);

    impl Cpu for Idle {
        fn reset(&mut self, sp: u32, reset_vector: u32) {
            self.0 = CoreRegs::at_reset(sp, reset_vector);
        }
        fn run(&mut self, _bus: &mut dyn Bus, _max_instr: u32) -> Exit {
            Exit::Budget
        }
        fn regs(&self) -> &CoreRegs {
            &self.0
        }
        fn regs_mut(&mut self) -> &mut CoreRegs {
            &mut self.0
        }
        fn retired(&self) -> u64 {
            0
        }
        fn watch(&mut self, _addrs: &[u32]) {}
        fn invalidate(&mut self, _addr: u32, _len: u32) {}
    }

    const BOARD: &str = r#"
schema = 1
[mcu]
name = "test"
irqs = 2
priority_bits = 4
irq_names = ["A", "B"]
[memory]
flash = { base = 0x0800_0000, size = 0x1_0000 }
code_end = 0x0800_E000
sram = { base = 0x2000_0000, size = 0x1_0000 }
sram_bitband = 0x2200_0000
periph = { base = 0x4000_0000, size = 0x3_0000 }
periph_bitband = 0x4200_0000
ppb = { base = 0xE000_0000, size = 0x10_0000 }
[[instance]]
name = "DWT"
kind = "cortex-m3.dwt"
base = 0xE000_1000
[board]
button_active_low = true
release_after_latch_s = 0.1
"#;

    fn build(extra: &str, image: &[u8]) -> Result<Machine, BuildError> {
        let board: BoardFile = toml::from_str(&format!("{BOARD}{extra}")).unwrap();
        let config = MachineConfig {
            board: &board,
            stockmap: None,
            image,
            load_addr: 0x0800_3000,
            hclk: 72_000_000,
            plant: PlantState::default(),
            injections: Vec::new(),
            power_cycles: Vec::new(),
            cpi: 1.25,
            max_quantum: 4000,
            marker_window_s: 0.0,
        };
        let mut registry = Registry::new();
        crate::board::register(&mut registry);
        Machine::new(&config, &registry, Box::new(|_: &[Region]| Ok(Box::new(Idle::default()) as Box<dyn Cpu>)))
    }

    fn error(extra: &str) -> String {
        build(extra, &[0; 0x200]).err().map(|e| e.to_string()).unwrap_or_default()
    }

    #[test]
    fn builds_and_resets_from_the_image_vectors() {
        let mut image = vec![0; 0x200];
        image[..8].copy_from_slice(&[0x00, 0x10, 0x00, 0x20, 0x01, 0x31, 0x00, 0x08]);
        let machine = build("", &image).unwrap();
        assert_eq!((machine.cpu().regs().sp(), machine.cpu().regs().pc), (0x2000_1000, 0x0800_3100));
        assert_eq!(machine.memory(0x0800_3000, 4), Some(&image[..4]));
    }

    #[test]
    fn wiring_mistakes_name_the_wire_end() {
        let wire = |from: &str, to: &str| format!("[[wire]]\nfrom = \"{from}\"\nto = \"{to}\"\n");
        assert_eq!(error(&wire("NOPE.tx", "board.ble_rx")), "wire end \"NOPE.tx\": unknown instance");
        assert_eq!(error(&wire("board.ble_tx", "board.nope")), "wire end \"board.nope\": no such port");
        assert_eq!(error(&wire("board", "board.latch")), "wire end \"board\": expected INSTANCE.PORT");
        let dma = "[[dma]]\nrequest = \"board.ble_tx\"\nchannel = \"DWT\"\n";
        assert_eq!(error(dma), "DMA channel \"DWT\" is not CONTROLLER.N");
        let unknown = "[[instance]]\nname = \"X\"\nkind = \"nope\"\n";
        assert_eq!(error(unknown), "instance X: unknown kind \"nope\"");
    }

    #[test]
    fn the_image_must_fit_the_code_window() {
        let message = build("", &vec![0; 0xB001]).err().map(|e| e.to_string()).unwrap_or_default();
        assert_eq!(message, "a 45057 byte image at 0x08003000 runs past 0x0800E000");
    }

    #[test]
    fn bit_band_aliases_read_modify_write_the_word() {
        let mut machine = build("", &[0; 0x200]).unwrap();
        let sys = &mut machine.sys;
        let alias = |byte: u32, bit: u32| 0x4200_0000 + byte * 32 + bit * 4;
        sys.write(0x4000_1000, euc_core::Size::Word, 0xF0).unwrap();
        sys.write(alias(0x1000, 4), euc_core::Size::Word, 0).unwrap();
        sys.write(alias(0x1000, 1), euc_core::Size::Word, 1).unwrap();
        assert_eq!(sys.read(0x4000_1000, euc_core::Size::Word), Ok(0xE2));
        assert_eq!(sys.read(alias(0x1000, 5), euc_core::Size::Word), Ok(1));
        assert!(sys.slots.iter().any(|slot| slot.name == "UNMODELED_40001000"), "unmodeled pages read back");
        sys.write(0x2200_0000 + 0x10 * 32 + 3 * 4, euc_core::Size::Word, 1).unwrap();
        assert_eq!(sys.mem.read(0x2000_0010, euc_core::Size::Word), Some(8));
    }

    #[test]
    fn exception_names() {
        let names = vec!["WWDG".to_owned()];
        let name = |e| exception_name(e, &names);
        assert_eq!(
            (name(3), name(15), name(16), name(17), name(9)),
            ("HardFault".into(), "SysTick".into(), "WWDG".into(), "IRQ1".into(), "exception 9".into())
        );
        let mut machine = build("", &[0; 0x200]).unwrap();
        assert!(MemoryBus(&mut machine.sys.mem).read(0x2000_0000, euc_core::Size::Word).is_ok());
    }
}
