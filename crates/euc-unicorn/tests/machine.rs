//! The whole machine on the Unicorn backend with a hand-assembled program and a test
//! peripheral: NVIC enable over MMIO, a level interrupt from a component, exception entry at a
//! quantum boundary, inline exception return, and poll fast-forward.

#![allow(unsafe_code)]

use euc_core::{Cpu, Ctx, Peripheral, Region, Registry, ResetKind};
use euc_sim::config::BoardFile;
use euc_sim::{Machine, MachineConfig, PlantState, Report};
use euc_unicorn::UnicornCpu;

const HCLK: u64 = 72_000_000;
const TICK: u64 = HCLK / 1000;
const LOAD: u32 = 0x0800_3000;

/// Raises IRQ 0 every millisecond; `STATUS` (offset 0) counts ticks, a write to offset 4 acks.
struct Ticker {
    ticks: u32,
    next: u64,
}

impl Peripheral for Ticker {
    fn reset(&mut self, _kind: ResetKind, ctx: &mut Ctx<'_>) {
        self.ticks = 0;
        self.next = ctx.now() + TICK;
        ctx.schedule(self.next, 0);
    }

    fn read(&mut self, off: u32, _mask: u32, _ctx: &mut Ctx<'_>) -> u32 {
        if off == 0 { self.ticks } else { 0 }
    }

    fn write(&mut self, off: u32, _value: u32, _mask: u32, ctx: &mut Ctx<'_>) {
        if off == 4 {
            ctx.set_irq(0, false);
        }
    }

    fn on_event(&mut self, _tag: u32, ctx: &mut Ctx<'_>) {
        self.ticks += 1;
        ctx.set_irq(0, true);
        self.next += TICK;
        ctx.schedule(self.next, 0);
    }
}

const BOARD: &str = r#"
schema = 1

[mcu]
name = "test"
irqs = 4
priority_bits = 4
irq_names = ["TICK", "B", "C", "D"]

[memory]
flash = { base = 0x0800_0000, size = 0x1_0000 }
code_end = 0x0800_E000
sram = { base = 0x2000_0000, size = 0x1_0000 }
sram_bitband = 0x2200_0000
periph = { base = 0x4000_0000, size = 0x3_0000 }
periph_bitband = 0x4200_0000
ppb = { base = 0xE000_0000, size = 0x10_0000 }

[[instance]]
name = "TICKER"
kind = "test.ticker"
base = 0x4000_0000

[board]
button_active_low = true
release_after_latch_s = 0.1
"#;

/// `movw rd, #imm16` (T3).
fn movw(rd: u16, imm: u16) -> [u16; 2] {
    [0xF240 | ((imm >> 11) & 1) << 10 | imm >> 12, ((imm >> 8) & 7) << 12 | rd << 8 | (imm & 0xFF)]
}

/// `movt rd, #imm16` (T1).
fn movt(rd: u16, imm: u16) -> [u16; 2] {
    let [hi, lo] = movw(rd, imm);
    [hi | 0x0080, lo]
}

fn image() -> Vec<u8> {
    let mut image = vec![0_u8; 0x200];
    let mut put32 = |at: usize, word: u32| image[at..at + 4].copy_from_slice(&word.to_le_bytes());
    put32(0, 0x2000_1000); // initial SP
    put32(4, LOAD + 0x101); // reset
    put32(16 * 4, LOAD + 0x141); // IRQ0
    let mut code = Vec::new();
    // Reset: point VTOR at the table, enable IRQ0, poll TICKER.STATUS until it reaches 3, spin.
    code.extend(movw(0, 0xED08));
    code.extend(movt(0, 0xE000));
    code.extend(movw(1, 0x3000));
    code.extend(movt(1, 0x0800));
    code.push(0x6001); // str r1,[r0]
    code.extend(movw(0, 0xE100));
    code.extend(movt(0, 0xE000));
    code.extend([0x2101, 0x6001]); // movs r1,#1; str r1,[r0]
    code.extend(movw(2, 0x0000));
    code.extend(movt(2, 0x4000));
    code.extend([0x6813, 0x2B03, 0xDBFC, 0xE7FE]); // loop: ldr r3,[r2]; cmp r3,#3; blt loop; b .
    // IRQ0 handler at +0x140: count in RAM, ack the ticker, return.
    code.extend(movw(0, 0x0000));
    code.extend(movt(0, 0x2000));
    code.extend([0x6801, 0x3101, 0x6001]); // ldr r1,[r0]; adds r1,#1; str r1,[r0]
    code.extend(movw(2, 0x0004));
    code.extend(movt(2, 0x4000));
    code.extend([0x6011, 0x4770]); // str r1,[r2]; bx lr
    let (reset, handler) = code.split_at(23);
    for (i, half) in reset.iter().enumerate() {
        image[0x100 + 2 * i..0x102 + 2 * i].copy_from_slice(&half.to_le_bytes());
    }
    for (i, half) in handler.iter().enumerate() {
        image[0x140 + 2 * i..0x142 + 2 * i].copy_from_slice(&half.to_le_bytes());
    }
    image
}

#[test]
fn interrupts_polls_and_returns_work_end_to_end() {
    let board: BoardFile = toml::from_str(BOARD).unwrap();
    let image = image();
    let config = MachineConfig {
        board: &board,
        stockmap: None,
        image: &image,
        load_addr: LOAD,
        hclk: HCLK,
        plant: PlantState::default(),
        injections: Vec::new(),
        power_cycles: Vec::new(),
        cpi: 1.25,
        max_quantum: 4000,
        marker_window_s: 0.0,
    };
    let mut registry = Registry::new();
    registry.register("test.ticker", |_| Ok(Box::new(Ticker { ticks: 0, next: 0 })));
    let cpu = Box::new(|regions: &[Region]| {
        // SAFETY: `Machine::new` owns the memory behind every region in boxed buffers that never
        // move and outlive the CPU, and touches it during a run only from bus callbacks.
        let cpu = unsafe { UnicornCpu::new(regions) }.map_err(|e| e.to_string())?;
        Ok(Box::new(cpu) as Box<dyn Cpu>)
    });
    let mut machine = Machine::new(&config, &registry, cpu).unwrap();
    machine.run(10 * TICK + TICK / 2);

    let report = Report::new(&machine, &[]);
    assert_eq!(report.halted, None, "{:#?}", report.blockers);
    assert!(report.blockers.is_empty(), "{:#?}", report.blockers);
    let counter = u32::from_le_bytes(machine.memory(0x2000_0000, 4).unwrap().try_into().unwrap());
    assert_eq!(counter, 10, "one handler run per millisecond tick");
    assert_eq!(machine.exceptions_taken()[16], 10);
    assert_eq!(report.nvic_enables[0].what, "NVIC enable TICK");
    let poll = &report.poll_fast_forward[0];
    assert_eq!(poll.register, "TICKER.+000");
    assert!(poll.skipped_ms > 1.5, "the status poll was fast-forwarded: {poll:?}");
}
