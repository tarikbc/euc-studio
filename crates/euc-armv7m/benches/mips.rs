//! Raw interpreter speed in instructions per second on three kernels that look like compiled
//! firmware: a load/ALU/store loop, a bitwise CRC-32 with an IT block, and calls with
//! `PUSH`/`POP`, a halfword table and a compare chain. Memory is plain RAM, so this measures the
//! core alone; `tests/stock_boot.rs` measures the whole machine on the stock image.
//!
//! ```text
//! cargo bench -p euc-armv7m --bench mips
//! ```

#![allow(unsafe_code)]

use std::ptr::NonNull;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use euc_armv7m::Armv7m;
use euc_core::{Bus, BusFault, BusFaultKind, CoreRegs, Cpu, Exit, Region, RegionKind, Size};

const FLASH: u32 = 0x0800_0000;
const RAM: u32 = 0x2000_0000;
const SIZE: u32 = 0x1_0000;
/// Instructions per measured run.
const BATCH: u32 = 1_000_000;

/// The kernels, assembled with `llvm-mc -triple=thumbv7m-none-eabi`: `mix` at +0x00, `crc` at
/// +0x40, `calls` at +0x80 (source in the comments of each benchmark below).
const KERNELS: [u16; 89] = [
    0xF240, 0x0000, 0xF2C2, 0x0000, 0xF242, 0x0100, 0xF2C2, 0x0100, 0xF44F, 0x6200, 0xF850, 0x3B04, 0x441C, 0xEA85,
    0x15F3, 0xFB03, 0xF604, 0xF841, 0x6B04, 0x1E52, 0xD1F4, 0xE7E9, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xF240, 0x0100, 0xF2C2, 0x0100, 0xF44F, 0x7280, 0xF248, 0x3320, 0xF6CE, 0x53B8,
    0xF06F, 0x0000, 0xF811, 0x4B01, 0x4060, 0x2508, 0x0840, 0xBF28, 0x4058, 0x3D01, 0xD1FA, 0x3A01, 0xD1F4, 0xE7E7,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xF240, 0x0700, 0xF2C2, 0x0700, 0x2600, 0x4630,
    0xF000, 0xF804, 0x3601, 0x2EC8, 0xDBF9, 0xE7F3, 0xB530, 0xF000, 0x0407, 0xF837, 0x5014, 0x2D64, 0xD801, 0x182D,
    0xE000, 0x1A2D, 0xF827, 0x5014, 0xBD30,
];

/// No MMIO in these kernels.
struct NoMmio;

impl Bus for NoMmio {
    fn read(&mut self, addr: u32, _size: Size) -> Result<u32, BusFault> {
        Err(BusFault { addr, access: euc_core::Access::Read, kind: BusFaultKind::Unmapped })
    }
    fn write(&mut self, addr: u32, _size: Size, _value: u32) -> Result<(), BusFault> {
        Err(BusFault { addr, access: euc_core::Access::Write, kind: BusFaultKind::Unmapped })
    }
    fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
        Err(BusFault { addr, access: euc_core::Access::Fetch, kind: BusFaultKind::Unmapped })
    }
    fn exception_return(&mut self, _regs: &mut CoreRegs, _exc_return: u32) {}
}

struct Rig {
    _flash: Box<[u8]>,
    _ram: Box<[u8]>,
    cpu: Armv7m,
}

fn rig(entry: u32) -> Rig {
    let mut flash = vec![0; SIZE as usize].into_boxed_slice();
    for (i, h) in KERNELS.iter().enumerate() {
        flash[2 * i..2 * i + 2].copy_from_slice(&h.to_le_bytes());
    }
    let mut ram: Box<[u8]> = (0..SIZE).map(|i| (i.wrapping_mul(2_654_435_761) >> 13) as u8).collect();
    let regions = [
        Region {
            base: FLASH,
            size: SIZE,
            kind: RegionKind::Memory { ptr: NonNull::from(&mut flash[0]), writable: false },
        },
        Region { base: RAM, size: SIZE, kind: RegionKind::Memory { ptr: NonNull::from(&mut ram[0]), writable: true } },
    ];
    // SAFETY: both buffers are boxed and live in the rig next to the core.
    let mut cpu = unsafe { Armv7m::new(&regions) }.unwrap();
    cpu.reset(RAM + SIZE, (FLASH + entry) | 1);
    Rig { _flash: flash, _ram: ram, cpu }
}

fn kernels(c: &mut Criterion) {
    let mut group = c.benchmark_group("mips");
    group.throughput(Throughput::Elements(u64::from(BATCH)));
    for (name, entry) in [
        // ldr r3,[r0],#4; add r4,r3; eor.w r5,r5,r3,ror #7; mul r6,r3,r4; str r6,[r1],#4; subs; bne
        ("load_alu_store", 0x00),
        // ldrb; eors; movs; loop of lsrs / it cs / eorcs / subs / bne
        ("crc32_it", 0x40),
        // bl leaf; push {r4,r5,lr}; and; ldrh [r7,r4,lsl #1]; cmp; bhi; adds/subs; strh; pop {..,pc}
        ("calls", 0x80),
    ] {
        let mut rig = rig(entry);
        group.bench_function(name, |b| {
            b.iter(|| assert_eq!(rig.cpu.run(&mut NoMmio, BATCH), Exit::Budget));
        });
    }
    group.finish();
}

criterion_group!(benches, kernels);
criterion_main!(benches);
