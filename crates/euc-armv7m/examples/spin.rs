//! Runs one of the benchmark kernels for a few seconds and prints the instruction rate: a target
//! for a sampling profiler.
//!
//! ```text
//! cargo run -p euc-armv7m --release --example spin -- [mix|crc|calls] [seconds]
//! ```

#![allow(unsafe_code)]

use std::ptr::NonNull;
use std::time::Instant;

use euc_armv7m::Armv7m;
use euc_core::{Access, Bus, BusFault, BusFaultKind, CoreRegs, Cpu, Exit, Region, RegionKind, Size};

const FLASH: u32 = 0x0800_0000;
const RAM: u32 = 0x2000_0000;
const SIZE: u32 = 0x1_0000;

/// The kernels of `benches/mips.rs`.
const KERNELS: [u16; 89] = [
    0xF240, 0x0000, 0xF2C2, 0x0000, 0xF242, 0x0100, 0xF2C2, 0x0100, 0xF44F, 0x6200, 0xF850, 0x3B04, 0x441C, 0xEA85,
    0x15F3, 0xFB03, 0xF604, 0xF841, 0x6B04, 0x1E52, 0xD1F4, 0xE7E9, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xF240, 0x0100, 0xF2C2, 0x0100, 0xF44F, 0x7280, 0xF248, 0x3320, 0xF6CE, 0x53B8,
    0xF06F, 0x0000, 0xF811, 0x4B01, 0x4060, 0x2508, 0x0840, 0xBF28, 0x4058, 0x3D01, 0xD1FA, 0x3A01, 0xD1F4, 0xE7E7,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xF240, 0x0700, 0xF2C2, 0x0700, 0x2600, 0x4630,
    0xF000, 0xF804, 0x3601, 0x2EC8, 0xDBF9, 0xE7F3, 0xB530, 0xF000, 0x0407, 0xF837, 0x5014, 0x2D64, 0xD801, 0x182D,
    0xE000, 0x1A2D, 0xF827, 0x5014, 0xBD30,
];

struct NoMmio;

impl Bus for NoMmio {
    fn read(&mut self, addr: u32, _size: Size) -> Result<u32, BusFault> {
        Err(BusFault { addr, access: Access::Read, kind: BusFaultKind::Unmapped })
    }
    fn write(&mut self, addr: u32, _size: Size, _value: u32) -> Result<(), BusFault> {
        Err(BusFault { addr, access: Access::Write, kind: BusFaultKind::Unmapped })
    }
    fn fetch(&mut self, addr: u32) -> Result<u16, BusFault> {
        Err(BusFault { addr, access: Access::Fetch, kind: BusFaultKind::Unmapped })
    }
    fn exception_return(&mut self, _regs: &mut CoreRegs, _exc_return: u32) {}
}

fn main() {
    let mut args = std::env::args().skip(1);
    let entry = match args.next().as_deref() {
        Some("crc") => 0x40,
        Some("calls") => 0x80,
        _ => 0x00,
    };
    let seconds: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(5.0);
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
    // SAFETY: both buffers live until the end of `main`, next to the core.
    let mut cpu = unsafe { Armv7m::new(&regions) }.unwrap();
    cpu.reset(RAM + SIZE, (FLASH + entry) | 1);
    let started = Instant::now();
    let mut runs = 0_u64;
    while started.elapsed().as_secs_f64() < seconds {
        assert_eq!(cpu.run(&mut NoMmio, 4000), Exit::Budget);
        runs += 1;
    }
    let wall = started.elapsed().as_secs_f64();
    println!(
        "{:.1} MIPS ({} instructions in {wall:.2} s, quanta of 4000)",
        cpu.retired() as f64 / wall / 1e6,
        runs * 4000
    );
}
