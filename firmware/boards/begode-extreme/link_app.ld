/*
 * Begode Extreme main board: application image, linked behind the 12 KB vendor bootloader.
 *
 * Flash (STM32F103RC class, 2 KB pages):
 *   0x08000000  bootloader, 12 KB (never touched by the image)
 *   0x08003000  application image, at most 48 KB          <- this script
 *   0x0800F000  settings journal page   \
 *   0x0800F800  IMU calibrated marker    |  storage pages: the image must end before them,
 *   0x08010000  IMU offsets              |  or an OTA update would erase the wheel's data
 *   0x08010800  bootloader request page /
 *
 * Image layout:
 *   +0x000  .isr_vector  76-entry vector table (0x130 bytes); the bootloader starts the image through it
 *   +0x130  16 bytes of zero padding
 *   +0x140  .euc_info    128-byte image info block (docs/spec/euc-info.md)
 *   +0x1C0  .text, .rodata, then the initial values of .data
 *
 * RAM: 20 KB assumed (the part has 48 KB; the stock firmware uses under 8 KB). The main stack sits at the
 * top, .data and .bss at the bottom, and the space between them is unused. There is no heap.
 *
 * Symbols with the ld_ prefix are read by C code (ports/stm32f1/startup.c) and by tools/size_report.py.
 */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08003000, LENGTH = 48K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

ld_stack_size = 2K;

SECTIONS
{
    .isr_vector ORIGIN(FLASH) :
    {
        ld_image_start = .;
        KEEP(*(.isr_vector))
    } > FLASH

    .euc_info ORIGIN(FLASH) + 0x140 :
    {
        KEEP(*(.euc_info))
    } > FLASH

    .text : ALIGN(4)
    {
        *(.text .text.*)
    } > FLASH

    .rodata : ALIGN(4)
    {
        *(.rodata .rodata.*)
    } > FLASH

    /* Unwind index; only present if libgcc brings one along. */
    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx .ARM.exidx.* .gnu.linkonce.armexidx.*)
    } > FLASH

    .data : ALIGN(4)
    {
        ld_data_start = .;
        *(.data .data.*)
        . = ALIGN(4);
        ld_data_end = .;
    } > RAM AT > FLASH
    ld_data_load = LOADADDR(.data);
    ld_image_end = LOADADDR(.data) + SIZEOF(.data);
    ld_image_size = ld_image_end - ld_image_start;

    .bss (NOLOAD) : ALIGN(4)
    {
        ld_bss_start = .;
        *(.bss .bss.* COMMON)
        . = ALIGN(4);
        ld_bss_end = .;
    } > RAM

    .stack ORIGIN(RAM) + LENGTH(RAM) - ld_stack_size (NOLOAD) :
    {
        ld_stack_bottom = .;
        . = . + ld_stack_size;
        ld_stack_top = .;
    } > RAM

    /DISCARD/ :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    }
}

/* Storage pages the application reads and writes (2 KB each; see the map at the top). */
ld_settings_page = 0x0800F000;
ld_calibrated_page = 0x0800F800;
ld_imu_offsets_page = 0x08010000;
ld_update_request_page = 0x08010800;

/* The checks the OTA image gate repeats (docs/spec/euc-info.md), made at link time. */
ASSERT(ORIGIN(FLASH) % 512 == 0, "VTOR needs the 76-entry vector table on a 512-byte boundary")
ASSERT(SIZEOF(.isr_vector) == 0x130, "vector table must have 76 entries (0x130 bytes)")
ASSERT(ADDR(.euc_info) == 0x08003140, ".euc_info must sit at image offset 0x140")
ASSERT(SIZEOF(.euc_info) == 128, ".euc_info must be 128 bytes")
ASSERT(ld_image_end <= 0x0800F000, "image overlaps the storage pages at 0x0800F000")
ASSERT(ld_image_size <= 49152, "image is larger than 48 KB")
ASSERT(ld_bss_end <= ld_stack_bottom, ".data + .bss overlap the stack")
ASSERT(ld_stack_top % 8 == 0, "AAPCS needs an 8-byte aligned initial stack pointer")
