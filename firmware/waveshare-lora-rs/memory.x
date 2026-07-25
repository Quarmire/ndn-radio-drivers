/* GD32F103C8 (STM32F103 clone): 128K flash usable (GD32 C8 has 128K), 20K RAM.
   Standalone at 0x08000000 (no bootloader — reflashed via ST-Link/openocd or, once
   CMD_ENTER_BOOTLOADER is present, over USB via stm32flash).

   The top 8 bytes of SRAM (from 0x20004FF8) are carved OUT of the linker's RAM region and hold a
   reset-surviving "boot flag": CMD_ENTER_BOOTLOADER writes a magic there and does a system reset;
   the #[pre_init] hook reads it and jumps to the ROM bootloader from a clean reset state. It sits at
   _stack_start (top of RAM) — the stack grows DOWN from there, so it is never clobbered, and a
   SYSRESETREQ retains SRAM, so the magic survives the reset. 8 (not 4) bytes keep _stack_start
   8-byte aligned as AAPCS requires. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  RAM   : ORIGIN = 0x20000000, LENGTH = 20K - 8
}
