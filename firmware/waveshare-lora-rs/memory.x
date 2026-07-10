/* GD32F103C8 (STM32F103 clone): 128K flash usable (GD32 C8 has 128K), 20K RAM.
   Standalone at 0x08000000 (no bootloader — reflashed via ST-Link/openocd). */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  RAM   : ORIGIN = 0x20000000, LENGTH = 20K
}
