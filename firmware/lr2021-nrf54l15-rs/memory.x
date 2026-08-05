/* nRF54L15 application core.
 *
 * RRAM 1524 KB @ 0x00000000, RAM 256 KB @ 0x20000000 (secure "app-s" view).
 *
 * VERIFY BEFORE FLASHING: if the XIAO ships a UF2/MBR bootloader, FLASH ORIGIN must move past it
 * and LENGTH shrink accordingly. Read the board's own bootloader map — do not assume this. Getting
 * it wrong bricks nothing (RRAM is reflashable over SWD) but the image will not run.
 */
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 1524K
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
