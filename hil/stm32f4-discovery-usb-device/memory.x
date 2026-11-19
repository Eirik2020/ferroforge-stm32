/* STM32F407VG (RM0090 section 2.3): 1 MiB of flash, SRAM1 and SRAM2
   contiguous at 0x2000_0000 (128 KiB). CCM RAM is left out: OTG_FS has no
   DMA here, but only the core reaches CCM. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
