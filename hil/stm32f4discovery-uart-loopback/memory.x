/* STM32F407VG (RM0090 section 2.3): 1 MiB of flash at 0x0800_0000; SRAM1
   (112 KiB) and SRAM2 (16 KiB) contiguous at 0x2000_0000. The 64 KiB of CCM
   RAM is left out: DMA does not reach it. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
