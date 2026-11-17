/* STM32H753ZI (RM0433 section 2.3.2, DS12117 section 3.5): 2 MiB of flash at
   0x0800_0000. Everything runs from the 512 KiB AXI SRAM at 0x2400_0000,
   because DMA1 and DMA2 cannot reach the DTCM at 0x2000_0000 (RM0433
   figure 1), and the UART buffers are statics. The data cache stays off,
   so they need no non-cacheable region. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   : ORIGIN = 0x24000000, LENGTH = 512K
}
