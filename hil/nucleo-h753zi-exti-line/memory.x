/* STM32H753ZI (RM0433 section 2.3.2, DS12117 section 3.5): 2 MiB of flash at
   0x0800_0000 and the 512 KiB AXI SRAM at 0x2400_0000, as the other H753
   benches use. This one has no DMA. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   : ORIGIN = 0x24000000, LENGTH = 512K
}
