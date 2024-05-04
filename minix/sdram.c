#include "vpu.h"
#include "udelay.h"

#define PM_KEY ((uint32_t) 0x5A000000)
#define CM_KEY ((uint32_t) 0x5A000000)
#define A2W_KEY ((uint32_t) 0x5A000000)
void uart_init(void)
{
  int i = 0;
  GPIO->GPFSEL1 = GPIO->GPFSEL1 & ~((7 << 12) | (7 << 15));
  GPIO->GPFSEL1 = GPIO->GPFSEL1 | (2 << 12) | (2 << 15);
  GPIO->GPPUD = 0;
  udelay(150);
  for(i = 0; i < 150; i++);
  GPIO->GPPUDCLK0 = (1 << 14) | (1 << 15);
  for(i = 0; i < 150; i++);
  udelay(150);
  GPIO->GPPUDCLK0 = 0; 

  AUX->AUX_ENABLES = 1;
  AUX->AUX_MU_IER_REG = 0;
  AUX->AUX_MU_CNTL_REG = 0;
  AUX->AUX_MU_LCR_REG = 3;
  AUX->AUX_MU_MCR_REG = 0;
  AUX->AUX_MU_IER_REG = 0;
  AUX->AUX_MU_IIR_REG = 0xC6;
  AUX->AUX_MU_BAUD_REG = ((100000000/(115200*8)) - 1);
  AUX->AUX_MU_LCR_REG = 0x03;
  AUX->AUX_MU_CNTL_REG = 3;
}
void sdram_init(void)
{
  uint32_t cpuid, m, vendor_id, bc, memsize;
  int colbits, rowbits, banklow;

  PWRMAN->SMPS = PM_KEY | 0x1; //PM_SMPS_CTRLEN;
  A2W->SMPS_LDO1 = A2W_KEY | (0x40000);
  A2W->SMPS_LDO0 = A2W_KEY | 0x0;
  A2W->XOSC_CTRL |= A2W_KEY | (0x10); //A2W_XOSC_CTRL_DDREN;

  /* Switch to CPRMAN Clock */
  CM->SDCDIV = CM_KEY | (1 << 12); //CM_IDIV=1
  CM->SDCCTL = CM_KEY | (CM->SDCCTL & 0xFFFFFFF0) | 1; //CM_SRC_OSC
  CM->SDCCTL |= CM_KEY | (0x10); // CM_ENAB
  while(!(CM->SDCCTL & 0x80)); // CM_BUSY

  /* Update clk manager */
  CM->SDCCTL |= CM_KEY | (0x00020000); //CM_SDCTL_UPDATE_SET
  while(!(CM->SDCCTL & 0x00010000)); //CM_SDCTL_ACCEPT_SET

  CM->SDCCTL = CM_KEY | (CM->SDCCTL & 0xFFFF0FFF); //CM_SDCTL_CTRL_CLR

  CM->SDCCTL = CM_KEY | (CM->SDCCTL & 0xFFFDFFFF); //CM_SDCTL_UPDATE_CLR
  while(CM->SDCCTL & 0x00010000); //CM_SDCTL_ACCEPT_SET

  /* Reset SDRAM PHY */
  SDRAMCTL->PHYC = 1; //SD_PHYC_PHYRST_BITS
  udelay(64);
  SDRAMCTL->PHYC = 0;

  /* Reset DPHY CTRL */
  DPHYCSR->DQ_PHY_MISC_CTRL = 0x7;
  DPHYCSR->DQ_PAD_MISC_CTRL = 0x0;
  DPHYCSR->BOOT_READ_DQS_GATE_CTRL = 0x11;

  /* Reset PHY DLL */
  APHYCSR->PHY_BIST_CNTRL_SPR = 0x30;

  DPHYCSR->GLBL_DQ_DLL_RESET = 0x1;
  APHYCSR->GLBL_ADDR_DLL_RESET = 0x1;

  /* Stall */
  SDRAMCTL->CS;
  SDRAMCTL->CS;
  SDRAMCTL->CS;
  SDRAMCTL->CS;

  DPHYCSR->GLBL_DQ_DLL_RESET = 0x0;
  APHYCSR->GLBL_ADDR_DLL_RESET = 0x0;
  while((DPHYCSR->GLBL_MSTR_DLL_LOCK_STAT & 0xFFFF) != 0xFFFF);

  SDRAMCTL->SA = 0x006E3395;
  SDRAMCTL->SB = 0x0F9;
  SDRAMCTL->SC = 0x6000431;
  SDRAMCTL->SD = 0x10000011;
  SDRAMCTL->SE = 0x10106000;
  SDRAMCTL->PT1 = 0x0AF002;
  SDRAMCTL->PT2 = 0x8C;
  SDRAMCTL->MRT = 0x3;
  SDRAMCTL->CS = 0x200042;

  while(!(SDRAMCTL->CS & 0x00008000)); //SD_CS_SDUP_SET

  while(!(SDRAMCTL->MR & 0x80000000)); //SD_MR_DONE_SET
  SDRAMCTL->MR = ((0x2) & 0xFF) | (((0x4) & 0xFF)  << 8) | 0x10000000; //SD_MR_RW_SET
  
  __asm__("version %0" : "=r" (cpuid));

  while(!(SDRAMCTL->MR & 0x80000000)); //SD_MR_DONE_SET
  SDRAMCTL->MR = (0xFF & 0xFF) | ((0 & 0xFF) << 8) | 0x10000000; //SD_MR_RW_SET
  while(!(SDRAMCTL->MR & 0x80000000));
  
  while(!(SDRAMCTL->MR & 0x80000000)); //SD_MR_DONE_SET
  SDRAMCTL->MR = (0x2 & 0xFF) | ((0x4 & 0xFF) << 8) | 0x10000000; //SD_MR_RW_SET
  while(!(SDRAMCTL->MR & 0x80000000));

  APHYCSR->ADDR_PAD_DRV_SLEW_CTRL = 0x333;
  if(((cpuid >> 4) & 0xFFF) == 0x14)
  {
    DPHYCSR->DQ_PAD_DRV_SLEW_CTRL = (2 << 8) | (2 << 4) | (3);
  }
  else
  {
    DPHYCSR->DQ_PAD_DRV_SLEW_CTRL = (3 << 8) | (3 << 4) | (3);
  }
  APHYCSR->PHY_BIST_CNTRL_SPR = 0x20;

  APHYCSR->ADDR_PVT_COMP_CTRL = 0x1;
  while(!(APHYCSR->ADDR_PVT_COMP_STATUS & 2));

  DPHYCSR->DQ_PVT_COMP_CTRL = 0x1;
  while(!(DPHYCSR->DQ_PVT_COMP_STATUS & 2));

  APHYCSR->PHY_BIST_CNTRL_SPR = 0;

  m = SDRAMCTL->MRT;
  SDRAMCTL->MRT = 20;
  SDRAMCTL->MR = (10 & 0xFF) | ((0 & 0xFF) << 8) | 0x10000000 | 0x20000000;
  while(!(SDRAMCTL->MR & 0x80000000));

  if(((cpuid >> 4) & 0xFFF) == 0x14)
  {
    while(!(SDRAMCTL->MR & 0x80000000));
    SDRAMCTL->MR = (3 & 0xFF) | ((3 & 0xFF)  << 8) | 0x10000000;
  }
  else
  {
    while(!(SDRAMCTL->MR & 0x80000000));
    SDRAMCTL->MR = (2 & 0xFF) | ((3 & 0xFF)  << 8) | 0x10000000;
  }

  /* Read vendor ID */
  while(!(SDRAMCTL->MR & 0x80000000));
  SDRAMCTL->MR = (5 & 0xFF);
  while(!((m = SDRAMCTL->MR) & 0x80000000));

  if(m & 0x40000000) //SD_MR_TIMEOUT_SET
  {
    //failed to read manufacturer id
  }
  else
  {
    vendor_id = (m & 0x00FF0000) >> 16; //SD_MR_RDATA_LSB
  }

  /* Read basic configuration */
  while(!(SDRAMCTL->MR & 0x80000000));
  SDRAMCTL->MR = (8 & 0xFF);
  while(!((m = SDRAMCTL->MR) & 0x80000000));

  if(m & 0x40000000) //SD_MR_TIMEOUT_SET
  {
    //failed to read basic configuration
  }
  else
  {
    bc = (m & 0x00FF0000) >> 16; //SD_MR_RDATA_LSB
  }

  switch(bc)
  {
	  case 0x58:
		  memsize = 1024;
		  break;
	  case 0x18:
		  memsize = 512;
		  break;
	  case 0x14:
		  memsize = 256;
		  break;
	  case 0x10:
		  memsize = 128;
		  break;
	  default:
	          memsize = 0;
  }
  if(memsize == 0)
  {
    AUX->AUX_MU_IO_REG = 'x';
    while(!(AUX->AUX_MU_LSR_REG & (1 << 6)));
    return;
  }	  
  /* Enable high-frequency SDRAM PLL */  
  SDRAMCTL->CS = (SDRAMCTL->CS & ~(0x00040000 | 0x00000004 | 
		  0x00000001)) | 0x00000008;
  while(SDRAMCTL->CS & 0x00008000);

  /* Update clock manager */
  CM->SDCCTL |= CM_KEY | (0x00020000); //CM_SDCTL_UPDATE_SET
  while(!(CM->SDCCTL & 0x00010000)); //CM_SDCTL_ACCEPT_SET

  CM->SDCCTL = CM_KEY | (CM->SDCCTL & ~(0x00000010 | 0x0000F000));

  CM->SDCCTL = CM_KEY | (CM->SDCCTL & 0xFFFDFFFF); //CM_SDCTL_UPDATE_CLR
  while(CM->SDCCTL & 0x00010000); //CM_SDCTL_ACCEPT_SET

  /* Migrate over to master PLL */
  APHYCSR->DDR_PLL_PWRDWN = 0;
  APHYCSR->DDR_PLL_GLOBAL_RESET = 0;
  APHYCSR->DDR_PLL_POST_DIV_RESET = 0;

  /* 400 MHz */
  /* 19.2MHz * 83 / 4 = 398.4 MHz */
  APHYCSR->DDR_PLL_VCO_FREQ_CNTRL0 = (1 << 16) | (83);
  APHYCSR->DDR_PLL_VCO_FREQ_CNTRL1 = 0;
  APHYCSR->DDR_PLL_MDIV_VALUE = 0;
  
  APHYCSR->DDR_PLL_GLOBAL_RESET = 1;

  while(!(APHYCSR->DDR_PLL_LOCK_STATUS & (1 << 16)));

  APHYCSR->DDR_PLL_POST_DIV_RESET = 1;

  /* Update clock manager */
  CM->SDCCTL |= CM_KEY | (0x00020000); //CM_SDCTL_UPDATE_SET
  while(!(CM->SDCCTL & 0x00010000)); //CM_SDCTL_ACCEPT_SET
  
  CM->SDCCTL = CM_KEY | (0x4 << 12) | (CM->SDCCTL & 0xFFFF0FFF);

  CM->SDCCTL = CM_KEY | (CM->SDCCTL & 0xFFFDFFFF); //CM_SDCTL_UPDATE_CLR
  while(CM->SDCCTL & 0x00010000); //CM_SDCTL_ACCEPT_SET

  /* Refresh rate = 3113 * (1.0/400) = 7.78us */

  SDRAMCTL->SA = (3113 << 16) | 0x00000100 | 0x00000080 | 0x00000001
	  | 0x3214;
  SDRAMCTL->SB = 0x00000080 | (3 << 5) | 0x00000010 | (3 << 2) |
	  (3);
  SDRAMCTL->SC = (50 << 24) | (2 << 20) | (7 << 8) | 
	  (4 << 4) | (3);
  SDRAMCTL->SD = (7 << 28) | (24 << 20) | (1 << 16) |
	  (15 << 8) | (6 << 4) | (6);
  SDRAMCTL->SE = (1 << 28) | (4 << 20) | (18 << 12) | (1 << 8)
	  | (54);
  SDRAMCTL->PT1 = (79800 << 8) | (40);
  SDRAMCTL->PT2 = 3990;

  SDRAMCTL->MRT = 0x3;

  /* Reset PHY DLL */
  APHYCSR->PHY_BIST_CNTRL_SPR = 0x30;

  DPHYCSR->GLBL_DQ_DLL_RESET = 0x1;
  APHYCSR->GLBL_ADDR_DLL_RESET = 0x1;

  /* Stall */
  SDRAMCTL->CS;
  SDRAMCTL->CS;
  SDRAMCTL->CS;
  SDRAMCTL->CS;

  DPHYCSR->GLBL_DQ_DLL_RESET = 0x0;
  APHYCSR->GLBL_ADDR_DLL_RESET = 0x0;
  while((DPHYCSR->GLBL_MSTR_DLL_LOCK_STAT & 0xFFFF) != 0xFFFF);

  while(!(APHYCSR->GLBL_ADR_DLL_LOCK_STAT == 3));

  APHYCSR->PHY_BIST_CNTRL_SPR = 0x0;
  SDRAMCTL->CS = (((4 << 19) | 0x00000040 | 0x00000002) & 
		  ~(0x00000080 | 0x00000008)) | 0x00000001;
  return;
}
void main(void)
{
  uint32_t *ptr = (uint32_t *) 0xC0000000;
  uart_init();
  sdram_init();
  char c = 't';
  *ptr = c;

  AUX->AUX_MU_IO_REG = *ptr;
  while(!(AUX->AUX_MU_LSR_REG & (1 << 6)));

  ptr = (uint32_t *) 0xFFFFFFFC;
  *ptr = c;
  AUX->AUX_MU_IO_REG = *ptr;
  while(!(AUX->AUX_MU_LSR_REG & (1 << 6)));

  while(1);
}
