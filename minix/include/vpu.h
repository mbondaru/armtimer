#ifndef RPI_H_
#define RPI_H_

/* Use standard integer types with explicit width */
#include "stdint.h"

/* IO definitions */
/* (Access restrictions to peripheral registers) */
/* IO Type Qualifiers are used to specify access to */
/* peripheral variables for automatic generation of */
/* peripheral register debug information */
/* (CMSIS Global Defines) */

#define __I volatile
#define __O volatile
#define __IO volatile

/* CMSIS-compatible Interrupt Number Definition */
typedef enum IRQn
{
   /* Cortex-M4 Processor Exceptions Numbers */
   Zero = -31,
   MisalignedMemAccess = -30,
   DivByZero = -29,
   UndefInstr = -28,
   ForbiddenInstr = -27,
   IllegalMemoy = -26,
   BusError = -25,
   FloatingPoint = -24,
   ISP_IRQn = -23,
   Dummy = -22,
   ICache = -21,
   VECCore = -20,
   BadL2Alias = -19,
   Breakpoint = -18,
   Unk0 = -17,
   Unk1 = -16,
   Unk2 = -15,
   Unk3 = -14,
   Unk4 = -13,
   Unk5 = -12,
   Unk6 = -11,
   Unk7 = -10,
   Unk8 = -9,
   Unk9 = -8,
   Unk10 = -7,
   Unk11 = -6,
   Unk12 = -5,
   Unk13 = -4,
   Unk14 = -3,
   Unk15 = -2,
   Unk16 = -1,
   Unk17 = 0,
   Swi0 = 1,
   Swi1 = 2,
   Swi2 = 3,
   Swi3 = 4,
   Swi4 = 5,
   Swi5 = 6,
   Swi6 = 7,
   Swi7 = 8,
   Swi8 = 9,
   Swi9 = 10,
   Swi10 = 11,
   Swi11 = 12,
   Swi12 = 13,
   Swi13 = 14,
   Swi14 = 15,
   Swi15 = 16,
   Swi16 = 17,
   Swi17 = 18,
   Swi18 = 19,
   Swi19 = 20,
   Swi20 = 21,
   Swi21 = 22,
   Swi22 = 23,
   Swi23 = 24,
   Swi24 = 25,
   Swi25 = 26,
   Swi26 = 27,
   Swi27 = 28,
   Swi28 = 29,
   Swi29 = 30,
   Swi30 = 31,
   Swi31 = 32,
   Timer0_IRQn = 33,
   Timer1_IRQn = 34,
   Timer2_IRQn = 35,
   Timer3_IRQn = 36,
   Codec0_IRQn = 37,
   Codec1_IRQn = 38,
   Codec2_IRQn = 39,
   JPEG_IRQn = 40,
   ISP_IRQn = 41,
   USB_IRQn = 42,
   //TODO

   /*Peripheral Exceptions Numbers */
   CM_IRQn = 0,
   DMA_ERR_IRQn = 1,
   DMA_INT0_IRQn = 2,
   PORT1_IRQn = 3
} IRQn_Type;

/* Definition of standard bits */
#define BIT0 (uint16_t) (0x0001)
#define BIT1 (uint16_t) (0x0002)
#define BIT2 (uint16_t) (0x0004)
#define BIT3 (uint16_t) (0x0008)
#define BIT4 (uint16_t) (0x0010)
#define BIT5 (uint16_t) (0x0020)
#define BIT6 (uint16_t) (0x0040)
#define BIT7 (uint16_t) (0x0080)
#define BIT(x) ((uint16_t)1 << (x))

/* Thread Control Base */
#define TH_BASE ((uint32_t) 0x18e00000)
/* Device and peripheral memory map */
#define FLASH_BASE ((uint32_t) 0x00000000)
/* ARM peripherals */
#define PERIPH_BASE ((uint32_t) 0x7E000000)
/* ARM Core0,1,2,3 timers and mailboxes */
#define LOCAL_PERIPH_BASE ((uint32_t) 0x40000000)
/* Exception Vector Table */
#define VT_BASE ((uint32_t) 0x00000000)

/* Auxiliaries: UART1 & SPI1, SPI2 */
#define AUX_BASE ((uint32_t) PERIPH_BASE + 0x00215000)
/* SDRAM controller */
#define SD_BASE ((uint32_t) PERIPH_BASE + 0x7ee00000)
/* Broadcom Serial Controller I2C single master only operation */
/* BSC2 Master is dedicated to the HDMI interface and should not */
/* be accessed by other programs */
#define BSC0_MASTER_BASE ((uint32_t) PERIPH_BASE + 0x00205000)
#define BSC1_MASTER_BASE ((uint32_t) PERIPH_BASE + 0x00804000)
#define BSC2_MASTER_BASE ((uint32_t) PERIPH_BASE + 0x00805000)

/* DMA Controller comprised of 16 DMA Channels */
/* Adjacent DMA Channels are offset by 0x100 */
/* DMA Channel 15 is physically removed from other DMA Channels */
#define DMA0_DMA14_BASE ((uint32_t) PERIPH_BASE + 0x00007000)
#define DMA15_BASE ((uint32_t) PERIPH_BASE + 0x00E05000)

/* External Mass Meda Controller */
/* Embedded Multimedia(TM) and SD(TM) card interface by Arasan(TM) */
#define EMMC_BASE ((uint32_t) PERIPH_BASE + 0x00300000)

/* GPIO (General Purpose I/O) supports 54 GPIO lines */
#define GPIO_BASE ((uint32_t) PERIPH_BASE + 0x00200000)

/* Base address for the ARM interrupt register */
#define IRQ_BASE ((uint32_t) PERIPH_BASE + 0x0000B200)
/* Base address for the VPU interrupt controller 0 */
#define IC0_BASE ((uint32_t) PERIPH_BASE + 0x00002000)
/* Base address for the VPU interrupt controller 1 */
#define IC1_BASE ((uint32_t) PERIPH_BASE + 0x00002800)

/* PCM Audio Interface */
#define PCM_BASE ((uint32_t) PERIPH_BASE + 0x00203000)

/* Pulse Width Modulator (PWM) */
/*#define PWM_BASE ((uint32_t) PERIPH_BASE + 0x0020C000)*/
/* Address in memory not specified in BCM2835 reference manual */

/* Auxiliaries: UART1 & SPI1, SPI2 */
#define AUX_BASE ((uint32_t) PERIPH_BASE + 0x00215000)
/* SPI (Serial Peripheral Interface) */
#define SPI_BASE ((uint32_t) PERIPH_BASE + 0x00204000)

/* BSC/SPI slave */
#define BSC_SPI_SLAVE_BASE ((uint32_t) PERIPH_BASE + 0x00214000)

/* System Timer */
#define SYSTIMER_BASE ((uint32_t) PERIPH_BASE + 0x00003000)

/* PL011 UART */
/* #define PL011_USRT_BASE ((uint32_t) PERIPH_BASE + 0x00201000) */

/* Timer (ARM side) */
#define ARM_TIMER_BASE ((uint32_t) PERIPH_BASE + 0x0000B000)
/* SDC Addr Front */
#define APHY_CSR_BASE ((uint32_t) PERIPH_BASE + 0x00E06000)
/* SDC DQ Front */
#define DPHY_CSR_BASE ((uint32_t) PERIPH_BASE + 0x00E07000)
/* PLL controller */
#define A2W_BASE ((uint32_t) PERIPH_BASE + 0x00102000)

/* USB */
#define USB_BASE ((uint32_t) PERIPH_BASE + 0x00980080)

/* GPIO Clocks Clock Manager */
#define CM_BASE ((uint32_t) PERIPH_BASE + 0x00101000)
/* Multicore Sync*/
#define MS_BASE ((uint32_t) PERIPH_BASE)
/* SD Host */
#define SH_BASE ((uint32_t) PERIPH_BASE + 0x00202000)

/* GPIO Pads drive control */
//#define GPIO_PADS_CTL_BASE ((uint32_t) PERIPH_BASE + 0x00100000)
// Moved to PWRMAN_BASE

#define SMI_BASE ((uint32_t) PERIPH_BASE + 0x00600000)
#define L1CCTL ((uint32_t) PERIPH_BASE + 0x00E02000)
#define L2CCTL ((uint32_t) PERIPH_BASE + 0x00E01000)
#define VPU_ARB ((uint32_t) PERIPH_BASE + 0x00E04000)
#define VPU_ACR ((uint32_t) PERIPH_BASE + 0x0080A000)
#define VPU_ASB ((uint32_t) PERIPH_BASE + 0x0000A000)
#define VC4_BASE ((uint32_t) PERIPH_BASE + 0x00C00000)
#define PIXVALVE0_BASE ((uint32_t) PERIPH_BASE + 0x00206000)
#define PIXVALVE1_BASE ((uint32_t) PERIPH_BASE + 0x00207000)
#define PIXVALVE2_BASE ((uint32_t) PERIPH_BASE + 0x00807000)
/* VEC encoder */
#define VEC_BASE ((uint32_t) PERIPH_BASE + 0x00806000)
#define HDMI_BASE ((uint32_t) PERIPH_BASE + 0x00902000)
/* Hardware Video Scaler */
#define HVS_BASE ((uint32_t) PERIPH_BASE + 0x00400000)
#define HVS_DLIST_START ((uint32_t) PERIPH_BASE + 0x00402000)
#define HVS_DLIST_SIZE ((uint32_t) PERIPH_BASE + 0x00404000)
/* Display Parallel Interface */
#define DPI_BASE ((uint32_t) PERIPH_BASE + 0x00208000)
/* Display Serial Interface */
#define DSI_BASE ((uint32_t) PERIPH_BASE + 0x00209000)
/* Peripheral register definitions */
/* Device specific Peripheral register structures */
#define ISP_BASE ((uint32_t) PERIPH_BASE + 0x00C75000)
#define PWRMAN_BASE ((uint32_t) PERIPH_BASE + 0x00100000)
#define ARMCTL_BASE ((uint32_t) PERIPH_BASE + 0x0000B000)
/* ARM JTAG BASH */
#define AJB_BASE ((uint32_t) PERIPH_BASE + 0x002000C0)
typedef struct {
  __IO uint32_t PLLA_DIG0;
  __IO uint32_t PLLA_DIG1;
  __IO uint32_t PLLA_DIG2;
  __IO uint32_t PLLA_DIG3;
  __IO uint32_t PLLA_ANA0;
  __IO uint32_t PLLA_ANA1;
  __IO uint32_t PLLA_ANA2;
  __IO uint32_t PLLA_ANA3;
  __IO uint32_t PLLC_DIG0;
  __IO uint32_t PLLC_DIG1;
  __IO uint32_t PLLC_DIG2;
  __IO uint32_t PLLC_DIG3;
  __IO uint32_t PLLC_ANA0;
  __IO uint32_t PLLC_ANA1;
  __IO uint32_t PLLC_ANA2;
  __IO uint32_t PLLC_ANA3;
  __IO uint32_t PLLD_DIG0;
  __IO uint32_t PLLD_DIG1;
  __IO uint32_t PLLD_DIG2;
  __IO uint32_t PLLD_DIG3;
  __IO uint32_t PLLD_ANA0;
  __IO uint32_t PLLD_ANA1;
  __IO uint32_t PLLD_ANA2;
  __IO uint32_t PLLD_ANA3;
  __IO uint32_t PLLH_DIG0;
  __IO uint32_t PLLH_DIG1;
  __IO uint32_t PLLH_DIG2;
  __IO uint32_t PLLH_DIG3;
  __IO uint32_t PLLH_ANA0;
  __IO uint32_t PLLH_ANA1;
  __IO uint32_t PLLH_ANA2;
  __IO uint32_t PLLH_ANA3;
  __IO uint32_t HDMI_CTL0;
  __IO uint32_t HDMI_CTL1;
  __IO uint32_t HDMI_CTL2;
  __IO uint32_t HDMI_CTL3;
  __IO uint32_t XOSC0;
  __IO uint32_t XOSC1;
  __IO uint32_t RESERVED0[2];
  __IO uint32_t SMPS_CTLA0;
  __IO uint32_t SMPS_CTLA1;
  __IO uint32_t SMPS_CTLA2;
  __IO uint32_t RESERVED1;
  __IO uint32_t SMPS_CTLB0;
  __IO uint32_t SMPS_CTLB1;
  __IO uint32_t SMPS_CTLB2;
  __IO uint32_t RESERVED2;
  __IO uint32_t SMPS_CTLC0;
  __IO uint32_t SMPS_CTLC1;
  __IO uint32_t SMPS_CTLC2;
  __IO uint32_t SMPS_CTLC3;
  __IO uint32_t SMPS_LDO0;
  __IO uint32_t SMPS_LDO1;
  __IO uint32_t RESERVED3[2];
  __IO uint32_t PLLB_DIG0;
  __IO uint32_t PLLB_DIG1;
  __IO uint32_t PLLB_DIG2;
  __IO uint32_t PLLB_DIG3;
  __IO uint32_t PLLB_ANA0;
  __IO uint32_t PLLB_ANA1;
  __IO uint32_t PLLB_ANA2;
  __IO uint32_t PLLB_ANA3;
  __IO uint32_t PLLA_CTRL;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t PLLA_ANA_SSCS;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t PLLC_CTRL;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t PLLC_ANA_SSCS;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t PLLD_CTRL;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t PLLD_ANA_SSCS;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t PLLH_CTRL;
  __IO uint32_t RESERVED4[7];
  __IO uint32_t HDMI_CTL_RCAL;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t XOSC_CTRL;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t SMPS_A_MODE; 
  __IO uint32_t RESERVED4[3];
  __IO uint32_t SMPS_B_STAT;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t SMPS_C_CLK;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t SMPS_L_SPV;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t PLLB_CTRL;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t PLLB_ANA_SSCS;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t PLLA_FRAC;
  __IO uint32_t RESERVED5[3]; 
  __IO uint32_t PLLA_ANA_SSCL;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t PLLC_FRAC;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t PLLC_ANA_SSCL;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t PLLD_FRAC;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t PLLD_ANA_SSCL;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t PLLH_FRAC;
  __IO uint32_t RESERVED5[7];
  __IO uint32_t HDMI_CTL_HFEN;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t XOSC_CPR;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t SMPS_A_VOLTS; 
  __IO uint32_t RESERVED5[7];
  __IO uint32_t SMPS_C_CTL;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t SMPS_L_SPA;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t PLLB_FRAC;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t PLLB_ANA_SSCL;
  __IO uint32_t RESERVED5[3];
  __IO uint32_t PLLA_DSI0;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLA_ANA_KAIP;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLC_CORE2; 
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLC_ANA_KAIP;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLD_DSI0;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLD_ANA_KAIP;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLH_AUX;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLH_ANA_KAIP;
  __IO uint32_t RESERVED6[7];
  __IO uint32_t XOSC_BIAS;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t SMPS_A_GAIN; 
  __IO uint32_t RESERVED6[11];
  __IO uint32_t SMPS_L_SCV;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLB_ARM;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLB_ANA_KAIP;
  __IO uint32_t RESERVED6[3];
  __IO uint32_t PLLA_CORE;
  __IO uint32_t RESERVED7[3];
  __IO uint32_t PLLA_ANA_STAT;
  __IO uint32_t RESERVED7[3]; 
  __IO uint32_t PLLC_CORE1;
  __IO uint32_t RESERVED7[3];
  __IO uint32_t PLLC_ANA_STAT;
  __IO uint32_t RESERVED7[3];
  __IO uint32_t PLLD_CORE;
  __IO uint32_t RESERVED7[3];
  __IO uint32_t PLLD_ANA_STAT;
  __IO uint32_t RESERVED7[3];
  __IO uint32_t PLLH_RCAL;
  __IO uint32_t RESERVED7[11];
  __IO uint32_t XOSC_PWR;
  __IO uint32_t RESERVED7[15];
  __IO uint32_t SMPS_L_SCA;
  __IO uint32_t RESERVED7[3];
  __IO uint32_t PLLB_SP0;
  __IO uint32_t RESERVED7[3];
  __IO uint32_t PLLB_ANA_STAT;
  __IO uint32_t RESERVED7[3];
  __IO uint32_t PLLA_PER;
  __IO uint32_t RESERVED8[3];
  __IO uint32_t PLLA_ANA_SCTL;
  __IO uint32_t RESERVED8[3]; 
  __IO uint32_t PLLC_PER;    
  __IO uint32_t RESERVED8[3];
  __IO uint32_t PLLC_ANA_SCTL;
  __IO uint32_t RESERVED8[3];
  __IO uint32_t PLLD_PER;
  __IO uint32_t RESERVED8[3];
  __IO uint32_t PLLD_ANA_SCTL;
  __IO uint32_t RESERVED8[3];
  __IO uint32_t PLLH_PIX;
  __IO uint32_t RESERVED8[3];
  __IO uint32_t PLLH_ANA_SCTL;
  __IO uint32_t RESERVED8[23];
  __IO uint32_t SMPS_L_SIV;
  __IO uint32_t RESERVED8[3];
  __IO uint32_t PLLB_SP1;
  __IO uint32_t RESERVED8[3];
  __IO uint32_t PLLB_ANA_SCTL;
  __IO uint32_t RESERVED8[3];
  __IO uint32_t PLLA_CCP2;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t PLLA_ANA_VCO;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t PLLC_CORE0;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t PLLC_ANA_VCO;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t PLLD_DSI1;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t PLLD_ANA_VCO;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t PLLH_ANA_STAT;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t PLLH_ANA_VCO;
  __IO uint32_t RESERVED9[23];
  __IO uint32_t SMPS_L_SIA;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t PLLB_SP2;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t PLLB_ANA_VCO;
  __IO uint32_t RESERVED9[67];
  __IO uint32_t PLLA_DIG0R;
  __IO uint32_t PLLA_DIG1R;
  __IO uint32_t PLLA_DIG2R;
  __IO uint32_t PLLA_DIG3R;
  __IO uint32_t PLLA_ANA0R;
  __IO uint32_t PLLA_ANA1R;
  __IO uint32_t PLLA_ANA2R;
  __IO uint32_t PLLA_ANA3R;
  __IO uint32_t PLLC_DIG0R;
  __IO uint32_t PLLC_DIG1R;
  __IO uint32_t PLLC_DIG2R;
  __IO uint32_t PLLC_DIG3R;
  __IO uint32_t PLLC_ANA0R;
  __IO uint32_t PLLC_ANA1R;
  __IO uint32_t PLLC_ANA2R;
  __IO uint32_t PLLC_ANA3R;
  __IO uint32_t PLLD_DIG0R;
  __IO uint32_t PLLD_DIG1R;
  __IO uint32_t PLLD_DIG2R;
  __IO uint32_t PLLD_DIG3R;
  __IO uint32_t PLLD_ANA0R;
  __IO uint32_t PLLD_ANA1R;
  __IO uint32_t PLLD_ANA2R;
  __IO uint32_t PLLD_ANA3R;
  __IO uint32_t PLLH_DIG0R;
  __IO uint32_t PLLH_DIG1R;
  __IO uint32_t PLLH_DIG2R;
  __IO uint32_t PLLH_DIG3R;
  __IO uint32_t PLLH_ANA0R;
  __IO uint32_t PLLH_ANA1R;
  __IO uint32_t PLLH_ANA2R;
  __IO uint32_t PLLH_ANA3R;
  __IO uint32_t HDMI_CTL0R;
  __IO uint32_t HDMI_CTL1R;
  __IO uint32_t HDMI_CTL2R;
  __IO uint32_t HDMI_CTL3R;
  __IO uint32_t XOSC0R;
  __IO uint32_t XOSC1R;
  __IO uint32_t RESERVED10[2];
  __IO uint32_t SMPS_CTLA0R;
  __IO uint32_t SMPS_CTLA1R;
  __IO uint32_t SMPS_CTLA2R;
  __IO uint32_t RESERVED11;
  __IO uint32_t SMPS_CTLB0R;
  __IO uint32_t SMPS_CTLB1R;
  __IO uint32_t SMPS_CTLB2R;
  __IO uint32_t RESERVED12;
  __IO uint32_t SMPS_CTLC0R;
  __IO uint32_t SMPS_CTLC1R;
  __IO uint32_t SMPS_CTLC2R;
  __IO uint32_t SMPS_CTLC3R;
  __IO uint32_t SMPS_LDO0R;
  __IO uint32_t SMPS_LDO1R;
  __IO uint32_t RESREVED13[2];
  __IO uint32_t PLLB_DIG0R;
  __IO uint32_t PLLB_DIG1R;
  __IO uint32_t PLLB_DIG2R;
  __IO uint32_t PLLB_DIG3R;
  __IO uint32_t PLLB_ANA0R;
  __IO uint32_t PLLB_ANA1R;
  __IO uint32_t PLLB_ANA2R;
  __IO uint32_t PLLB_ANA3R;
  __IO uint32_t PLLA_CTRLR;   //900
  __IO uint32_t RESERVED14[3];
  __IO uint32_t PLLA_ANA_SSCSR; 
  __IO uint32_t RESERVED14[3];
  __IO uint32_t PLLC_CTRLR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t PLLC_ANA_SSCSR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t PLLD_CTRLR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t PLLD_ANA_SSCSR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t PLLH_CTRLR;
  __IO uint32_t RESERVED14[7];
  __IO uint32_t HDMI_CTL_RCALR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t XOSC_CTRLR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t SMPS_A_MODER;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t SMPS_B_STATR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t SMPS_C_CLKR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t SMPS_L_SPVR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t PLLB_CTRLR;
  __IO uint32_t RESERVED14[3];
  __IO uint32_t PLLB_ANA_SSCSR;
  __IO uint32_t RESERVED14[3];


  __IO uint32_t PLLA_FRACR;  //A00
  __IO uint32_t RESERVED15[3];
  __IO uint32_t PLLA_ANA_SSCLR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t PLLC_FRACR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t PLLC_ANA_SSCLR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t PLLD_FRACR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t PLLD_ANA_SSCLR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t PLLH_FRACR;
  __IO uint32_t RESERVED15[7];
  __IO uint32_t HDMI_CTL_HFENR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t XOSC_CPRR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t SMPS_A_VOLTSR;
  __IO uint32_t RESERVED15[7];
  __IO uint32_t SMPS_C_CTLR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t SMPS_L_SPAR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t PLLB_FRACR;
  __IO uint32_t RESERVED15[3];
  __IO uint32_t PLLB_ANA_SSCLR;
  __IO uint32_t RESERVED15[3];


  __IO uint32_t PLLA_DSI0R;  // B00
  __IO uint32_t RESERVED16[3];
  __IO uint32_t PLLA_ANA_KAIPR;
  __IO uint32_t RESERVED16[3];
  __IO uint32_t PLLC_CORE2R;
  __IO uint32_t RESERVED16[3];
  __IO uint32_t PLLC_ANA_KAIPR;
  __IO uint32_t RESERVED16[3];
  __IO uint32_t PLLD_DSI0R;
  __IO uint32_t RESERVED16[3];
  __IO uint32_t PLLD_ANA_KAIPR;
  __IO uint32_t RESERVED16[3];
  __IO uint32_t PLLH_AUXR;
  __IO uint32_t RESERVED16[3];
  __IO uint32_t PLLH_ANA_KAIPR;
  __IO uint32_t RESERVED16[7];
  __IO uint32_t XOSC_BIASR;
  __IO uint32_t RESERVED16[3];
  __IO uint32_t SMPS_A_GAINR;
  __IO uint32_t RESERVED16[11];
  __IO uint32_t SMPS_L_SCVR;
  __IO uint32_t RESERVED16[3];
  __IO uint32_t PLLB_ARMR;
  __IO uint32_t RESERVED16[3];
  __IO uint32_t PLLB_ANA_KAIPR;
  __IO uint32_t RESERVED16[3];
 

  __IO uint32_t PLLA_CORER;    //C00
  __IO uint32_t RESERVED17[3];
  __IO uint32_t PLLA_ANA_STATR;
  __IO uint32_t RESERVED17[3];
  __IO uint32_t PLLC_CORE1R;
  __IO uint32_t RESERVED17[3];
  __IO uint32_t PLLC_ANA_STATR;
  __IO uint32_t RESERVED17[3];
  __IO uint32_t PLLD_CORER;
  __IO uint32_t RESERVED17[3];
  __IO uint32_t PLLD_ANA_STATR;
  __IO uint32_t RESERVED17[3];
  __IO uint32_t PLLH_RCALR;
  __IO uint32_t RESERVED17[11];
  __IO uint32_t XOSC_PWRR;
  __IO uint32_t RESERVED17[15];
  __IO uint32_t SMPS_L_SCAR;
  __IO uint32_t RESERVED17[3];
  __IO uint32_t PLLB_SP0R;
  __IO uint32_t RESERVED17[3];
  __IO uint32_t PLLB_ANA_STATR;
  __IO uint32_t RESERVED17[3];


  __IO uint32_t PLLA_PERR;      //D00
  __IO uint32_t RESERVED18[3];
  __IO uint32_t PLLA_ANA_SCTLR;
  __IO uint32_t RESERVED18[3];
  __IO uint32_t PLLC_PERR;
  __IO uint32_t RESERVED18[3];
  __IO uint32_t PLLC_ANA_SCTLR;
  __IO uint32_t RESERVED18[3];
  __IO uint32_t PLLD_PERR;
  __IO uint32_t RESERVED18[3];
  __IO uint32_t PLLD_ANA_SCTLR;
  __IO uint32_t RESERVED18[3];
  __IO uint32_t PLLH_PIXR;
  __IO uint32_t RESERVED18[3];
  __IO uint32_t PLLH_ANA_SCTLR;
  __IO uint32_t RESERVED18[23];
  __IO uint32_t SMPS_L_SIVR;
  __IO uint32_t RESERVED18[3];
  __IO uint32_t PLLB_SP1R;
  __IO uint32_t RESERVED18[3];
  __IO uint32_t PLLB_ANA_SCTLR;
  __IO uint32_t RESERVED18[3];


  __IO uint32_t PLLA_CCP2R;     //E00
  __IO uint32_t RESERVED19[3];
  __IO uint32_t PLLA_ANA_VCOR;
  __IO uint32_t RESERVED19[3];
  __IO uint32_t PLLC_CORE0R;
  __IO uint32_t RESERVED19[3];
  __IO uint32_t PLLC_ANA_VCOR;
  __IO uint32_t RESERVED19[3];
  __IO uint32_t PLLD_DSI1R;
  __IO uint32_t RESERVED19[3];
  __IO uint32_t PLLD_ANA_VCOR;
  __IO uint32_t RESERVED19[3];
  __IO uint32_t PLLH_ANA_STATR;
  __IO uint32_t RESERVED19[3];
  __IO uint32_t PLLH_ANA_VCOR;
  __IO uint32_t RESERVED19[23];
  __IO uint32_t SMPS_L_SIAR;
  __IO uint32_t RESERVED19[3];
  __IO uint32_t PLLB_SP2R;
  __IO uint32_t RESERVED19[3];
  __IO uint32_t PLLB_ANA_VCOR;
  __IO uint32_t RESERVED19[3];


  __IO uint32_t PLLA_MULTI;    //F00
  __IO uint32_t RESERVED20[3];
  __IO uint32_t PLLA_ANA_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t PLLC_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t PLLC_ANA_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t PLLD_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t PLLD_ANA_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t PLLH_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t PLLH_ANA_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t HDMI_CTL_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t XOSC_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t SMPS_A_MULTI; 
  __IO uint32_t RESERVED20[3];
  __IO uint32_t SMPS_B_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t SMPS_C_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t SMPS_L_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t PLLB_MULTI;
  __IO uint32_t RESERVED20[3];
  __IO uint32_t PLLB_ANA_MULTI;
} A2W_Type;

typedef struct {
  __IO uint32_t ADDR_REV_ID;
  __IO uint32_t ADDR_DLL_RESET;
  __IO uint32_t ADDR_DLL_RECAL;
  __IO uint32_t ADDR_DLL_CNTRL;
  __IO uint32_t ADDR_DLL_PH_LD_VAL;
  __IO uint32_t MASTER_DLL_OUTPUT;
  __IO uint32_t SLAVE_DLL_OFFSET;
  __IO uint32_t GLBL_ADR_MSTR_DLL_BYPEN;
  __IO uint32_t GLBL_ADR_DLL_LOCK_STAT;
  __IO uint32_t DDR_PLL_GLOBAL_RESET;
  __IO uint32_t DDR_PLL_POST_DIV_RESET;
  __IO uint32_t DDR_PLL_VCO_FREQ_CNTRL0;
  __IO uint32_t DDR_PLL_VCO_FREQ_CNTRL1;
  __IO uint32_t DDR_PLL_MDIV_VALUE;
  __IO uint32_t DDR_PLL_CONFIG_CNTRL;
  __IO uint32_t DDR_PLL_MISC_CNTRL;
  __IO uint32_t DDR_PLL_SPRDSPECT_CTRL0;
  __IO uint32_t DDR_PLL_SPRDSPECT_CTRL1;
  __IO uint32_t DDR_PLL_LOCK_STATUS;
  __IO uint32_t DDR_PLL_HOLD_CH;
  __IO uint32_t DDR_PLL_ENABLE_CH;
  __IO uint32_t DDR_PLL_BYPASS;
  __IO uint32_t DDR_PLL_PWRDWN;
  __IO uint32_t DDR_PLL_CH0_DESKEW_CTRL;
  __IO uint32_t DDR_PLL_CH1_DESKEW_CTRL; 
  __IO uint32_t DDR_PLL_CSR_DESKEW_STATUS;
  __IO uint32_t ADDR_PAD_DRV_SLEW_CTRL;
  __IO uint32_t ADDR_PAD_MISC_CTRL;
  __IO uint32_t ADDR_PVT_COMP_CTRL;
  __IO uint32_t ADDR_PVT_COMP_OVRD_CTRL;
  __IO uint32_t ADDR_PVT_COMP_STATUS;
  __IO uint32_t ADDR_PVT_COMP_DEBUG;
  __IO uint32_t PHY_BIST_CNTRL_SPR;
  __IO uint32_t PHY_BIST_CA_CRC_SPR;
  __IO uint32_t ADDR_SPR0_RW;
  __IO uint32_t ADDR_SPR1_RO;
  __IO uint32_t ADDR_SPR_RO;
} SDRAM_APHY_Type;

typedef struct {
  __IO uint32_t GLBL_DQ_REV_ID;
  __IO uint32_t GLBL_DQ_DLL_RESET;
  __IO uint32_t GLBL_DQ_DLL_RECALIBRATE;
  __IO uint32_t GLBL_DQ_DLL_CNTRL;
  __IO uint32_t GLBL_DQ_DLL_PHASE_LD_VL;
  __IO uint32_t GLBL_DQ_MSTR_DLL_BYP_EN;
  __IO uint32_t BYTE0_SLAVE_DLL_OFFSET;
  __IO uint32_t BYTE1_SLAVE_DLL_OFFSET;
  __IO uint32_t BYTE2_SLAVE_DLL_OFFSET;
  __IO uint32_t BYTE3_SLAVE_DLL_OFFSET;
  __IO uint32_t BYTE0_MASTER_DLL_OUTPUT;
  __IO uint32_t BYTE1_MASTER_DLL_OUTPUT;
  __IO uint32_t BYTE2_MASTER_DLL_OUTPUT;
  __IO uint32_t BYTE3_MASTER_DLL_OUTPUT;
  __IO uint32_t NORM_READ_DQS_GATE_CTRL;
  __IO uint32_t BOOT_READ_DQS_GATE_CTRL;
  __IO uint32_t PHY_FIFO_PNTRS;
  __IO uint32_t DQ_PHY_MISC_CTRL;
  __IO uint32_t DQ_PAD_DRV_SLEW_CTRL;
  __IO uint32_t DQ_PAD_MISC_CTRL;
  __IO uint32_t DQ_PVT_COMP_CTRL;
  __IO uint32_t DQ_PVT_COMP_OVERRD_CTRL;
  __IO uint32_t DQ_PVT_COMP_STATUS;
  __IO uint32_t DQ_PVT_COMP_DEBUG;
  __IO uint32_t DQ_PHY_READ_CTRL;
  __IO uint32_t DQ_PHY_READ_STATUS;
  __IO uint32_t DQ_SPR_RW;
  __IO uint32_t DQ_SPR1_RO;
  __IO uint32_t RESERVED0[482];
  __IO uint32_t CRC_CTRL;
  __IO uint32_t CRC_DATA;

} SDRAM_DPHY_Type;
typedef struct {
  __IO uint32_t SEMA_0;
  __IO uint32_t SEMA_1;
  __IO uint32_t SEMA_2;
  __IO uint32_t SEMA_3;
  __IO uint32_t SEMA_4;
  __IO uint32_t SEMA_5;
  __IO uint32_t SEMA_6;
  __IO uint32_t SEMA_7;
  __IO uint32_t SEMA_8;
  __IO uint32_t SEMA_9;
  __IO uint32_t SEMA_10;
  __IO uint32_t SEMA_11;
  __IO uint32_t SEMA_12;
  __IO uint32_t SEMA_13;
  __IO uint32_t SEMA_14;
  __IO uint32_t SEMA_15;
  __IO uint32_t SEMA_16;
  __IO uint32_t SEMA_17;
  __IO uint32_t SEMA_18;
  __IO uint32_t SEMA_19;
  __IO uint32_t SEMA_20;
  __IO uint32_t SEMA_21;
  __IO uint32_t SEMA_22;
  __IO uint32_t SEMA_23;
  __IO uint32_t SEMA_24;
  __IO uint32_t SEMA_25;
  __IO uint32_t SEMA_26;
  __IO uint32_t SEMA_27;
  __IO uint32_t SEMA_28;
  __IO uint32_t SEMA_29;
  __IO uint32_t SEMA_30;
  __IO uint32_t SEMA_31;
  __IO uint32_t STATUS;
  __IO uint32_t IREQ_0;
  __IO uint32_t IREQ_1;
  __IO uint32_t RESERVED0;
  __IO uint32_t ICSET_0;
  __IO uint32_t ICSET_1;
  __IO uint32_t ICCLR_0;
  __IO uint32_t ICCLR_1;
  __IO uint32_t MBOX_0;
  __IO uint32_t MBOX_1;
  __IO uint32_t MBOX_2;
  __IO uint32_t MBOX_3;
  __IO uint32_t MBOX_4;
  __IO uint32_t MBOX_5;
  __IO uint32_t MBOX_6;
  __IO uint32_t MBOX_7;
  __IO uint32_t VPUSEMA_0;
  __IO uint32_t VPUSEMA_1;
  __IO uint32_t VPU_STAT;  
} Multicore_Sync_Type;

typedef struct {
  __IO uint32_t CS;
  __IO uint32_t SA;
  __IO uint32_t SB;
  __IO uint32_t SC;
  __IO uint32_t PT2;
  __IO uint32_t PT1;
  __IO uint32_t IDL;
  __IO uint32_t RTC;
  __IO uint32_t WTC;
  __IO uint32_t RDC;
  __IO uint32_t WDC;
  __IO uint32_t RAC;
  __IO uint32_t CYC;
  __IO uint32_t CMD;
  __IO uint32_t DAT;
  __IO uint32_t SECSRT0;
  __IO uint32_t SECEND0;
  __IO uint32_t SECSRT1;
  __IO uint32_t SECEND1;
  __IO uint32_t SECSRT2;
  __IO uint32_t SECEND2;
  __IO uint32_t SECSRT3;
  __IO uint32_t SECEND3;
  __IO uint32_t RESERVED0;
  __IO uint32_t PHYC;
  __IO uint32_t MRT; 
  __IO uint32_t RESERVED1[5];
  __IO uint32_t TMC;
  __IO uint32_t RWC;
  __IO uint32_t VAD;
  __IO uint32_t VIN;
  __IO uint32_t RESERVED2;
  __IO uint32_t MR;
  __IO uint32_t SD;
  __IO uint32_t SE;
  __IO uint32_t VER;
  __IO uint32_t STALL; //Only for engineering and debug
  __IO uint32_t RESERVED3[4];
  __IO uint32_t SF;
  __IO uint32_t RESERVED4[18];
  __IO uint32_t CARCRC;
  __IO uint32_t DMRCRC0;
  __IO uint32_t DMRCRC1;
  __IO uint32_t DQRCRC0;
  __IO uint32_t DQRCRC1;
  __IO uint32_t DQRCRC2;
  __IO uint32_t DQRCRC3;
  __IO uint32_t DQRCRC4;
  __IO uint32_t DQRCRC5;
  __IO uint32_t DQRCRC6;
  __IO uint32_t DQRCRC7;
  __IO uint32_t DQRCRC8; //Only for 64 bit wide SDRAM
  __IO uint32_t DQRCRC9; //Only for 64 bit wide SDRAM
  __IO uint32_t DQRCRC10; //Only for 64 bit wide SDRAM
  __IO uint32_t DQRCRC11; //Only for 64 bit wide SDRAM
  __IO uint32_t DQRCRC12; //Only for 64 bit wide SDRAM
  __IO uint32_t DQRCRC13; //Only for 64 bit wide SDRAM
  __IO uint32_t DQRCRC14; //Only for 64 bit wide SDRAM
  __IO uint32_t DQRCRC15; //Only for 64 bit wide SDRAM
  __IO uint32_t DQLCRC0;
  __IO uint32_t DQLCRC1;
  __IO uint32_t DQLCRC2;
  __IO uint32_t DQLCRC3;
  __IO uint32_t DQLCRC4;
  __IO uint32_t DQLCRC5;
  __IO uint32_t DQLCRC6;
  __IO uint32_t DQLCRC7;
  __IO uint32_t DQLCRC8; //Only for 64 bit wide SDRAM
  __IO uint32_t DQLCRC9; //Only for 64 bit wide SDRAM
  __IO uint32_t DQLCRC10; //Only for 64 bit wide SDRAM
  __IO uint32_t DQLCRC11; //Only for 64 bit wide SDRAM
  __IO uint32_t DQLCRC12; //Only for 64 bit wide SDRAM
  __IO uint32_t DQLCRC13; //Only for 64 bit wide SDRAM
  __IO uint32_t DQLCRC14; //Only for 64 bit wide SDRAM
  __IO uint32_t DQLCRC15; //Only for 64 bit wide SDRAM
} SDRAM_Controller_Type;

typedef struct {
  __IO uint32_t L2_CONT_OFF;
  __IO uint32_t L2_FLUSH_STA;
  __IO uint32_t L2_FLUSH_END;
  __IO uint32_t RESERVED0[29];
  __IO uint32_t L2_ALIAS_EXCEPTION;
  __IO uint32_t L2_ALIAS_EXCEPTION_ID;
  __IO uint32_t L2_ALIAS_EXCEPTION_ADDR;
  __IO uint32_t RESERVED1[29];
  __IO uint32_t RD_HITS;
  __IO uint32_t RD_MISSES;
  __IO uint32_t WR_HITS;
  __IO uint32_t WR_MISSES;
  __IO uint32_t WR_BACKS;
  __IO uint32_t IN_FLIGHT;
  __IO uint32_t RESERVED2;
  __IO uint32_t STALLS;
  __IO uint32_t TAG_STALLS;
  __IO uint32_t SD_STALLS;
} L2_Cache_Controller_Type;
typedef struct {
  __IO uint32_t AXI_BRDG_VERSION;
  __IO uint32_t CPR_CTRL;
  __IO uint32_t V3D_S_CTRL;
  __IO uint32_t V3D_M_CTRL;
  __IO uint32_t ISP_S_CTRL;
  __IO uint32_t ISP_M_CTRL;
  __IO uint32_t H264_S_CTRL;
  __IO uint32_t H264_M_CTRL;
} VPU_ASB_Type;
typedef struct {
  __IO uint32_t CONTROL;
} VPU_ACR_Type;
typedef struct {
  __IO uint32_t UC;
  __IO uint32_t L2;
} VPU_ARB_Type;
typedef struct {
  __IO uint32_t L1_IC0_CONTROL;
  __IO uint32_t L1_IC0_PRIORITY;
  __IO uint32_t L1_IC0_FLUSH_S;
  __IO uint32_t L1_IC0_FLUSH_E;
  __IO uint32_t RESERVED0[12];
  __IO uint32_t L1_IC0_RD_HITS;
  __IO uint32_t L1_IC0_RD_MISSES;
  __IO uint32_t L1_IC0_BP_HITS;
  __IO uint32_t L1_IC0_BP_MISSES;
  __IO uint32_t L1_IC0_RAS_PUSHES;
  __IO uint32_t L1_IC0_RAS_POPS;
  __IO uint32_t L1_IC0_RAS_UNDERFLOW;
  __IO uint32_t RESERVED1[9];
  __IO uint32_t L1_IC1_CONTROL;
  __IO uint32_t L1_IC1_PRIORITY;
  __IO uint32_t L1_IC1_FLUSH_S;
  __IO uint32_t L1_IC1_FLUSH_E;
  __IO uint32_t RESERVED2[12];
  __IO uint32_t L1_IC1_RD_HITS;
  __IO uint32_t L1_IC1_RD_MISSES;
  __IO uint32_t L1_IC1_BP_HITS;
  __IO uint32_t L1_IC1_BP_MISSES;
  __IO uint32_t L1_IC1_RAS_PUSHES;
  __IO uint32_t L1_IC1_RAS_POPS;
  __IO uint32_t L1_IC1_RAS_UNDERFLOW;
  __IO uint32_t RESERVED3[9];
  __IO uint32_t L1_D_CONTROL;
  __IO uint32_t L1_D_FLUSH_S;
  __IO uint32_t L1_D_FLUSH_E;
  __IO uint32_t L1_D_PRIORITY;
  __IO uint32_t RESERVED4[12];
  __IO uint32_t L1_D0_RD_HITS;
  __IO uint32_t L1_D0_RD_SNOOPS;
  __IO uint32_t L1_D0_RD_MISSES;
  __IO uint32_t L1_D0_RD_THRUS;
  __IO uint32_t L1_D0_WR_HITS;
  __IO uint32_t L1_D0_WR_SNOOPS;
  __IO uint32_t L1_D0_WR_MISSES;
  __IO uint32_t L1_D0_WR_THRUS;
  __IO uint32_t L1_D0_WBACKS;
  __IO uint32_t RESERVED5[7];
  __IO uint32_t L1_D1_RD_HITS;
  __IO uint32_t L1_D1_SNOOPS;
  __IO uint32_t L1_D1_RD_MISSES;
  __IO uint32_t L1_D1_RD_THRUS;
  __IO uint32_t L1_D1_WR_HITS;
  __IO uint32_t L1_D1_WR_SNOOPS;
  __IO uint32_t L1_D1_WR_MISSES;
  __IO uint32_t L1_D1_WR_THRUS;
  __IO uint32_t L1_D1_WBACKS;
  __IO uint32_t RESERVED6[407];
  __IO uint32_t L1_L1_SANDBOX_START0;
  __IO uint32_t L1_L1_SANDBOX_END0;
  __IO uint32_t L1_L1_SANDBOX_START1;
  __IO uint32_t L1_L1_SANDBOX_END1;
  __IO uint32_t L1_L1_SANDBOX_START2;
  __IO uint32_t L1_L1_SANDBOX_END2;
  __IO uint32_t L1_L1_SANDBOX_START3;
  __IO uint32_t L1_L1_SANDBOX_END3;
  __IO uint32_t L1_L1_SANDBOX_START4;
  __IO uint32_t L1_L1_SANDBOX_END4;
  __IO uint32_t L1_L1_SANDBOX_START5;
  __IO uint32_t L1_L1_SANDBOX_END5;
  __IO uint32_t L1_L1_SANDBOX_START6;
  __IO uint32_t L1_L1_SANDBOX_END6;
  __IO uint32_t L1_L1_SANDBOX_PERI_BR;
} L1_Cache_Controller_Type;

typedef struct {
  __IO uint32_t XCS;
  __IO uint32_t XCFG;
  __IO uint32_t XSTPC;
  __IO uint32_t XITPC;
  __IO uint32_t XTOPC;
  __IO uint32_t XTOUD;
  __IO uint32_t XT1PC;
  __IO uint32_t XT1UD;
  __IO uint32_t XT2PC;
  __IO uint32_t XT2UD;
  __IO uint32_t XT3PC;
  __IO uint32_t XT3UD;
} Thread_Controller_Type;

typedef struct {
  __IO uint32_t C;
  __IO uint32_t S;
  __IO uint32_t SRC0;
  __IO uint32_t SRC1;
  __IO uint32_t MASK0;
  __IO uint32_t MASK1;
  __IO uint32_t MASK2;
  __IO uint32_t MASK3;
  __IO uint32_t MASK4;
  __IO uint32_t MASK5;
  __IO uint32_t MASK6;
  __IO uint32_t MASK7;
  __IO uint32_t VADDR;
  __IO uint32_t WAKEUP;
  __IO uint32_t PROFILE;
  __IO uint32_t FORCE0;
  __IO uint32_t FORCE1;
  __IO uint32_t FORCE0_SET;
  __IO uint32_t FORCE1_SET;
  __IO uint32_t FORCE0_CLR;
  __IO uint32_t FORCE1_CLR;
} IC_Type;

typedef struct {
   __IO uint32_t AJBCONF;
   __IO uint32_t AJBTMS;
   __IO uint32_t AJBTDI;
   __IO uint32_t AJBTDO;
} ARM_JTAG_BASH_Type;

typedef struct {
   __IO uint32_t CTRL;
   __IO uint32_t STATUS;
   __IO uint32_t ID;
   __IO uint32_t SYSTEM;
   __IO uint32_t TILECTRL;
   __IO uint32_t STATSADDR;
   __IO uint32_t TILESTATUS;
   __IO uint32_t TILEADDR;
   __IO uint32_t DESCADDR;
   __IO uint32_t DESCCTRL;
   __IO uint32_t DESCSTATUS;
   __IO uint32_t RESERVED0;
   __IO uint32_t FR_SIZE;
   __IO uint32_t FR_CTRL;
   __IO uint32_t FR_BAYEREN;
   __IO uint32_t FR_YCBCREN;
   __IO uint32_t FR_OFF0;
   __IO uint32_t FR_OFF1;
   __IO uint32_t FR_SWOFF;
   __IO uint32_t FR_MOSAIC;
   __IO uint32_t RESERVED1[3];
   __IO uint32_t FR_FIF00;
   __IO uint32_t FR_FIF01;
   __IO uint32_t RESERVED2[10];
   __IO uint32_t WG_RED;
   __IO uint32_t WG_BLUE;
   __IO uint32_t WG_OFFSETG;
   __IO uint32_t WG_GAIN;
   __IO uint32_t WG_THRESH;
   __IO uint32_t WG_OFFSETR;
   __IO uint32_t WG_OFFSETB;
   __IO uint32_t RESERVED3;
   __IO uint32_t YG_MATRIX;
   __IO uint32_t RESERVED4[4];
   __IO uint32_t YG_OFFSET;
   __IO uint32_t RESERVED5[2];
   __IO uint32_t YG_Y;
   __IO uint32_t YG_SCALE;
   __IO uint32_t RESERVED6[10];
   __IO uint32_t II_CTRL;
   __IO uint32_t II_ADDR;
   __IO uint32_t II_ENDADDR;
   __IO uint32_t II_DPCM;
   __IO uint32_t II_FIFO;
   __IO uint32_t RESERVED7[7];
   __IO uint32_t DI_ADDR;
   __IO uint32_t DI_ENDADDR;
   __IO uint32_t RESERVED8[10];
   __IO uint32_t BL_ABSC_R;
   __IO uint32_t RESERVED9[7];
   __IO uint32_t BL_ABSC_GR;
   __IO uint32_t RESERVED10[7];
   __IO uint32_t BL_ABSC_B;
   __IO uint32_t RESERVED11[7];
   __IO uint32_t BL_ORD_SLOPE_R;
   __IO uint32_t RESERVED12[15];
   __IO uint32_t BL_ORD_SLOPE_GR;
   __IO uint32_t RESERVED13[15];
   __IO uint32_t BL_ORD_SLOPE_B;
   __IO uint32_t RESERVED14[15];
   __IO uint32_t BL_TB;
   __IO uint32_t BL_LR;
   __IO uint32_t BL_MT;
   __IO uint32_t BL_SHIFT;
   union {
      __IO uint64_t BL_SUM;
      struct {
         __IO uint32_t BL_SUM_LO;
         __IO uint32_t BL_SUM_HI;
      };
   };
   __IO uint32_t BL_COUNT;
   __IO uint32_t RESERVED15[13];
   __IO uint32_t DP_HI_OFFSET;
   __IO uint32_t RESERVED16[2];
   __IO uint32_t DP_LO_OFFSET;
   __IO uint32_t RESERVED17[8];
   __IO uint32_t BL_ABSC_GB;
   __IO uint32_t RESERVED18[15];
   __IO uint32_t BL_ORD_SLOPE_GB;


   __IO uint32_t RESERVED19[218];
   __IO uint32_t TD_CTRL;
   __IO uint32_t TD_STATUS;
   __IO uint32_t TD_ADDR;
   __IO uint32_t TD_DESC;
   __IO uint32_t RESERVED20[9];
   __IO uint32_t TD_INIT0;
   __IO uint32_t TD_INIT1;
   __IO uint32_t RESERVED21[7];
   __IO uint32_t RS_CTRL;
   __IO uint32_t RESERVED22[15]; 
   __IO uint32_t LS_CTRL;
   __IO uint32_t LS_OFFSETS;
   __IO uint32_t RESERVED23[10];
   __IO uint32_t XC_ABSC_LIM;
   __IO uint32_t RESERVED24[3];
   __IO uint32_t XC_ORD_LIM;
   __IO uint32_t RESERVED25[3];
   __IO uint32_t XC_SLOPE_LIM;
   __IO uint32_t RESERVED26[23];
   __IO uint32_t DN_CTRL;
   __IO uint32_t DN_ABSC_GD;
   __IO uint32_t RESERVED27[3];
   __IO uint32_t DN_ORD_GD;
   __IO uint32_t RESERVED28[3];
   __IO uint32_t DN_SLOPE_GD;
   __IO uint32_t RESERVED29[3];
   __IO uint32_t DN_GAIN_GD;
   __IO uint32_t DN_SHIFT_GD;
   __IO uint32_t DN_OFFSET;
   __IO uint32_t DN_ABSC_GN;
   __IO uint32_t RESERVED30[3];
   __IO uint32_t DN_ORD_GN;
   __IO uint32_t RESERVED31[3];
   __IO uint32_t DN_SLOPE_GN;
   __IO uint32_t RESERVED32[3];
   __IO uint32_t DN_CBCR_THRESH;
   __IO uint32_t RESERVED33[55];
   __IO uint32_t DM_CTRL;
   __IO uint32_t RESERVED34[31];
   __IO uint32_t YC_MATRIX;
   __IO uint32_t RESERVED35[4];
   __IO uint32_t YC_OFFSET;
   __IO uint32_t RESERVED36[46];
   __IO uint32_t GM_ABSC_R;
   __IO uint32_t RESERVED37[7];
   __IO uint32_t GM_ABSC_G;
   __IO uint32_t RESERVED38[7];
   __IO uint32_t GM_ABSC_B;
   __IO uint32_t RESERVED39[7];
   __IO uint32_t GM_ORD_R;
   __IO uint32_t RESERVED40[7];
   __IO uint32_t GM_ORD_G;
   __IO uint32_t RESERVED41[7];
   __IO uint32_t GM_ORD_B;
   __IO uint32_t RESERVED42[43];
   __IO uint32_t FC_Y_EDGE;
   __IO uint32_t FC_Y_LO_OFFSET;
   __IO uint32_t FC_Y_HI_OFFSET;
   __IO uint32_t FC_THRESH;
   __IO uint32_t RESERVED43[52];
   __IO uint32_t HR_CTRL;
   __IO uint32_t HR_SCALE_X;
   __IO uint32_t HR_SCALE_Y;
   __IO uint32_t HR_NORM;
   __IO uint32_t RESERVED44[52];
   __IO uint32_t LR_TSCALEX;
   __IO uint32_t LR_TSCALEY;
   __IO uint32_t LR_NORM_0_1;
   __IO uint32_t LR_NORM_2_3;
   __IO uint32_t LR_SHIFT;
   __IO uint32_t RESERVED45[3];
   __IO uint32_t CC_MATRIX;
   __IO uint32_t RESERVED46[4];
   __IO uint32_t CC_OFFSET;
   __IO uint32_t RESERVED47[14];
   __IO uint32_t ST_SHIFT;
   __IO uint32_t ST_R_OFF;
   __IO uint32_t RESERVED48[31];
   __IO uint32_t ST_R_RECT;
   __IO uint32_t RESERVED49[31];
   __IO uint32_t ST_HMASK0;
   __IO uint32_t RESERVED50;
   __IO uint32_t ST_FOC_FILT;
   __IO uint32_t RESERVED51[8];
   __IO uint32_t ST_FILT_GAINS;
   __IO uint32_t ST_FILT_TH;
   __IO uint32_t RESERVED52;
   __IO uint32_t ST_ROW_NUM;
   __IO uint32_t ST_R_TH;
   __IO uint32_t RESERVED53[2];
   __IO uint32_t ST_G_TH;
   __IO uint32_t RESERVED54[2];
   __IO uint32_t ST_B_TH;
   __IO uint32_t RESERVED55[2];
   __IO uint32_t ST_R_G_TH;
   __IO uint32_t RESERVED56[2];
   __IO uint32_t ST_B_G_TH;
   __IO uint32_t RESERVED57[2];
   __IO uint32_t ST_GROUP_0_X;
   __IO uint32_t RESERVED58[8];
   __IO uint32_t ST_GROUP_0_Y;
   __IO uint32_t RESERVED59[8];
   __IO uint32_t ST_GRP0_CTRL;
   __IO uint32_t ST_HGAIN0;
   __IO uint32_t ST_HGAIN1;
   __IO uint32_t RESERVED60[20];
   __IO uint32_t LO_CTRL;
   __IO uint32_t LO_COL_STRIDE1;
   __IO uint32_t LO_COL_STRIDE2;
   __IO uint32_t LO_ADDR1;
   __IO uint32_t LO_ADDR2;
   __IO uint32_t LO_ADDR3;
   __IO uint32_t LO_STRIDE1;
   __IO uint32_t LO_STRIDE2;
   __IO uint32_t RESERVED61[136];
   __IO uint32_t LS_CV;
   __IO uint32_t RESERVED62[2303];
   __IO uint32_t GM_SLOPE_R;
   __IO uint32_t RESERVED63[15];
   __IO uint32_t GM_SLOPE_G;
   __IO uint32_t RESERVED64[15];
   __IO uint32_t GM_SLOPE_B;
   __IO uint32_t RESERVED65[31];
   __IO uint32_t TM_Y_ABSC;
   __IO uint32_t RESERVED66[15];
   __IO uint32_t CP_CB_ABSC;
   __IO uint32_t RESERVED67[7];
   __IO uint32_t CP_CR_ABSC;
   __IO uint32_t RESERVED68[7];
   __IO uint32_t TM_Y_ORD_SLOPE;
   __IO uint32_t RESERVED69[15];
   __IO uint32_t CP_CB_ORD_SLOPE;
   __IO uint32_t RESERVED70[7];
   __IO uint32_t CP_CR_ORD_SLOPE;
   __IO uint32_t RESERVED71[391];
   __IO uint64_t DP_CLUSTER;
} ISP_Type;

typedef struct {
   __IO uint32_t C;
   __IO uint32_t ID;
} DPI_Type;

typedef struct {
   __IO uint32_t CORE_REV;
   __IO uint32_t SW_RESET_CONTROL;
   __IO uint32_t HDMI_HOTPLUG_INT;
   __IO uint32_t HDMI_HOTPLUG;
   __IO uint32_t RESERVED0[5];
   __IO uint32_t PACKET_STRIDE;
   __IO uint32_t RESERVED1[13];
   __IO uint32_t FIFO_CTL;
   __IO uint32_t RESERVED2[12];
   __IO uint32_t MAI_CHANNEL_MAP;
   __IO uint32_t MAI_CONFIG;
   __IO uint32_t MAI_FORMAT;
   __IO uint32_t AUDIO_PACKET_CONFIG;
   __IO uint32_t RAM_PACKET_CONFIG;
   __IO uint32_t RAM_PACKET_STATUS;
   __IO uint32_t CRP_CFG;
   __IO uint32_t CTS_0;
   __IO uint32_t CTS_1;
   __IO uint32_t CTS_PERIOD_0;
   __IO uint32_t CTS_PERIOD_1;
   __IO uint32_t RESERVED3;
   __IO uint32_t SCHEDULER_CONTROL;
   __IO uint32_t HORZA;
   __IO uint32_t HORZB;
   __IO uint32_t VERTA0;
   __IO uint32_t VERTB0;
   __IO uint32_t VERTA1;
   __IO uint32_t VERTB1;
   __IO uint32_t RESERVED4[3];
   __IO uint32_t CEC_CNTRL_1;
   __IO uint32_t CEC_CNTRL_2;
   __IO uint32_t CEC_CNTRL_3;
   __IO uint32_t CEC_CNTRL_4;
   __IO uint32_t CEC_CNTRL_5;
   __IO uint32_t CEC_TX_DATA_1;
   __IO uint32_t CEC_TX_DATA_2;
   __IO uint32_t CEC_TX_DATA_3;
   __IO uint32_t CEC_TX_DATA_4;
   __IO uint32_t CEC_RX_DATA_1;
   __IO uint32_t CEC_RX_DATA_2;
   __IO uint32_t CEC_RX_DATA_3;
   __IO uint32_t CEC_RX_DATA_4;
   __IO uint32_t RESERVED5[105];
   __IO uint32_t TX_PHY_RESET_CTL;
   __IO uint32_t TX_PHY_CTL0;
   __IO uint32_t RESERVED6[30];
   __IO uint32_t CPU_STATUS;
   __IO uint32_t CPU_SET;
   __IO uint32_t CPU_CLEAR;
   __IO uint32_t CPU_MASK_STATUS;
   __IO uint32_t CPU_MASK_SET;
   __IO uint32_t CPU_MASK_CLEAR;
} HDMI_Type;


typedef struct {
   __IO uint32_t CONTROL;
   __IO uint32_t V_CONTROL;
   __IO uint32_t VSYNCD_EVEN;
   __IO uint32_t HORZA;
   __IO uint32_t HORZB;
   __IO uint32_t VERTA;
   __IO uint32_t VERTB;
   __IO uint32_t VERTA_EVEN;
   __IO uint32_t VERTB_EVEN;
   __IO uint32_t INTEN;
   __IO uint32_t INTSTAT;
   __IO uint32_t STAT;
   __IO uint32_t HACT_ACT;
} Pixel_Valve_Type;

typedef struct {
   __IO uint32_t DISPCTRL;
   __IO uint32_t DISPSTAT;
   __IO uint32_t DISPID;
   __IO uint32_t DISPECTRL;
   __IO uint32_t DISPPROF;
   __IO uint32_t DISPDITHER;
   __IO uint32_t DISPEOLN;
   __IO uint32_t DISPLIST0;
   __IO uint32_t DISPLIST1;
   __IO uint32_t DISPLIST2;
   __IO uint32_t DISPLSTAT;
   __IO uint32_t DISPLACT0;
   __IO uint32_t DISPLACT1;
   __IO uint32_t DISPLACT2;
   __IO uint32_t DISPCTRL0;
   __IO uint32_t DISPBKGND0;
   __IO uint32_t DISPSTAT0;
   __IO uint32_t DISPBASE0;
   __IO uint32_t DISPCTRL1;
   __IO uint32_t DISPBKGND1;
   __IO uint32_t DISPSTAT1;
   __IO uint32_t DISPBASE1;
   __IO uint32_t DISPCTRL2;
   __IO uint32_t DISPBKGND2;
   __IO uint32_t DISPSTAT2;
   __IO uint32_t DISPBASE2;
   __IO uint32_t DISPALPHA2;
   __IO uint32_t OLEDOFFS;
   __IO uint32_t OLEDCOEF0;
   __IO uint32_t OLEDCOEF1;
   __IO uint32_t OLEDCOEF2;
   __IO uint32_t RESERVED0[12];
   __IO uint32_t DISPSLAVE0_1_2_STUFF[5];
   __IO uint32_t RESERVED[3];
   __IO uint32_t SCALER_GAMDATA;
} HVS_Type;

typedef struct {
   __IO uint32_t IDENT0;
   __IO uint32_t IDENT1;
   __IO uint32_t IDENT2;
   __IO uint32_t RESERVED0;
   __IO uint32_t SCRATCH;
   __IO uint32_t RESERVED1[3];
   __IO uint32_t L2CACTL;
   __IO uint32_t SLCACTL;
   __IO uint32_t RESERVED2[2];
   __IO uint32_t INTCTL;
   __IO uint32_t INTENA;
   __IO uint32_t INTDIS;
   __IO uint32_t RESERVED3[49];
   __IO uint32_t CT0CS;
   __IO uint32_t CT1CS;
   __IO uint32_t CT0EA;
   __IO uint32_t CT1EA;
   __IO uint32_t CT0CA;
   __IO uint32_t CT1CA;
   __IO uint32_t CT00RA0;
   __IO uint32_t CT01RA0;
   __IO uint32_t CT0LC;
   __IO uint32_t CT1LC;
   __IO uint32_t CT0PC;
   __IO uint32_t CT1PC;
   __IO uint32_t PCS;
   __IO uint32_t BFC;
   __IO uint32_t RFC;
   __IO uint32_t RESERVED4[113];
   __IO uint32_t BPCA;
   __IO uint32_t BPCS;
   __IO uint32_t BPOA;
   __IO uint32_t BPOS;
   __IO uint32_t BXCF;
   __IO uint32_t RESERVED5[63];
   __IO uint32_t SQRSV0;
   __IO uint32_t SQRSV1;
   __IO uint32_t SQCNTL;
   __IO uint32_t RESERVED6[5];
   __IO uint32_t SRQPC;
   __IO uint32_t SRQUA;
   __IO uint32_t SRQUL;
   __IO uint32_t SRQCS;
   __IO uint32_t RESERVED7[48];
   __IO uint32_t VPACNTL;
   __IO uint32_t VPMBASE;
   __IO uint32_t RESERVED8[90];
   __IO uint32_t PCTRC;
   __IO uint32_t PCTRE;
   __IO uint32_t PCTR0;
   __IO uint32_t PCTRS0;
   __IO uint32_t PCTR1;
   __IO uint32_t PCTRS1;
   __IO uint32_t PCTR2;
   __IO uint32_t PCTRS2;
   __IO uint32_t PCTR3;
   __IO uint32_t PCTRS3;
   __IO uint32_t PCTR4;
   __IO uint32_t PCTRS4;
   __IO uint32_t PCTR5;
   __IO uint32_t PCTRS5;
   __IO uint32_t PCTR6;
   __IO uint32_t PCTRS6;
   __IO uint32_t PCTR7;
   __IO uint32_t PCTRS7;
   __IO uint32_t PCTR8;
   __IO uint32_t PCTRS8;
   __IO uint32_t PCTR9;
   __IO uint32_t PCTRS9;
   __IO uint32_t PCTR10;
   __IO uint32_t PCTRS10;
   __IO uint32_t PCTR11;
   __IO uint32_t PCTRS11;
   __IO uint32_t PCTR12;
   __IO uint32_t PCTRS12;
   __IO uint32_t PCTR13;
   __IO uint32_t PCTRS13;
   __IO uint32_t PCTR14;
   __IO uint32_t PCTRS14;
   __IO uint32_t PCTR15;
   __IO uint32_t PCTRS15;
   __IO uint32_t RESERVED9[512];
   __IO uint32_t DBGE;
   __IO uint32_t FDBGO;
   __IO uint32_t FDBGB;
   __IO uint32_t FDBGR;
   __IO uint32_t FDBGS;
   __IO uint32_t RESERVED10[3];
   __IO uint32_t ERRSTAT;
} V3D_Type;

typedef struct {
   /* Control and Status register */
   __IO uint32_t SMICS;
   /* length/count (n external transfers) */
   __IO uint32_t SMIL;
   /* address register */
   __IO uint32_t SMIA;
   /* data register */
   __IO uint32_t SMID;
   /* device 0 read settings */
   __IO uint32_t SMIDSR0;
   /* device 0 write settings */
   __IO uint32_t SMIDSW0;
   /* device 1 read settings */
   __IO uint32_t SMIDSR1;
   /* device 1 write settings */
   __IO uint32_t SMIDSW1;
   /* device 2 read settings */
   __IO uint32_t SMIDSR2;
   /* device 2 write settings */
   __IO uint32_t SMIDSW2;
   /* device 3 read settings */
   __IO uint32_t SMIDSR3;
   /* device 3 write settings */
   __IO uint32_t SMIDSW3;
   /* DMA Control Registers */
   __IO uint32_t SMIDC;
   /* direct control/status register */
   __IO uint32_t SMIDCS;
   /* direct address register */
   __IO uint32_t SMIDA;
   /* direct data registers */
   __IO uint32_t SMIDD;
   /* FIFO debug register */
   __IO uint32_t SMIFD;
} SMI_Type;

typedef struct {
   __IO uint32_t RESET;
   __IO uint32_t UNDEF;
   __IO uint32_t SWI;
   __IO uint32_t PABT;
   __IO uint32_t DABT;
   __IO uint32_t RESERVED;
   __IO uint32_t IRQ;
   __IO uint32_t FIQ[20];
} Vector_Table_Type;

typedef struct {
   /* GPIO Function Select 0 */
   __IO uint32_t GPFSEL0;
   /* GPIO Function Select 1 */
   __IO uint32_t GPFSEL1;
   /* GPIO Function Select 2 */
   __IO uint32_t GPFSEL2;
   /* GPIO Function Select 3 */
   __IO uint32_t GPFSEL3;
   /* GPIO Function Select 4 */
   __IO uint32_t GPFSEL4;
   /* GPIO Function Select 5 */
   __IO uint32_t GPFSEL5;
   uint32_t RESERVED0;
   /* GPIO Pin Output Set 0 */
   __O uint32_t GPSET0;
   /* GPIO Pin Output Set 1 */
   __O uint32_t GPSET1;
   uint32_t RESERVED1;
   /* GPIO Pin Output Clear 0 */
   __O uint32_t GPCLR0;
   /* GPIO Pin Output Clear 1 */
   __O uint32_t GPCLR1;
   uint32_t RESERVED2;
   /* GPIO Pin Level 0 */
   __I uint32_t GPLEV0;
   /* GPIO Pin Level 1 */
   __I uint32_t GPLEV1;
   uint32_t RESERVED3;
   /* GPIO Pin Event Detect Status 0 */
   __IO uint32_t GPEDS0;
   /* GPIO Pin Event Detect Status 1 */
   __IO uint32_t GPEDS1;
   uint32_t RESERVED4;
   /* GPIO Pin Rising Edge Detect Enable 0 */
   __IO uint32_t GPREN0;
   /* GPIO Pin Rising Edge Detect Enable 1 */
   __IO uint32_t GPREN1;
   uint32_t RESERVED5;
   /* GPIO Pin Falling Edge Detect Enable 0 */
   __IO uint32_t GPFEN0;
   /* GPIO Pin Falling Edge Detect Enable 1 */
   __IO uint32_t GPFEN1;
   uint32_t RESERVED6;
   /* GPIO Pin High Detect Enable 0 */
   __IO uint32_t GPHEN0;
   /* GPIO Pin High Detect Enable 1 */
   __IO uint32_t GPHEN1;
   uint32_t RESERVED7;
   /* GPIO Pin Low Detect Enable 0 */
   __IO uint32_t GPLEN0;
   /* GPIO Pin Low Detect Enable 1 */
   __IO uint32_t GPLEN1;
   uint32_t RESERVED8;
   /* GPIO Pin Async. Rising Edge Detect 0 */
   __IO uint32_t GPAREN0;
   /* GPIO Pin Async. Rising Edge Detect 1 */
   __IO uint32_t GPAREN1;
   uint32_t RESERVED9;
   /* GPIO Pin Async. Falling Edge Detect 0 */
   __IO uint32_t GPAFEN0;
   /* GPIO Pin Async. Falling Edge Detect 1 */
   __IO uint32_t GPAFEN1;
   uint32_t RESERVED10;
   /* GPIO Pin Pull-up/down Enable */
   __IO uint32_t GPPUD;
   /* GPIO Pin Pull-up/down Enable Clock 0 */
   __IO uint32_t GPPUDCLK0;
   /* GPIO Pin Pull-up/down Enable Clock 1 */
   __IO uint32_t GPPUDCLK1;
   uint32_t RESERVED11; 
   uint32_t RESERVED12; 
   uint32_t RESERVED13; 
   uint32_t RESERVED14; 
   __IO uint32_t TEST;
} GPIO_Type;

typedef struct {
   __IO uint32_t GNRIC;
   __IO uint32_t AUDIO;
   __IO uint32_t RESERVED0[4];
   __IO uint32_t STATUS;
   __IO uint32_t RSTC;
   __IO uint32_t RSTS;
   __IO uint32_t WDOG;
   __IO uint32_t PADS0; //GPIO 0-27
   __IO uint32_t PADS2; //GPIO 28-45
   __IO uint32_t PADS3;  //GPIO 45-53 
   __IO uint32_t PADS4;
   __IO uint32_t PADS5;
   __IO uint32_t PADS6;
   __IO uint32_t RESERVED1;
   __IO uint32_t CAM0;
   __IO uint32_t CAM1;
   __IO uint32_t CCP2TX;
   __IO uint32_t DSI0;
   __IO uint32_t DSI1;
   __IO uint32_t HDMI;
   __IO uint32_t USB;
   __IO uint32_t PXLD0;
   __IO uint32_t PXBG;
   __IO uint32_t DFT;
   __IO uint32_t SMPS;
   __IO uint32_t XOSC;
   __IO uint32_t SPAREW;
   __IO uint32_t SPARER;
   __IO uint32_t AVS_RSTDR;
   __IO uint32_t AVS_STAT;
   __IO uint32_t AVS_EVENT;
   __IO uint32_t AVS_INTEN;
   __IO uint32_t RESERVED2[28];
   __IO uint32_t DUMMY;
   __IO uint32_t RESERVED3[2];
   __IO uint32_t IMAGE;
   __IO uint32_t GRAFX;
   __IO uint32_t PROC;
} Power_Manager_Type;

typedef struct {
   /*Generic Clock Control */
   __IO uint32_t GNRICCTL;
   /*Generic Clock Divisor */
   __IO uint32_t GNRICDIV;
   /* VPU Clock Control */
   __IO uint32_t VPUCTL;
   /* VPU Clock Divisor */
   __IO uint32_t VPUDIV;
   /* System Clock Control */
   __IO uint32_t SYSCTL;
   /* System Clock Divisor */
   __IO uint32_t SYSDIV;
   /* PERIA Clock Control */
   __IO uint32_t PERIACTL;
   /* PERIA Clock Divisor */
   __IO uint32_t PERIADIV;
   /* PERII Clock Control */
   __IO uint32_t PERIICTL;
   /* PERII Clock Divisor */
   __IO uint32_t PERIIDIV;
   /* H264 Clock Control */
   __IO uint32_t H264CTL;
   /* H264 Clock Divisor */
   __IO uint32_t H264DIV;
   /* ISP Clock Control */
   __IO uint32_t ISPCTL;
   /* ISP Clock Divisor */
   __IO uint32_t ISPDIV;
   /* V3D Clock Control */
   __IO uint32_t V3DCTL;
   /* V3D Clock Divisor */
   __IO uint32_t V3DDIV;
   /* Camera 0 Clock Control */
   __IO uint32_t CAM0CTL;
   /* Camera 0 Clock Divisor */
   __IO uint32_t CAM0DIV;
   /* Camera 1 Clock Control */
   __IO uint32_t CAM1CTL;
   /* Camera 1 Clock Divisor */
   __IO uint32_t CAM1DIV;
   /* CCP2 Clock Control */
   __IO uint32_t CCP2CTL;
   /* CCP2 Clock Divisor */
   __IO uint32_t CCP2DIV;
   /* DSI0E Clock Control */
   __IO uint32_t DSI0ECTL;
   /* DSI0E Clock Divisor */
   __IO uint32_t DSI0EDIV;
   /* DSI0P Clock Control */
   __IO uint32_t DSI0PCTL;
   /* DSI0P Clock Divisor */
   __IO uint32_t DSI0PDIV;
   /* DPI Clock Control */
   __IO uint32_t DPICTL;
   /* DPI Clock Divisor */
   __IO uint32_t DPIDIV;
   /* General Purpose Clock Control 0*/
   __IO uint32_t GPCTL0;
   /* General Purpose Clock Divisor 0*/
   __IO uint32_t GPDIV0;
   /* General Purpose Clock Control 1*/
   __IO uint32_t GPCTL1;
   /* General Purpose Clock Divisor 1*/
   __IO uint32_t GPDIV1;
   /* General Purpose Clock Control 2*/
   __IO uint32_t GPCTL2;
   /* General Purpose Clock Divisor 2*/
   __IO uint32_t GPDIV2;
   /* HSM Clock Control */
   __IO uint32_t HSMCTL;
   /* HSM Clock Divisor */
   __IO uint32_t HSMDIV;
   /* OTP Clock Control */
   __IO uint32_t OTPCTL;
   /* OTP Clock Divisor */
   __IO uint32_t OTPDIV;
   /* PCM/I2S Clock Control */
   __IO uint32_t PCMCTL;
   /* PCM/I2S Clock Divisor */
   __IO uint32_t PCMDIV;
   /* PWM Clock Control */
   __IO uint32_t PWMCTL;
   /* PWM Clock Divisor */
   __IO uint32_t PWMDIV;
   /* SLIM Clock Control */
   __IO uint32_t SLIMCTL;
   /* SLIM Clock Divisor */
   __IO uint32_t SLIMDIV;
   /* SMI Clock Control */
   __IO uint32_t SMICTL;
   /* SMI Clock Divisor */
   __IO uint32_t SMIDIV;
   /* TCNT Clock Control */
   __IO uint32_t TCNTCTL;
   /* TCNT Clock Divisor */
   __IO uint32_t TCNTDIV;
   /* TEC Clock Control */
   __IO uint32_t TECCTL;
   /* TEC Clock Divisor */
   __IO uint32_t TECDIV;
   /* TD0 Clock Control */
   __IO uint32_t TD0CTL;
   /* TD0 Clock Divisor */
   __IO uint32_t TD0DIV;
   /* TD1 Clock Control */
   __IO uint32_t TD1CTL;
   /* TD1 Clock Divisor */
   __IO uint32_t TD1DIV;
   /* TSENS Clock Control */
   __IO uint32_t TSENSCTL;
   /* TSENS Clock Divisor */
   __IO uint32_t TSENSDIV;
   /* Timer Clock Control */
   __IO uint32_t TIMERCTL;
   /* Timer Clock Divisor */
   __IO uint32_t TIMERDIV;
   /* UART Clock Control */
   __IO uint32_t UARTCTL;
   /* UART Clock Divisor */
   __IO uint32_t UARTDIV;
   /* VEC Clock Control */
   __IO uint32_t VECCTL;
   /* VEC Clock Divisor */
   __IO uint32_t VECDIV;
   /* Oscillator Count */
   __IO uint32_t OSCCOUNT;
   /* PLLA */
   __IO uint32_t PLLA;
   /* PLLC */
   __IO uint32_t PLLC;
   /* PLLD */
   __IO uint32_t PLLD;
   /* PLLH */
   __IO uint32_t PLLH;
   /* Lock */
   __IO uint32_t LOCK;
   /* Event */
   __IO uint32_t EVENT;
   /* INTEN */
   __IO uint32_t INTEN;
   /* DSI0HSCK */
   __IO uint32_t DSI0HSCK;
   /* CKSM */
   __IO uint32_t CKSM;
   /* Oscillator Frequency Integer */
   __IO uint32_t OSCFREQI;
   /* Oscillator Frequency Fraction */
   __IO uint32_t OSCFREQF;
   /* PLLT Control */
   __IO uint32_t PLLTCTL;
   /* PLLT0 Count */
   __IO uint32_t PLLTCNT0;
   /* PLLT1 Count */
   __IO uint32_t PLLTCNT1;
   /* PLLT2 Count */
   __IO uint32_t PLLTCNT2;
   /* PLLT3 Count */
   __IO uint32_t PLLTCNT3;
   /* TD Clock Enable */
   __IO uint32_t TDCLKEN;
   /* Burst Control */
   __IO uint32_t BURSTCTL;
   /* Burst Count */
   __IO uint32_t BURSTCNT;
   /* DSI1E Clock Control*/
   __IO uint32_t DSI1ECTL;
   /* DSI1E Clock Divisor*/
   __IO uint32_t DSI1EDIV;
   /* DSI1P Clock Control*/
   __IO uint32_t DSI1PCTL;
   /* DSI1P Clock Divisor*/
   __IO uint32_t DSI1PDIV;
   /* DFT Clock Control*/
   __IO uint32_t DFTCTL;
   /* DFT Clock Divisor*/
   __IO uint32_t DFTDIV;
   /* PLLB */
   __IO uint32_t PLLB;
   __IO uint32_t RESERVED0[7];
   /* Pulse Clock Control */
   __IO uint32_t PULSECTL;
   /* Pulse Clock Divisor */
   __IO uint32_t PULSEDIV;
   /* SDC Clock Control */
   __IO uint32_t SDCCTL;
   /* SDC Clock Divisor */
   __IO uint32_t SDCDIV;
   /* ARM Clock Control */
   __IO uint32_t ARMCTL;
   /* ARM Clock Divisor */
   __IO uint32_t ARMDIV;
   /* AVEO Clock Control */
   __IO uint32_t AVEOCTL;
   /* AVEO Clock Divisor */
   __IO uint32_t AVEODIV;
   /* EMMC Clock Control */
   __IO uint32_t EMMCCTL;
   /* EMMC Clock Divisor */
   __IO uint32_t EMMCDIV;
} CM_Type; 

typedef struct {
   /* Control and Status register */
   __IO uint32_t CS;
   /* Conrol Block Address register */
   __IO uint32_t CONBLK_AD;
   /* Control Block Word 0 (Transfer Information) */
   __I uint32_t TI;
   /* Control Block Word 1 (Source Address) */
   __I uint32_t SOURCE_AD;
   /* Control Block Word 2 (Destination Address) */
   __I uint32_t DEST_AD;
   /* Control Block Word 3 (Transfer Length) */
   __I uint32_t TXFR_LEN;
   /* Control Block Word 4 (2D Stride) */
   __I uint32_t STRIDE;
   /* Control Block Word 5 (Next Control Block Address) */
   __I uint32_t NEXTCONBK;
   /* Debug */
   __IO uint32_t DEBUG;
} DMA_Channel_Type;

typedef struct {
   /* Interrupt status of each DMA channel */
   __IO uint32_t INT_STATUS;
   uint32_t RESERVED0;
   uint32_t RESERVED1;
   uint32_t RESERVED2;
   /* Global enable bits for each DMA channel */
   __IO uint32_t ENABLE; 
} DMA_Global_Type;

typedef struct {
   /* Transfer Information */
   uint32_t TI;
   /* Source Address */
   uint32_t SOURCE_AD;
   /* Destination Address */
   uint32_t DEST_AD;
   /* Transfer Length */
   uint32_t TXFR_LEN;
   /* 2D Mode Stride */
   uint32_t STRIDE;
   /* Next Control Block Address */
   uint32_t NEXTCONBK;
   uint32_t RESERVED0;
   uint32_t RESERVED1;
} DMA_Control_Block_Type;

typedef struct {
   __IO uint32_t CONTROL0;
   __IO uint32_t RESERVED0[2];
   __IO uint32_t ID_SECURE;
   __IO uint32_t RESERVED1[60];
   __IO uint32_t TRANSLATE[32];
   __IO uint32_t RESERVED2[32];
   __IO uint32_t IRQ_PEND0;
   __IO uint32_t IRQ_PEND1;
   __IO uint32_t IRQ_PEND2;
   __IO uint32_t IRQ_FAST;
   __IO uint32_t IRQ_ENBL1;
   __IO uint32_t IRQ_ENBL2;
   __IO uint32_t IRQ_ENBL3;
   __IO uint32_t IRQ_DIBL1;
   __IO uint32_t IRQ_DIBL2;
   __IO uint32_t IRQ_DIBL3;
   __IO uint32_t RESERVED3[118];
   __IO uint32_t T_LOAD;
   __IO uint32_t T_VALUE;
   __IO uint32_t T_CONTROL;
   __IO uint32_t T_IRQCNTL;
   __IO uint32_t T_RAWIRQ;
   __IO uint32_t T_MSKIRQ;
   __IO uint32_t T_RELOAD;
   __IO uint32_t T_PREDIV;
   __IO uint32_t T_FREECNT;
   __IO uint32_t RESERVED4[7];
   __IO uint32_t CONTROL1;
   __IO uint32_t STATUS;
   __IO uint32_t ERRHALT;
   __IO uint32_t ID;
   __IO uint32_t RESERVED5[247];
   __IO uint32_t ARM_0_SEM0;
   __IO uint32_t ARM_0_SEM1;
   __IO uint32_t ARM_0_SEM2;
   __IO uint32_t ARM_0_SEM3;
   __IO uint32_t ARM_0_SEM4;
   __IO uint32_t ARM_0_SEM5;
   __IO uint32_t ARM_0_SEM6;
   __IO uint32_t ARM_0_SEM7;
   __IO uint32_t RESERVED6[8];
   __IO uint32_t ARM_0_BELL0;
   __IO uint32_t ARM_0_BELL1;
   __IO uint32_t ARM_0_BELL2;
   __IO uint32_t ARM_0_BELL3;
   __IO uint32_t RESERVED7[12];
   __IO uint32_t ARM_0_MAIL0_RDWRT[4];
   __IO uint32_t ARM_0_MAIL0_POL;
   __IO uint32_t ARM_0_MAIL0_SND;
   __IO uint32_t ARM_0_MAIL0_STA;
   __IO uint32_t ARM_0_MAIL0_CNF;
   __IO uint32_t ARM_0_MAIL1_WRT[4]; //Do not Use
   __IO uint32_t ARM_0_MAIL1_POL; //Do not Use
   __IO uint32_t ARM_0_MAIL1_SND; //Do not Use
   __IO uint32_t ARM_0_MAIL1_STA;
   __IO uint32_t ARM_0_MAIL1_CNF; //Do not Use
   __IO uint32_t RESERVED8[8];
   __IO uint32_t ARM_0_SEMCLRDBG;
   __IO uint32_t ARM_0_BELLCLRDBG;
   __IO uint32_t RESERVED9[4];
   __IO uint32_t ARM_0_ALL_IRQS;
   __IO uint32_t ARM_0_MY_IRQS;
   __IO uint32_t ARM_1_SEM0;
   __IO uint32_t ARM_1_SEM1;
   __IO uint32_t ARM_1_SEM2;
   __IO uint32_t ARM_1_SEM3;
   __IO uint32_t ARM_1_SEM4;
   __IO uint32_t ARM_1_SEM5;
   __IO uint32_t ARM_1_SEM6;
   __IO uint32_t ARM_1_SEM7;
   __IO uint32_t RESERVED10[8];
   __IO uint32_t ARM_1_BELL0;
   __IO uint32_t ARM_1_BELL1;
   __IO uint32_t ARM_1_BELL2;
   __IO uint32_t ARM_1_BELL3;
   __IO uint32_t RESERVED11[12];
   __IO uint32_t ARM_1_MAIL0_RDWRT[4];
   __IO uint32_t ARM_1_MAIL0_POL; //Do not Use
   __IO uint32_t ARM_1_MAIL0_SND; //Do not Use
   __IO uint32_t ARM_1_MAIL0_STA;
   __IO uint32_t ARM_1_MAIL0_CNF; //Do not Use
   __IO uint32_t ARM_1_MAIL1_RDWRT[4];
   __IO uint32_t ARM_1_MAIL1_POL;
   __IO uint32_t ARM_1_MAIL1_SND;
   __IO uint32_t ARM_1_MAIL1_STA;
   __IO uint32_t ARM_1_MAIL1_CNF;
   __IO uint32_t RESERVED12[8];
   __IO uint32_t ARM_1_SEMCLRDBG;
   __IO uint32_t ARM_1_BELLCLRDBG;
   __IO uint32_t RESERVED13[4];
   __IO uint32_t ARM_1_ALL_IRQS;
   __IO uint32_t ARM_1_MY_IRQS;
   __IO uint32_t ARM_2_SEM0;
   __IO uint32_t ARM_2_SEM1;
   __IO uint32_t ARM_2_SEM2;
   __IO uint32_t ARM_2_SEM3;
   __IO uint32_t ARM_2_SEM4;
   __IO uint32_t ARM_2_SEM5;
   __IO uint32_t ARM_2_SEM6;
   __IO uint32_t ARM_2_SEM7;
   __IO uint32_t RESERVED14[8];
   __IO uint32_t ARM_2_BELL0;
   __IO uint32_t ARM_2_BELL1;
   __IO uint32_t ARM_2_BELL2;
   __IO uint32_t ARM_2_BELL3;
   __IO uint32_t RESERVED15[12];
   __IO uint32_t ARM_2_MAIL0_RDWRT[4];
   __IO uint32_t ARM_2_MAIL0_POL; // Do not Use
   __IO uint32_t ARM_2_MAIL0_SND; // Do not Use
   __IO uint32_t ARM_2_MAIL1_STA;
   __IO uint32_t ARM_2_MAIL1_CNF; // Do not use
   __IO uint32_t RESERVED16[8];
   __IO uint32_t ARM_2_SEMCLRDBG;
   __IO uint32_t ARM_2_BELLCLRDBG;
   __IO uint32_t RESERVED17[4];
   __IO uint32_t ARM_2_ALL_IRQS;
   __IO uint32_t ARM_2_MY_IRQS;
   __IO uint32_t ARM_3_SEM0;
   __IO uint32_t ARM_3_SEM1;
   __IO uint32_t ARM_3_SEM2;
   __IO uint32_t ARM_3_SEM3;
   __IO uint32_t ARM_3_SEM4;
   __IO uint32_t ARM_3_SEM5;
   __IO uint32_t ARM_3_SEM6;
   __IO uint32_t ARM_3_SEM7;
   __IO uint32_t RESERVED18[8];
   __IO uint32_t ARM_3_BELL0;
   __IO uint32_t ARM_3_BELL1;
   __IO uint32_t ARM_3_BELL2;
   __IO uint32_t ARM_3_BELL3;
   __IO uint32_t RESERVED19[12];
   __IO uint32_t ARM_3_MAIL0_RDWRT[4];
   __IO uint32_t ARM_3_MAIL0_POL; // Do not Use
   __IO uint32_t ARM_3_MAIL0_SND; // Do not Use
   __IO uint32_t ARM_3_MAIL0_STA;
   __IO uint32_t ARM_3_MAIL0_CNF; // Do not Use
   __IO uint32_t ARM_3_MAIL1_RDWRT[4];
   __IO uint32_t ARM_3_MAIL1_POL; //Do not use
   __IO uint32_t ARM_3_MAIL1_SND; //Do not use
   __IO uint32_t ARM_3_MAIL1_STA;
   __IO uint32_t ARM_3_MAIL1_CNF; // Do not use
   __IO uint32_t RESERVED20[8];
   __IO uint32_t ARM_3_SEMCLRDBG;
   __IO uint32_t ARM_3_BELLCLRDBG;
   __IO uint32_t RESERVED21[4];
   __IO uint32_t ARM_3_ALL_IRQS;
   __IO uint32_t ARM_3_MY_IRQS;
} ARM_Control_Type;
   
typedef struct {
   /* 0x200 bytes for */
   /* Page aligned boundary to start at PERIPH_BASE + 0x0000B000) */
  // __IO uint32_t RESERVED[128];
   /* Interrupt Basic Pending */
   __I uint32_t IRQBP;
   /* Interrupt Pending 1 */
   __I uint32_t IRQP1;
   /* Interrupt Pending 2 */
   __I uint32_t IRQP2;
   /* FIQ control */
   __IO uint32_t FIQCNTL;
   /* Enable Interrupts 1 */
   __IO uint32_t IRQEN1;
   /* Enable Interrupts 2 */
   __IO uint32_t IRQEN2;
   /* Enable Basic Interrupts */
   __IO uint32_t BASIC_IRQEN;
   /* Disable Interrupts 1 */
   __IO uint32_t DISABLE_IRQ1;
   /* Disable Interrupts 2 */
   __IO uint32_t DISABLE_IRQ2;
   /* Disable Basic Interrupts */
   __IO uint32_t DISABLE_BASIC_IRQ;
} Interrupt_Controller_Type;

typedef struct {
   /* PWM Control */
   __IO uint32_t CTL;
   /* PWM Status */
   __IO uint32_t STA;
   /* PWM DMA Configuration */
   __IO uint32_t DMAC;
   /* PWM Channel 1 Range */
   __IO uint32_t RNG1;
   /* PWM Channel 1 Data */
   __IO uint32_t DAT1;
   /* PWM FIFO Input */
   __IO uint32_t FIF1;
   /* PWM Channel 2 Range */
   __IO uint32_t RNG2;
   /* PWM Channel 2 Data */
   __IO uint32_t DAT2;
} PWM_Type;

typedef struct {
   /* Main control and status bits for the SPI */
   __IO uint32_t CS;
   /* FIFO TX/RX buffer */ 
   __IO uint32_t FIFO;
   /* SPI clock rate */
   __IO uint32_t CLK;
   /* SPI data length rate */
   uint32_t DLEN;
   /* LoSSI output hold delay */
   uint32_t LTOH;
   /* Generation of DREQ and Panic signals */
   /* to an external DMA */
   uint32_t DC;
} SPI_Type;

typedef struct {
   /* Auxiliary Interrupt status */
   __I uint32_t AUX_IRQ;
   /* Auxiliary enables */
   __IO uint32_t AUX_ENABLES;
   __IO uint32_t RESERVED0[14];
   /* Mini UART I/O Data */
   __IO uint32_t AUX_MU_IO_REG;
   /* Mini UART Interrupt Enable */
   __IO uint32_t AUX_MU_IER_REG;
   /* Mini UART Interrupt Identify */
   __IO uint32_t AUX_MU_IIR_REG;
   /* Mini UART Line Control */
   __IO uint32_t AUX_MU_LCR_REG;
   /* Mini UART Modem Control */
   __IO uint32_t AUX_MU_MCR_REG;
   /* Mini UART Line Status */
   __I uint32_t AUX_MU_LSR_REG;
   /* Mini UART Modem Status */
   __I uint32_t AUX_MU_MSR_REG;
   /* Mini UART Scratch */
   __IO uint32_t AUX_MU_SCRATCH;
   /* Mini UART Extra Control */
   __IO uint32_t AUX_MU_CNTL_REG;
   /* Mini UART Extra Status */
   __I uint32_t AUX_MU_STAT_REG;
   /* Mini UART Baudrate */
   __IO uint32_t AUX_MU_BAUD_REG;
   __IO uint32_t RESERVED1[5];
   /* SPI 1 Control Register 0 */
   __IO uint32_t AUX_SPI0_CNTL0_REG;
   /* SPI 1 Control Register 1 */
   __IO uint32_t AUX_SPI0_CNTL1_REG;
   /* SPI 1 Status */
   __I uint32_t AUX_SPI0_STAT_REG;
   __IO uint32_t RESERVED2;
   /* SPI 1 Data */
   __IO uint32_t AUX_SPI0_IO_REG;
   /* SPI 1 Peek */
   __I uint32_t AUX_SPI0_PEEK_REG;
   __IO uint32_t RESERVED3[2];
   /* SPI 2 Control register 0 */
   __IO uint32_t AUX_SPI1_CNTL0_REG;
   /* SPI 2 Control register 1 */
   __IO uint32_t AUX_SPI1_CNTL1_REG;
   /* SPI 2 Status */
   __I uint32_t AUX_SPI1_STAT_REG;
   __IO uint32_t RESERVED4;
   /* SPI 2 Data */
   __IO uint32_t AUX_SPI1_IO_REG;
   /* SPI 2 Peek */
   __I uint32_t AUX_SPI1_PEEK_REG;
} AUX_Type;

typedef struct {
   /* System Timer Control/Status */
   __IO uint32_t CS;
   /* System Timer Counter Lower 32 bits */
   __I uint32_t CLO;
   /* System Timer Counter Higher 32 bits */
   __I uint32_t CHI;
   /* System Timer Compare 0 */
   __IO uint32_t C0;
   /* System Timer Compare 1 */
   __IO uint32_t C1;
   /* System Timer Compare 2 */
   __IO uint32_t C2;
   /* System Timer Compare 3 */
   __IO uint32_t C3;
} SysTimer_Type;

typedef struct {
   /* 0x400 bytes for */
   /* Page-aligned boundary to start at PERIPH_BASE + 0x0000B000 */
   __IO uint32_t RESERVED0[256];
   /* Timer Load Register */
   __IO uint32_t LD;
   /* Timer Value Register (R) */
   __I uint32_t VAL;
   /* Timer control Register */
   __IO uint32_t CTL;
   /* Timer IRQ clear register (W) */
   __O uint32_t IRQ_CLR_ACK;
   /* Timer Raw IRQ register (R) */
   __I uint32_t IRQ_RAW;
   /* Timer Masked IRQ register (R) */
   __I uint32_t IRQ_MASKED;
   /* Timer Reload register */
   __IO uint32_t RELOAD;
   /* Timer Pre-Divider register */
   __IO uint32_t PD;
   /* Free-running counter */
   __I uint32_t FRC;
} ARMTimer_Type;

/* Provide ARM timer, IRQs, mailboxes */
typedef struct {
   /* 0x40000000 Control Register */
   __IO uint32_t CTL;
   /* 0x40000004 unused */
   __IO uint32_t RESERVED0;
   /* 0x40000008 core timer prescaler */
   __IO uint32_t CORETIMER_PRESCALER;
   /* 0x4000000C GPU interrupts routing */
   __IO uint32_t GPU_IRQ_ROUTING;
   /* 0x40000010 Performance monitor interrupts routing write-set */
   __O uint32_t PMU_IRQ_ROUTING_SET;
   /* 0x40000014 Performance monitor interrupts routing write-clear */
   __O uint32_t PMU_IRQ_ROUTING_CLR;
   /* 0x40000018 unused */
   __IO uint32_t RESERVED1;
   /* 0x4000001C Core timer access LS 32 bits */
   __IO uint32_t CORETIMER_BITS_LO;
   /* 0x40000020 Core timer access MS 32 bits */
   __IO uint32_t CORETIMER_BITS_HI;
   /* 0x40000024 Local Interrupt 0 [1-7] routing */
   __IO uint32_t LOCAL_IRQ_ROUTING;
   /* 0x40000028 Local Interrupts 8-15 routing (unused) */
   __IO uint32_t RESERVED2;
   /* 0x4000002C AXI outstanding counters */
   __IO uint32_t AXI_OUTSTANDING_COUNTERS;
   /* 0x40000030 AXI outstanding IRQ */
   __IO uint32_t AXI_OUTSTANDING_IRQ;
   /* 0x40000034 Local Timer control & status */
   __IO uint32_t LTCSR;
   /* 0x40000038 Local Timer write flags (IRQ Clear and reload) (W) */
   __O uint32_t LTWFR;
   /* 0x4000003C unused */
   __IO uint32_t RESERVED3;
   /* 0x40000040 Core0 Timers Interrupt control */
   __IO uint32_t CORE0_TIMERS_IRQ_CTL;
   /* 0x40000044 Core1 Timers Interrupt control */
   __IO uint32_t CORE1_TIMERS_IRQ_CTL;
   /* 0x40000048 Core2 Timers Interrupt control */
   __IO uint32_t CORE2_TIMERS_IRQ_CTL;
   /* 0x4000004C Core3 Timers Interrupt control */
   __IO uint32_t CORE3_TIMERS_IRQ_CTL;
   /* 0x40000050 Core0 Mailboxes Interrupt control */
   __IO uint32_t CORE0_MAILBOXES_IRQ_CTL;
   /* 0x40000054 Core1 Mailboxes Interrupt control */
   __IO uint32_t CORE1_MAILBOXES_IRQ_CTL;
   /* 0x40000058 Core2 Mailboxes Interrupt control */
   __IO uint32_t CORE2_MAILBOXES_IRQ_CTL;
   /* 0x4000005C Core3 Mailboxes Interrupt control */
   __IO uint32_t CORE3_MAILBOXES_IRQ_CTL;
   /* 0x40000060 Core0 IRQ Source */
   __IO uint32_t CORE0_IRQ_SRC;
   /* 0x40000064 Core1 IRQ Source */
   __IO uint32_t CORE1_IRQ_SRC;
   /* 0x40000068 Core2 IRQ Source */
   __IO uint32_t CORE2_IRQ_SRC;
   /* 0x4000006C Core3 IRQ Source */
   __IO uint32_t CORE3_IRQ_SRC;
   /* 0x40000070 Core0 FIQ Source */
   __IO uint32_t CORE0_FIQ_SRC;
   /* 0x40000074 Core1 FIQ Source */
   __IO uint32_t CORE1_FIQ_SRC;
   /* 0x40000078 Core2 FIQ Source */
   __IO uint32_t CORE2_FIQ_SRC;
   /* 0x4000007C Core3 FIQ Source */
   __IO uint32_t CORE3_FIQ_SRC;
   /* 0x40000080 Core0 Mailbox 0 write-set (W) */
   __O uint32_t CORE0_MAILBOX_0_WRITE_SET;
   /* 0x40000084 Core0 Mailbox 1 write-set (W) */
   __O uint32_t CORE0_MAILBOX_1_WRITE_SET;
   /* 0x40000088 Core0 Mailbox 2 write-set (W) */
   __O uint32_t CORE0_MAILBOX_2_WRITE_SET;
   /* 0x4000008C Core0 Mailbox 3 write-set (W) */
   __O uint32_t CORE0_MAILBOX_3_WRITE_SET;
   /* 0x40000090 Core1 Mailbox 0 write-set (W) */
   __O uint32_t CORE1_MAILBOX_0_WRITE_SET;
   /* 0x40000094 Core1 Mailbox 1 write-set (W) */
   __O uint32_t CORE1_MAILBOX_1_WRITE_SET;
   /* 0x40000098 Core1 Mailbox 2 write-set (W) */
   __O uint32_t CORE1_MAILBOX_2_WRITE_SET;
   /* 0x4000009C Core1 Mailbox 3 write-set (W) */
   __O uint32_t CORE1_MAILBOX_3_WRITE_SET;
   /* 0x400000A0 Core2 Mailbox 0 write-set (W) */
   __O uint32_t CORE2_MAILBOX_0_WRITE_SET;
   /* 0x400000A4 Core2 Mailbox 1 write-set (W) */
   __O uint32_t CORE2_MAILBOX_1_WRITE_SET;
   /* 0x400000A8 Core2 Mailbox 2 write-set (W) */
   __O uint32_t CORE2_MAILBOX_2_WRITE_SET;
   /* 0x400000AC Core2 Mailbox 3 write-set (W) */
   __O uint32_t CORE2_MAILBOX_3_WRITE_SET;
   /* 0x400000B0 Core3 Mailbox 0 write-set (W) */
   __O uint32_t CORE3_MAILBOX_0_WRITE_SET;
   /* 0x400000B4 Core3 Mailbox 1 write-set (W) */
   __O uint32_t CORE3_MAILBOX_1_WRITE_SET;
   /* 0x400000B8 Core3 Mailbox 2 write-set (W) */
   __O uint32_t CORE3_MAILBOX_2_WRITE_SET;
   /* 0x400000BC Core3 Mailbox 3 write-set (W) */
   __O uint32_t CORE3_MAILBOX_3_WRITE_SET;
   /* 0x400000C0 Core0 Mailbox 0 read & write-high-to-clear */
   __IO uint32_t CORE0_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000C4 Core0 Mailbox 1 read & write-high-to-clear */
   __IO uint32_t CORE0_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000C8 Core0 Mailbox 2 read & write-high-to-clear */
   __IO uint32_t CORE0_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000CC Core0 Mailbox 3 read & write-high-to-clear */
   __IO uint32_t CORE0_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000D0 Core1 Mailbox 0 read & write-high-to-clear */
   __IO uint32_t CORE1_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000D4 Core1 Mailbox 1 read & write-high-to-clear */
   __IO uint32_t CORE1_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000D8 Core1 Mailbox 2 read & write-high-to-clear */
   __IO uint32_t CORE1_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000DC Core1 Mailbox 3 read & write-high-to-clear */
   __IO uint32_t CORE1_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000E0 Core2 Mailbox 0 read & write-high-to-clear */
   __IO uint32_t CORE2_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000E4 Core2 Mailbox 1 read & write-high-to-clear */
   __IO uint32_t CORE2_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000E8 Core2 Mailbox 2 read & write-high-to-clear */
   __IO uint32_t CORE2_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000EC Core2 Mailbox 3 read & write-high-to-clear */
   __IO uint32_t CORE2_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000F0 Core3 Mailbox 0 read & write-high-to-clear */
   __IO uint32_t CORE3_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000F4 Core3 Mailbox 1 read & write-high-to-clear */
   __IO uint32_t CORE3_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000F8 Core3 Mailbox 2 read & write-high-to-clear */
   __IO uint32_t CORE3_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000FC Core3 Mailbox 3 read & write-high-to-clear */
   __IO uint32_t CORE3_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR; 
} ARM_Control_Logic_Module_Type;

/* Peripheral Declaration */
#define DMA_CH0 ((DMA_Channel_Type *) DMA0_DMA14_BASE)
#define DMA_CH1 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0x100))
#define DMA_CH2 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0x200))
#define DMA_CH3 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0x300))
#define DMA_CH4 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0x400))

/* PWM DMA */
#define DMA_CH5 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0x500))

#define DMA_CH6 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0x600))
#define DMA_CH7 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0x700))
#define DMA_CH8 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0x800))
#define DMA_CH9 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0x900))
#define DMA_CH10 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0xA00))
#define DMA_CH11 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0xB00))
#define DMA_CH12 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0xC00))
#define DMA_CH13 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0xD00))
#define DMA_CH14 ((DMA_Channel_Type *) (DMA0_DMA14_BASE + 0xE00))
#define DMA_GLOBAL_CTLS ((DMA_Global_Type *) (DMA0_DMA14_BASE + 0xFE0))
#define DMA_CH15 ((DMA_Channel_Type *) DMA15_BASE)

#define GPIO ((GPIO_Type *) GPIO_BASE)
#define SYS_TMR ((SysTimer_Type *) SYS_TMR_BASE)
#define ARM_TMR ((ARMTimer_Type *) ARM_TMR_BASE)
#define AUX ((AUX_Type *) AUX_BASE)
#define SPI ((SPI_Type *) SPI_BASE)
#define PWM ((PWM_Type* ) PWM_BASE)
#define IRQ_CTLS ((Interrupt_Controller_Type *) IRQ_BASE)
#define CM ((CM_Type *) CM_BASE)
#define PADS ((GPIO_Pads_Control_Type *) GPIO_PADS_CTL_BASE)
#define ARMCTL ((ARM_Control_Logic_Module_Type *) LOCAL_PERIPH_BASE) 
#define INTCTRL0 ((IC_Type *) IC0_BASE)
#define INTCTRL1 ((IC_Type *) IC1_BASE)
#define PWRMAN ((Power_Manager_Type *) PWRMAN_BASE)
#define ISP ((ISP_Type *) ISP_BASE)
#define V3D ((V3D_Type *) V3D_BASE)
#define HVS ((HVS_Type *) HVS_BASE)
#define VEC ((VEC_Type *) VEC_BASE)
#define ARMCTL ((ARM_Control_Type *) ARMCTL_BASE)
#define AJB ((ARM_JTAG_BASH_Type *) AJB_BASE)
#define PIXVALVE0 ((Pixel_Valve_Type *) PIXVALVE0_BASE)
#define PIXVALVE1 ((Pixel_Valve_Type *) PIXVALVE1_BASE)
#define PIXVALVE2 ((Pixel_Valve_Type *) PIXVALVE2_BASE)
#define SDRAMCTL ((SDRAM_Controller_Type *) SD_BASE)
#define MULTICORE ((Multicore_Sync_Type *) MS_BASE)
#define SDHOST ((SD_Host_Type *) SH_BASE)
#define DPHY_CSR ((SDRAM_DPHY_Type *) DPHY_CSR_BASE)
#define APHY_CSR ((SDRAM_APHY_Type *) APHY_CSR_BASE)
#define A2W ((A2W_Type *) A2W_BASE)

/* Peripheral register control bits */
/* R/W - read and write bit */
/* W1C - write 1 to clear the bit */
/* RO - read only bit */
/* W1SC - write 1, will self-clear */

/* DMA Bits */

/* CS - DMA Control and Status Register (RW) */
/* Activate the DMA bit (R/W) */
#define DMA_CS_ACTIVE_OFS (0)
#define DMA_CS_ACTIVE ((uint32_t) 0x00000001)
/* DMA End Flag bit (W1C) */
#define DMA_CS_END_OFS (1)
#define DMA_CS_END ((uint32_t) 0x00000002)
/* DMA Interrupt Status bit (W1C) */
#define DMA_CS_INT_OFS (2)
#define DMA_CS_INT ((uint32_t) 0x00000004)
/* DMA DREQ (Data Request) state bit (RO) */
#define DMA_CS_DREQ_OFS (3)
#define DMA_CS_DREQ ((uint32_t) 0x00000008)
/* DMA Paused State bit (RO) */
#define DMA_CS_PAUSED_OFS (4)
#define DMA_CS_PAUSED ((uint32_t) 0x00000010)
/* DMA Paused by DREQ State bit (RO) */
#define DMA_CS_DREQ_STOPS_DMA_OFS (5)
#define DMA_CS_DREQ_STOPS_DMA ((uint32_t) 0x00000020)
/* Waiting for Outstanding Writes bit (RO) */
#define DMA_CS_WAITING_FOR_OUTSTANDING_WRITES_OFS (6)
#define DMA_CS_WAITING_FOR_OUTSTANDING_WRITES ((uint32_t) 0x00000040)
/* DMA Error (RO) bit */
#define DMA_CS_ERROR_OFS (8)
#define DMA_CS_ERROR ((uint32_t) 0x00000100)
/* AXI Priority Level bits (RW) */
#define DMA_CS_PRIORITY_OFS (16)
#define DMA_CS_PRIORITY_MASK ((uint32_t) 0x000F0000)
#define DMA_CS_PRIORITY0 ((uint32_t) 0x00010000)
#define DMA_CS_PRIORITY1 ((uint32_t) 0x00020000)
#define DMA_CS_PRIORITY2 ((uint32_t) 0x00040000)
#define DMA_CS_PRIORITY3 ((uint32_t) 0x00080000)
/* AXI Panic Priority Level bits (RW) */
#define DMA_CS_PANIC_PRIORITY_OFS (20)
#define DMA_CS_PANIC_PRIORITY_MASK ((uint32_t) 0x00F00000)
#define DMA_CS_PANIC_PRIORITY0 ((uint32_t) 0x00100000)
#define DMA_CS_PANIC_PRIORITY1 ((uint32_t) 0x00200000)
#define DMA_CS_PANIC_PRIORITY2 ((uint32_t) 0x00400000)
#define DMA_CS_PANIC_PRIORITY3 ((uint32_t) 0x00800000)
/* Wait for outstanding writes bit (RW) */
#define DMA_CS_WAIT_FOR_OUTSTANDING_WRITES_OFS (28)
#define DMA_CS_WAIT_FOR_OUTSTANDING_WRITES ((uint32_t) 0x10000000)
/* Disable debug pause signal bit (RW) */
#define DMA_CS_DISDEBUG_OFS (29)
#define DMA_CS_DISDEBUG ((uint32_t) 0x20000000)
/* Abort DMA bit (W1SC) */
#define DMA_CS_ABORT_OFS (30)
#define DMA_CS_ABORT ((uint32_t) 0x40000000)
/* DMA Channel Reset bit (W1SC) */
#define DMA_CS_RESET_OFS (31)
#define DMA_CS_RESET ((uint32_t) 0x80000000)

/* TI - DMA Transfer Information register (RW) */
/* DMA0 - DMA6 */
/* Interrupt Enable bit (RW) */
#define DMA0_DMA6_TI_INTEN_OFS (0)
#define DMA0_DMA6_TI_INTEN ((uint32_t) 0x00000001)
/* 2D Mode bit (RW) */
#define DMA0_DMA6_TI_TDMODE_OFS (1)
#define DMA0_DMA6_TI_TDMODE ((uint32_t) 0x00000002)
/* Wait for a Write Response bit (RW) */
#define DMA0_DMA6_TI_WAIT_RESP_OFS (3)
#define DMA0_DMA6_TI_WAIT_RESP ((uint32_t) 0x00000008)
/* Destination Address Increment bit (RW) */
#define DMA0_DMA6_TI_DEST_INC_OFS (4)
#define DMA0_DMA6_TI_DEST_INC ((uint32_t) 0x00000010)
/* Destination Transfer Width bit (RW) */
#define DMA0_DMA6_TI_DEST_WIDTH_OFS (5)
#define DMA0_DMA6_TI_DEST_WIDTH ((uint32_t) 0x00000020)
/* Control Destination Writes with DREQ bit (RW) */
#define DMA0_DMA6_TI_DEST_DREQ_OFS (6)
#define DMA0_DMA6_TI_DEST_DREQ ((uint32_t) 0x00000040)
/* Ignore Writes bit (RW) */
#define DMA0_DMA6_TI_DEST_IGNORE_OFS (7)
#define DMA0_DMA6_TI_DEST_IGNORE ((uint32_t) 0x00000080)
/* Source Address Increment bit (RW) */
#define DMA0_DMA6_TI_SRC_INC_OFS (8)
#define DMA0_DMA6_TI_SRC_INC ((uint32_t) 0x00000100)
/* Source Transfer Width bit (RW) */
#define DMA0_DMA6_TI_SRC_WIDTH_OFS (9)
#define DMA0_DMA6_TI_SRC_WIDTH ((uint32_t) 0x00000200)
/* Controle Source Reads with DREQ bit (RW) */
#define DMA0_DMA6_TI_SRC_DREQ_OFS (10)
#define DMA0_DMA6_TI_SRC_DREQ ((uint32_t) 0x00000400)
/* Ignore Reads bit (RW) */
#define DMA0_DMA6_TI_SRC_IGNORE_OFS (11)
#define DMA0_DMA6_TI_SRC_IGNORE ((uint32_t) 0x0000080)
/* Burst Transfer Length bits (RW) */
#define DMA0_DMA6_TI_BURST_LENGTH_OFS (12)
#define DMA0_DMA6_TI_BURST_LENGTH_MASK ((uint32_t) 0x0000F000)
#define DMA0_DMA6_TI_BURST_LENGTH0 ((uint32_t) 0x0000100)
#define DMA0_DMA6_TI_BURST_LENGTH1 ((uint32_t) 0x0000200)
#define DMA0_DMA6_TI_BURST_LENGTH2 ((uint32_t) 0x0000400)
#define DMA0_DMA6_TI_BURST_LENGTH3 ((uint32_t) 0x0000800)
/* Peripheral Mapping bits (RW) */
#define DMA0_DMA6_TI_PERMAP_OFS (16)
#define DMA0_DMA6_TI_PERMAP_MASK ((uint32_t) 0x001F0000)
#define DMA0_DMA6_TI_PERMAP0 ((uint32_t) 0x00010000)
#define DMA0_DMA6_TI_PERMAP1 ((uint32_t) 0x00020000)
#define DMA0_DMA6_TI_PERMAP2 ((uint32_t) 0x00040000)
#define DMA0_DMA6_TI_PERMAP3 ((uint32_t) 0x00080000)
#define DMA0_DMA6_TI_PERMAP4 ((uint32_t) 0x00100000)
/* Add Wait Cycles bits (RW) */
#define DMA0_DMA6_TI_WAITS_OFS (21)
#define DMA0_DMA6_TI_WAITS_MASK ((uint32_t) 0x03E00000)
#define DMA0_DMA6_TI_WAITS0 ((uint32_t) 0x00200000)
#define DMA0_DMA6_TI_WAITS1 ((uint32_t) 0x00400000)
#define DMA0_DMA6_TI_WAITS2 ((uint32_t) 0x00800000)
#define DMA0_DMA6_TI_WAITS3 ((uint32_t) 0x01000000)
#define DMA0_DMA6_TI_WAITS4 ((uint32_t) 0x02000000)
/* No Wide Bursts bit (RW) */
#define DMA0_DMA6_TI_NO_WIDE_BURSTS_OFS (26)
#define DMA0_DMA6_TI_NO_WIDE_BURSTS ((uint32_t) 0x04000000)
/* DMA7 - DMA 14 */
/* Interrupt Enable bit (RW) */
#define DMA7_DMA14_TI_INTEN_OFS (0)
#define DMA7_DMA14_TI_INTEN ((uint32_t) 0x00000001)
/* Wait for a Write Response bit (RW) */
#define DMA7_DMA14_TI_WAIT_RESP_OFS (3)
#define DMA7_DMA14_TI_WAIT_RESP ((uint32_t) 0x00000008)
/* Destination Address Increment bit (RW) */
#define DMA7_DMA14_TI_DEST_INC_OFS (4)
#define DMA7_DMA14_TI_DEST_INC ((uint32_t) 0x00000010)
/* Destination Transfer Width bit (RW) */
#define DMA7_DMA14_TI_DEST_WIDTH_OFS (5)
#define DMA7_DMA14_TI_DEST_WIDTH ((uint32_t) 0x00000020)
/* Control Destination Writes with DREQ bit (RW) */
#define DMA7_DMA14_TI_DEST_DREQ_OFS (6)
#define DMA7_DMA14_TI_DEST_DREQ ((uint32_t) 0x00000040)
/* Ignore Writes bit (RW) - no description */
#define DMA7_DMA14_TI_DEST_IGNORE_OFS (7)
#define DMA7_DMA14_TI_DEST_IGNORE ((uint32_t) 0x00000080)
/* Source Address Increment bit (RW) */
#define DMA7_DMA14_TI_SRC_INC_OFS (8)
#define DMA7_DMA14_TI_SRC_INC ((uint32_t) 0x00000100)
/* Source Transfer Width bit (RW) */
#define DMA7_DMA14_TI_SRC_WIDTH_OFS (9)
#define DMA7_DMA14_TI_SRC_WIDTH ((uint32_t) 0x00000200)
/* Controle Source Reads with DREQ bit (RW) */
#define DMA7_DMA14_TI_SRC_DREQ_OFS (10)
#define DMA7_DMA14_TI_SRC_DREQ ((uint32_t) 0x00000400)
/* Ignore Reads bit (RW) - no description */
#define DMA7_DMA14_TI_SRC_IGNORE_OFS (11)
#define DMA7_DMA14_TI_SRC_IGNORE ((uint32_t) 0x0000080)
/* Burst Transfer Length bits (RW) */
#define DMA7_DMA14_TI_BURST_LENGTH_OFS (12)
#define DMA7_DMA14_TI_BURST_LENGTH_MASK ((uint32_t) 0x0000F000)
#define DMA7_DMA14_TI_BURST_LENGTH0 ((uint32_t) 0x0000100)
#define DMA7_DMA14_TI_BURST_LENGTH1 ((uint32_t) 0x0000200)
#define DMA7_DMA14_TI_BURST_LENGTH2 ((uint32_t) 0x0000400)
#define DMA7_DMA14_TI_BURST_LENGTH3 ((uint32_t) 0x0000800)
/* Peripheral Mapping bits (RW) */
#define DMA7_DMA14_TI_PERMAP_OFS (16)
#define DMA7_DMA14_TI_PERMAP_MASK ((uint32_t) 0x001F0000)
#define DMA7_DMA14_TI_PERMAP0 ((uint32_t) 0x00010000)
#define DMA7_DMA14_TI_PERMAP1 ((uint32_t) 0x00020000)
#define DMA7_DMA14_TI_PERMAP2 ((uint32_t) 0x00040000)
#define DMA7_DMA14_TI_PERMAP3 ((uint32_t) 0x00080000)
#define DMA7_DMA14_TI_PERMAP4 ((uint32_t) 0x00100000)
/* Add Wait Cycles bits (RW) */
#define DMA7_DMA14_TI_WAITS_OFS (21)
#define DMA7_DMA14_TI_WAITS_MASK ((uint32_t) 0x03E00000)
#define DMA7_DMA14_TI_WAITS0 ((uint32_t) 0x00200000)
#define DMA7_DMA14_TI_WAITS1 ((uint32_t) 0x00400000)
#define DMA7_DMA14_TI_WAITS2 ((uint32_t) 0x00800000)
#define DMA7_DMA14_TI_WAITS3 ((uint32_t) 0x01000000)
#define DMA7_DMA14_TI_WAITS4 ((uint32_t) 0x02000000)

/* TXFR_LEN - DMA Transfer Length Register (RW) */
/* DMA0 - DMA6 */
/* Transfer Length in bytes bits (RW) */
#define DMA0_DMA6_TXFR_LEN_XLENGTH_OFS (0)
#define DMA0_DMA6_TXFR_LEN_XLENGTH_MASK ((uint32_t) 0x0000FFFF)
/* YLENGTH bits (RW) */
#define DMA0_DMA6_TXFR_LEN_YLENGTH_OFS (16)
#define DMA0_DMA6_TXFR_LEN_YLENGTH_MASK ((uint32_t) 0x3FFF0000)
/* DMA7 - DMA14 */
/* Transfer Length in bytes bits (RW) */
#define DMA7_DMA14_TXFR_LEN_XLENGTH_OFS (0)
#define DMA7_DMA14_TXFR_LEN_XLENGTH_MASK ((uint32_t) 0x0000FFFF)

/* STRIDE - DMA 2D Stride Register (RW) */
/* DMA0 - DMA6 */
/* Source Stride (2D Mode) bits (RW) */
#define DMA0_DMA6_STRIDE_S_STRIDE_OFS (0)
#define DMA0_DMA6_STRIDE_S_STRIDE_MASK ((uint32_t) 0x0000FFFF)
/* Destination Stride (2D Mode) bits (RW) */
#define DMA0_DMA6_STRIDE_D_STRIDE_OFS (16)
#define DMA0_DMA6_STRIDE_D_STRIDE_MASK ((uint32_t) 0xFFFF0000)


/* DEBUG - DMA Debug Register (RW) */
/* DMA0 - DMA6 */
/* Read Last Not Set Error bit (RW) */
#define DMA0_DMA6_DEBUG_READ_LAST_NOT_SET_ERROR_OFS (0)
#define DMA0_DMA6_DEBUG_READ_LAST_NOT_SET_ERROR ((uint32_t) 0x00000001)
/* FIFO Error (RW) */
#define DMA0_DMA6_DEBUG_FIFO_ERROR_OFS (1)
#define DMA0_DMA6_DEBUG_FIFO_ERROR ((uint32_t) 0x00000002)
/* Slave Read Response Error (RW) */
#define DMA0_DMA6_DEBUG_READ_ERROR_OFS (2)
#define DMA0_DMA6_DEBUG_READ_ERROR ((uint32_t) 0x00000004)
/* DMA Outstanding Writes Counter bits (RO) */
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES_OFS (4)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES_MASK ((uint32_t) 0x000000F0)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES0 ((uint32_t) 0x00000010)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES1 ((uint32_t) 0x00000020)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES2 ((uint32_t) 0x00000040)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES3 ((uint32_t) 0x00000080)
/* DMA ID bits (RO) */
#define DMA0_DMA6_DEBUG_DMA_ID_OFS (8)
#define DMA0_DMA6_DEBUG_DMA_ID_MASK ((uint32_t) 0x0000FF00)
#define DMA0_DMA6_DEBUG_DMA_ID0 ((uint32_t) 0x00000100)
#define DMA0_DMA6_DEBUG_DMA_ID1 ((uint32_t) 0x00000200)
#define DMA0_DMA6_DEBUG_DMA_ID2 ((uint32_t) 0x00000400)
#define DMA0_DMA6_DEBUG_DMA_ID3 ((uint32_t) 0x00000800)
#define DMA0_DMA6_DEBUG_DMA_ID4 ((uint32_t) 0x00001000)
#define DMA0_DMA6_DEBUG_DMA_ID5 ((uint32_t) 0x00002000)
#define DMA0_DMA6_DEBUG_DMA_ID6 ((uint32_t) 0x00004000)
#define DMA0_DMA6_DEBUG_DMA_ID7 ((uint32_t) 0x00008000)
/* DMA State Machine State bits (RO) */
#define DMA0_DMA6_DEBUG_DMA_STATE_OFS (16)
#define DMA0_DMA6_DEBUG_DMA_STATE_MASK ((uint32_t) 0x01FF0000)
#define DMA0_DMA6_DEBUG_DMA_STATE0 ((uint32_t) 0x00010000)
#define DMA0_DMA6_DEBUG_DMA_STATE1 ((uint32_t) 0x00020000)
#define DMA0_DMA6_DEBUG_DMA_STATE2 ((uint32_t) 0x00040000)
#define DMA0_DMA6_DEBUG_DMA_STATE3 ((uint32_t) 0x00080000)
#define DMA0_DMA6_DEBUG_DMA_STATE4 ((uint32_t) 0x00100000)
#define DMA0_DMA6_DEBUG_DMA_STATE5 ((uint32_t) 0x00200000)
#define DMA0_DMA6_DEBUG_DMA_STATE6 ((uint32_t) 0x00400000)
#define DMA0_DMA6_DEBUG_DMA_STATE7 ((uint32_t) 0x00800000)
#define DMA0_DMA6_DEBUG_DMA_STATE8 ((uint32_t) 0x01000000)
/* DMA Version bits (RO) */
#define DMA0_DMA6_DEBUG_VERSION_OFS (25)
#define DMA0_DMA6_DEBUG_VERSION_MASK ((uint32_t) 0x0E000000)
#define DMA0_DMA6_DEBUG_VERSION0 ((uint32_t) 0x02000000)
#define DMA0_DMA6_DEBUG_VERSION1 ((uint32_t) 0x04000000)
#define DMA0_DMA6_DEBUG_VERSION2 ((uint32_t) 0x08000000)
/* DMA Lite bit (RO) */
#define DMA0_DMA6_DEBUG_LITE_OFS (28)
#define DMA0_DMA6_DEBUG_LITE ((uint32_t) 0x10000000)
/* DMA7 - DMA14 */
/* Read Last Not Set Error bit (RW) */
#define DMA7_DMA14_DEBUG_READ_LAST_NOT_SET_ERROR_OFS (0)
#define DMA7_DMA14_DEBUG_READ_LAST_NOT_SET_ERROR ((uint32_t) 0x00000001)
/* FIFO Error (RW) */
#define DMA7_DMA14_DEBUG_FIFO_ERROR_OFS (1)
#define DMA7_DMA14_DEBUG_FIFO_ERROR ((uint32_t) 0x00000002)
/* Slave Read Response Error (RW) */
#define DMA7_DMA14_DEBUG_READ_ERROR_OFS (2)
#define DMA7_DMA14_DEBUG_READ_ERROR ((uint32_t) 0x00000004)
/* DMA Outstanding Writes Counter bits (RO) */
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES_OFS (4)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES_MASK ((uint32_t) 0x000000F0)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES0 ((uint32_t) 0x00000010)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES1 ((uint32_t) 0x00000020)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES2 ((uint32_t) 0x00000040)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES3 ((uint32_t) 0x00000080)
/* DMA ID bits (RO) */
#define DMA7_DMA14_DEBUG_DMA_ID_OFS (8)
#define DMA7_DMA14_DEBUG_DMA_ID_MASK ((uint32_t) 0x0000FF00)
#define DMA7_DMA14_DEBUG_DMA_ID0 ((uint32_t) 0x00000100)
#define DMA7_DMA14_DEBUG_DMA_ID1 ((uint32_t) 0x00000200)
#define DMA7_DMA14_DEBUG_DMA_ID2 ((uint32_t) 0x00000400)
#define DMA7_DMA14_DEBUG_DMA_ID3 ((uint32_t) 0x00000800)
#define DMA7_DMA14_DEBUG_DMA_ID4 ((uint32_t) 0x00001000)
#define DMA7_DMA14_DEBUG_DMA_ID5 ((uint32_t) 0x00002000)
#define DMA7_DMA14_DEBUG_DMA_ID6 ((uint32_t) 0x00004000)
#define DMA7_DMA14_DEBUG_DMA_ID7 ((uint32_t) 0x00008000)
/* DMA State Machine State bits (RO) */
#define DMA7_DMA14_DEBUG_DMA_STATE_OFS (16)
#define DMA7_DMA14_DEBUG_DMA_STATE_MASK ((uint32_t) 0x01FF0000)
#define DMA7_DMA14_DEBUG_DMA_STATE0 ((uint32_t) 0x00010000)
#define DMA7_DMA14_DEBUG_DMA_STATE1 ((uint32_t) 0x00020000)
#define DMA7_DMA14_DEBUG_DMA_STATE2 ((uint32_t) 0x00040000)
#define DMA7_DMA14_DEBUG_DMA_STATE3 ((uint32_t) 0x00080000)
#define DMA7_DMA14_DEBUG_DMA_STATE4 ((uint32_t) 0x00100000)
#define DMA7_DMA14_DEBUG_DMA_STATE5 ((uint32_t) 0x00200000)
#define DMA7_DMA14_DEBUG_DMA_STATE6 ((uint32_t) 0x00400000)
#define DMA7_DMA14_DEBUG_DMA_STATE7 ((uint32_t) 0x00800000)
#define DMA7_DMA14_DEBUG_DMA_STATE8 ((uint32_t) 0x01000000)
/* DMA Version bits (RO) */
#define DMA7_DMA14_DEBUG_VERSION_OFS (25)
#define DMA7_DMA14_DEBUG_VERSION_MASK ((uint32_t) 0x0E000000)
#define DMA7_DMA14_DEBUG_VERSION0 ((uint32_t) 0x02000000)
#define DMA7_DMA14_DEBUG_VERSION1 ((uint32_t) 0x04000000)
#define DMA7_DMA14_DEBUG_VERSION2 ((uint32_t) 0x08000000)
/* DMA Lite bit (RO) */
#define DMA7_DMA14_DEBUG_LITE_OFS (28)
#define DMA7_DMA14_DEBUG_LITE ((uint32_t) 0x10000000)

/* S_ADDR - DMA Source Address register (RW) */
/* DMA Source Address bits (RW) */
#define DMA_S_ADDR_OFS (0)
#define DMA_S_ADDR_MASK ((uint32_t) 0xFFFFFFFF)

/* D_ADDR - DMA Destination Address register (RW) */
/* DMA Destination Address bits (RW) */
#define DMA_D_ADDR_OFS (0)
#define DMA_D_ADDR_MASK ((uint32_t) 0xFFFFFFFF)

/* CONBLK - DMA Control Block Address register (RW) */
/* Control Block Address bits (RW) */
#define DMA_CONBLK_SCB_ADDR_OFS (0)
#define DMA_CONBLK_SCB_ADDR_MASK ((uint32_t) 0xFFFFFFFF)

/* NEXTCONBK - Next Control Block Address Register (RW) */
/* Address of next CB for chained DMA operations (RW) */
#define DMA_NEXTCONBK_ADDR_OFS (0)
#define DMA_NEXTCONBK_ADDR_MASK ((uint32_t) 0xFFFFFFFF)

/* DMA Global Register bits */
/* INT_STATUS - Interrupt Status of each DMA engine (RW) */
#define DMA_INT_STATUS_INT0_OFS (0)
#define DMA_INT_STATUS_INT0 ((uint32_t) 0x00000001)
#define DMA_INT_STATUS_INT1_OFS (1)
#define DMA_INT_STATUS_INT1 ((uint32_t) 0x00000002)
#define DMA_INT_STATUS_INT2_OFS (2)
#define DMA_INT_STATUS_INT2 ((uint32_t) 0x00000004)
#define DMA_INT_STATUS_INT3_OFS (3)
#define DMA_INT_STATUS_INT3 ((uint32_t) 0x00000008)
#define DMA_INT_STATUS_INT4_OFS (4)
#define DMA_INT_STATUS_INT4 ((uint32_t) 0x00000010)
#define DMA_INT_STATUS_INT5_OFS (5)
#define DMA_INT_STATUS_INT5 ((uint32_t) 0x00000020)
#define DMA_INT_STATUS_INT6_OFS (6)
#define DMA_INT_STATUS_INT6 ((uint32_t) 0x00000040)
#define DMA_INT_STATUS_INT7_OFS (7)
#define DMA_INT_STATUS_INT7 ((uint32_t) 0x00000080)
#define DMA_INT_STATUS_INT8_OFS (8)
#define DMA_INT_STATUS_INT8 ((uint32_t) 0x00000100)
#define DMA_INT_STATUS_INT9_OFS (9)
#define DMA_INT_STATUS_INT9 ((uint32_t) 0x00000200)
#define DMA_INT_STATUS_INT10_OFS (10)
#define DMA_INT_STATUS_INT10 ((uint32_t) 0x00000400)
#define DMA_INT_STATUS_INT11_OFS (11)
#define DMA_INT_STATUS_INT11 ((uint32_t) 0x00000800)
#define DMA_INT_STATUS_INT12_OFS (12)
#define DMA_INT_STATUS_INT12 ((uint32_t) 0x00001000)
#define DMA_INT_STATUS_INT13_OFS (13)
#define DMA_INT_STATUS_INT13 ((uint32_t) 0x00002000)
#define DMA_INT_STATUS_INT14_OFS (14)
#define DMA_INT_STATUS_INT14 ((uint32_t) 0x00004000)

/* Global DMA Enable Register (RW) */
#define DMA_ENABLE_EN0_OFS (0)
#define DMA_ENABLE_EN0 ((uint32_t) 0x00000001)
#define DMA_ENABLE_EN1_OFS (1)
#define DMA_ENABLE_EN1 ((uint32_t) 0x00000002)
#define DMA_ENABLE_EN2_OFS (2)
#define DMA_ENABLE_EN2 ((uint32_t) 0x00000004)
#define DMA_ENABLE_EN3_OFS (3)
#define DMA_ENABLE_EN3 ((uint32_t) 0x00000008)
#define DMA_ENABLE_EN4_OFS (4)
#define DMA_ENABLE_EN4 ((uint32_t) 0x00000010)
#define DMA_ENABLE_EN5_OFS (5)
#define DMA_ENABLE_EN5 ((uint32_t) 0x00000020)
#define DMA_ENABLE_EN6_OFS (6)
#define DMA_ENABLE_EN6 ((uint32_t) 0x00000040)
#define DMA_ENABLE_EN7_OFS (7)
#define DMA_ENABLE_EN7 ((uint32_t) 0x00000080)
#define DMA_ENABLE_EN8_OFS (8)
#define DMA_ENABLE_EN8 ((uint32_t) 0x00000100)
#define DMA_ENABLE_EN9_OFS (9)
#define DMA_ENABLE_EN9 ((uint32_t) 0x00000200)
#define DMA_ENABLE_EN10_OFS (10)
#define DMA_ENABLE_EN10 ((uint32_t) 0x00000400)
#define DMA_ENABLE_EN11_OFS (11)
#define DMA_ENABLE_EN11 ((uint32_t) 0x00000800)
#define DMA_ENABLE_EN12_OFS (12)
#define DMA_ENABLE_EN12 ((uint32_t) 0x00001000)
#define DMA_ENABLE_EN13_OFS (13)
#define DMA_ENABLE_EN13 ((uint32_t) 0x00002000)
#define DMA_ENABLE_EN14_OFS (14)
#define DMA_ENABLE_EN14 ((uint32_t) 0x00004000)

/* PWM Bits */

/* GPIO Bits */
/* GPIO Function Select 0 Register (RW) */
/* FSEL0 - Function Select 0 bits (RW) */
#define GPIO_GPFSEL0_FSEL0_OFS (0)
#define GPIO_GPFSEL0_FSEL0_MASK ((uint32_t) 0x00000007)
#define GPIO_GPFSEL0_FSEL00 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL0_FSEL01 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL0_FSEL02 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL0_FSEL0_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL0_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL0_FSEL0_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL0_FSEL0_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL0_FSEL0_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL0_FSEL0_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL0_FSEL0_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL0_FSEL0_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL0_FSEL0__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL0__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL0_FSEL0__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL0_FSEL0__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL0_FSEL0__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL0_FSEL0__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL0_FSEL0__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL0_FSEL0__ALT3 ((uint32_t) 0x00000007)
/* FSEL1 - Function Select 1 bits (RW) */
#define GPIO_GPFSEL0_FSEL1_OFS (3)
#define GPIO_GPFSEL0_FSEL1_MASK ((uint32_t) 0x00000038)
#define GPIO_GPFSEL0_FSEL10 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL0_FSEL11 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL0_FSEL12 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL0_FSEL1_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL1_1 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL0_FSEL1_2 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL0_FSEL1_3 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL0_FSEL1_4 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL0_FSEL1_5 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL0_FSEL1_6 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL0_FSEL1_7 ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL0_FSEL1__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL1__OUTPUT ((uint32_t) 0x00000040)
#define GPIO_GPFSEL0_FSEL1__ALT5 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL0_FSEL1__ALT4 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL0_FSEL1__ALT0 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL0_FSEL1__ALT1 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL0_FSEL1__ALT2 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL0_FSEL1__ALT3 ((uint32_t) 0x000001C0)
/* FSEL2 - Function Select 2 bits (RW) */
#define GPIO_GPFSEL0_FSEL2_OFS (6)
#define GPIO_GPFSEL0_FSEL2_MASK ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL0_FSEL20 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL0_FSEL21 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL0_FSEL22 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL0_FSEL2_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL2_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL0_FSEL2_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL0_FSEL2_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL0_FSEL2_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL0_FSEL2_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL0_FSEL2_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL0_FSEL2_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL0_FSEL2__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL2__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL0_FSEL2__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL0_FSEL2__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL0_FSEL2__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL0_FSEL2__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL0_FSEL2__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL0_FSEL2__ALT3 ((uint32_t) 0x00000007)
/* FSEL3 - Function Select 3 bits (RW) */
#define GPIO_GPFSEL0_FSEL3_OFS (9)
#define GPIO_GPFSEL0_FSEL3_MASK ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL0_FSEL30 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL0_FSEL31 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL0_FSEL32 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL0_FSEL3_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL3_1 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL0_FSEL3_2 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL0_FSEL3_3 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL0_FSEL3_4 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL0_FSEL3_5 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL0_FSEL3_6 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL0_FSEL3_7 ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL0_FSEL3__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL3__OUTPUT ((uint32_t) 0x0000200)
#define GPIO_GPFSEL0_FSEL3__ALT5 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL0_FSEL3__ALT4 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL0_FSEL3__ALT0 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL0_FSEL3__ALT1 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL0_FSEL3__ALT2 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL0_FSEL3__ALT3 ((uint32_t) 0x00000E00)
/* FSEL4 - Function Select 4 bits (RW) */
#define GPIO_GPFSEL0_FSEL4_OFS (12)
#define GPIO_GPFSEL0_FSEL4_MASK ((uint32_t) 0x00007000)
#define GPIO_GPFSEL0_FSEL40 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL0_FSEL41 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL0_FSEL42 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL0_FSEL4_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL4_1 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL0_FSEL4_2 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL0_FSEL4_3 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL0_FSEL4_4 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL0_FSEL4_5 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL0_FSEL4_6 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL0_FSEL4_7 ((uint32_t) 0x00007000)
#define GPIO_GPFSEL0_FSEL4__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL4__OUTPUT ((uint32_t) 0x00001000)
#define GPIO_GPFSEL0_FSEL4__ALT5 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL0_FSEL4__ALT4 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL0_FSEL4__ALT0 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL0_FSEL4__ALT1 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL0_FSEL4__ALT2 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL0_FSEL4__ALT3 ((uint32_t) 0x00007000)
/* FSEL5 - Function Select 5 bits (RW) */
#define GPIO_GPFSEL0_FSEL5_OFS (15)
#define GPIO_GPFSEL0_FSEL5_MASK ((uint32_t) 0x00038000)
#define GPIO_GPFSEL0_FSEL50 ((uint32_t) 0x00008000)
#define GPIO_GPFSEL0_FSEL51 ((uint32_t) 0x00010000)
#define GPIO_GPFSEL0_FSEL52 ((uint32_t) 0x00020000)
#define GPIO_GPFSEL0_FSEL5_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL5_1 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL0_FSEL5_2 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL0_FSEL5_3 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL0_FSEL5_4 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL0_FSEL5_5 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL0_FSEL5_6 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL0_FSEL5_7 ((uint32_t) 0x00380000)
#define GPIO_GPFSEL0_FSEL5__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL5__OUTPUT ((uint32_t) 0x00080000)
#define GPIO_GPFSEL0_FSEL5__ALT5 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL0_FSEL5__ALT4 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL0_FSEL5__ALT0 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL0_FSEL5__ALT1 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL0_FSEL5__ALT2 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL0_FSEL5__ALT3 ((uint32_t) 0x00380000)
/* FSEL6 - Function Select 6 bits (RW) */
#define GPIO_GPFSEL0_FSEL6_OFS (18)
#define GPIO_GPFSEL0_FSEL6_MASK ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL0_FSEL60 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL0_FSEL61 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL0_FSEL62 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL0_FSEL6_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL6_1 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL0_FSEL6_2 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL0_FSEL6_3 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL0_FSEL6_4 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL0_FSEL6_5 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL0_FSEL6_6 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL0_FSEL6_7 ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL0_FSEL6__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL6__OUTPUT ((uint32_t) 0x00040000)
#define GPIO_GPFSEL0_FSEL6__ALT5 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL0_FSEL6__ALT4 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL0_FSEL6__ALT0 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL0_FSEL6__ALT1 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL0_FSEL6__ALT2 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL0_FSEL6__ALT3 ((uint32_t) 0x001C0000)
/* FSEL7 - Function Select 7 bits (RW) */
#define GPIO_GPFSEL0_FSEL7_OFS (21)
#define GPIO_GPFSEL0_FSEL7_MASK ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL0_FSEL70 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL0_FSEL71 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL0_FSEL72 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL0_FSEL7_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL7_1 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL0_FSEL7_2 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL0_FSEL7_3 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL0_FSEL7_4 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL0_FSEL7_5 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL0_FSEL7_6 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL0_FSEL7_7 ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL0_FSEL7__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL7__OUTPUT ((uint32_t) 0x00200000)
#define GPIO_GPFSEL0_FSEL7__ALT5 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL0_FSEL7__ALT4 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL0_FSEL7__ALT0 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL0_FSEL7__ALT1 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL0_FSEL7__ALT2 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL0_FSEL7__ALT3 ((uint32_t) 0x00E00000)
/* FSEL8 - Function Select 8 bits (RW) */
#define GPIO_GPFSEL0_FSEL8_OFS (24)
#define GPIO_GPFSEL0_FSEL8_MASK ((uint32_t) 0x07000000)
#define GPIO_GPFSEL0_FSEL80 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL0_FSEL81 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL0_FSEL82 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL0_FSEL8_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL8_1 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL0_FSEL8_2 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL0_FSEL8_3 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL0_FSEL8_4 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL0_FSEL8_5 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL0_FSEL8_6 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL0_FSEL8_7 ((uint32_t) 0x07000000)
#define GPIO_GPFSEL0_FSEL8__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL8__OUTPUT ((uint32_t) 0x01000000)
#define GPIO_GPFSEL0_FSEL8__ALT5 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL0_FSEL8__ALT4 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL0_FSEL8__ALT0 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL0_FSEL8__ALT1 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL0_FSEL8__ALT2 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL0_FSEL8__ALT3 ((uint32_t) 0x07000000)
/* FSEL9 - Function Select 9 bits (RW) */
#define GPIO_GPFSEL0_FSEL9_OFS (27)
#define GPIO_GPFSEL0_FSEL9_MASK ((uint32_t) 0x38000000)
#define GPIO_GPFSEL0_FSEL90 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL0_FSEL91 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL0_FSEL92 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL0_FSEL9_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL9_1 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL0_FSEL9_2 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL0_FSEL9_3 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL0_FSEL9_4 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL0_FSEL9_5 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL0_FSEL9_6 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL0_FSEL9_7 ((uint32_t) 0x38000000)
#define GPIO_GPFSEL0_FSEL9__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL0_FSEL9__OUTPUT ((uint32_t) 0x08000000)
#define GPIO_GPFSEL0_FSEL9__ALT5 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL0_FSEL9__ALT4 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL0_FSEL9__ALT0 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL0_FSEL9__ALT1 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL0_FSEL9__ALT2 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL0_FSEL9__ALT3 ((uint32_t) 0x30000000)

/* GPIO Function Select 1 Register (RW) */
/* FSEL10 - Function Select 10 bits (RW) */
#define GPIO_GPFSEL1_FSEL10_OFS (0)
#define GPIO_GPFSEL1_FSEL10_MASK ((uint32_t) 0x00000007)
#define GPIO_GPFSEL1_FSEL100 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL1_FSEL101 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL1_FSEL102 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL1_FSEL10_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL10_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL1_FSEL10_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL1_FSEL10_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL1_FSEL10_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL1_FSEL10_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL1_FSEL10_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL1_FSEL10_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL1_FSEL10__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL10__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL1_FSEL10__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL1_FSEL10__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL1_FSEL10__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL1_FSEL10__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL1_FSEL10__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL1_FSEL10__ALT3 ((uint32_t) 0x00000007)
/* FSEL11 - Function Select 11 bits (RW) */
#define GPIO_GPFSEL1_FSEL11_OFS (3)
#define GPIO_GPFSEL1_FSEL11_MASK ((uint32_t) 0x00000038)
#define GPIO_GPFSEL1_FSEL110 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL1_FSEL111 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL1_FSEL112 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL1_FSEL11_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL11_1 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL1_FSEL11_2 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL1_FSEL11_3 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL1_FSEL11_4 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL1_FSEL11_5 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL1_FSEL11_6 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL1_FSEL11_7 ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL1_FSEL11__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL11__OUTPUT ((uint32_t) 0x00000040)
#define GPIO_GPFSEL1_FSEL11__ALT5 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL1_FSEL11__ALT4 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL1_FSEL11__ALT0 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL1_FSEL11__ALT1 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL1_FSEL11__ALT2 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL1_FSEL11__ALT3 ((uint32_t) 0x000001C0)
/* FSEL12 - Function Select 12 bits (RW) */
#define GPIO_GPFSEL1_FSEL12_OFS (6)
#define GPIO_GPFSEL1_FSEL12_MASK ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL1_FSEL120 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL1_FSEL121 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL1_FSEL122 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL1_FSEL12_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL12_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL1_FSEL12_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL1_FSEL12_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL1_FSEL12_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL1_FSEL12_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL1_FSEL12_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL1_FSEL12_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL1_FSEL12__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL12__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL1_FSEL12__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL1_FSEL12__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL1_FSEL12__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL1_FSEL12__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL1_FSEL12__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL1_FSEL12__ALT3 ((uint32_t) 0x00000007)
/* FSEL13 - Function Select 13 bits (RW) */
#define GPIO_GPFSEL1_FSEL13_OFS (9)
#define GPIO_GPFSEL1_FSEL13_MASK ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL1_FSEL130 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL1_FSEL131 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL1_FSEL132 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL1_FSEL13_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL13_1 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL1_FSEL13_2 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL1_FSEL13_3 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL1_FSEL13_4 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL1_FSEL13_5 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL1_FSEL13_6 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL1_FSEL13_7 ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL1_FSEL13__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL13__OUTPUT ((uint32_t) 0x0000200)
#define GPIO_GPFSEL1_FSEL13__ALT5 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL1_FSEL13__ALT4 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL1_FSEL13__ALT0 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL1_FSEL13__ALT1 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL1_FSEL13__ALT2 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL1_FSEL13__ALT3 ((uint32_t) 0x00000E00)
/* FSEL14 - Function Select 14 bits (RW) */
#define GPIO_GPFSEL1_FSEL14_OFS (12)
#define GPIO_GPFSEL1_FSEL14_MASK ((uint32_t) 0x00007000)
#define GPIO_GPFSEL1_FSEL140 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL1_FSEL141 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL1_FSEL142 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL1_FSEL14_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL14_1 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL1_FSEL14_2 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL1_FSEL14_3 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL1_FSEL14_4 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL1_FSEL14_5 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL1_FSEL14_6 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL1_FSEL14_7 ((uint32_t) 0x00007000)
#define GPIO_GPFSEL1_FSEL14__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL14__OUTPUT ((uint32_t) 0x00001000)
#define GPIO_GPFSEL1_FSEL14__ALT5 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL1_FSEL14__ALT4 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL1_FSEL14__ALT0 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL1_FSEL14__ALT1 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL1_FSEL14__ALT2 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL1_FSEL14__ALT3 ((uint32_t) 0x00007000)
/* FSEL15 - Function Select 15 bits (RW) */
#define GPIO_GPFSEL1_FSEL15_OFS (15)
#define GPIO_GPFSEL1_FSEL15_MASK ((uint32_t) 0x00038000)
#define GPIO_GPFSEL1_FSEL150 ((uint32_t) 0x00008000)
#define GPIO_GPFSEL1_FSEL151 ((uint32_t) 0x00010000)
#define GPIO_GPFSEL1_FSEL152 ((uint32_t) 0x00020000)
#define GPIO_GPFSEL1_FSEL15_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL15_1 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL1_FSEL15_2 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL1_FSEL15_3 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL1_FSEL15_4 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL1_FSEL15_5 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL1_FSEL15_6 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL1_FSEL15_7 ((uint32_t) 0x00380000)
#define GPIO_GPFSEL1_FSEL15__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL15__OUTPUT ((uint32_t) 0x00080000)
#define GPIO_GPFSEL1_FSEL15__ALT5 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL1_FSEL15__ALT4 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL1_FSEL15__ALT0 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL1_FSEL15__ALT1 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL1_FSEL15__ALT2 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL1_FSEL15__ALT3 ((uint32_t) 0x00380000)
/* FSEL16 - Function Select 16 bits (RW) */
#define GPIO_GPFSEL1_FSEL16_OFS (18)
#define GPIO_GPFSEL1_FSEL16_MASK ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL1_FSEL160 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL1_FSEL161 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL1_FSEL162 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL1_FSEL16_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL16_1 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL1_FSEL16_2 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL1_FSEL16_3 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL1_FSEL16_4 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL1_FSEL16_5 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL1_FSEL16_6 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL1_FSEL16_7 ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL1_FSEL16__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL16__OUTPUT ((uint32_t) 0x00040000)
#define GPIO_GPFSEL1_FSEL16__ALT5 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL1_FSEL16__ALT4 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL1_FSEL16__ALT0 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL1_FSEL16__ALT1 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL1_FSEL16__ALT2 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL1_FSEL16__ALT3 ((uint32_t) 0x001C0000)
/* FSEL17 - Function Select 17 bits (RW) */
#define GPIO_GPFSEL1_FSEL17_OFS (21)
#define GPIO_GPFSEL1_FSEL17_MASK ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL1_FSEL170 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL1_FSEL171 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL1_FSEL172 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL1_FSEL17_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL17_1 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL1_FSEL17_2 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL1_FSEL17_3 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL1_FSEL17_4 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL1_FSEL17_5 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL1_FSEL17_6 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL1_FSEL17_7 ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL1_FSEL17__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL17__OUTPUT ((uint32_t) 0x00200000)
#define GPIO_GPFSEL1_FSEL17__ALT5 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL1_FSEL17__ALT4 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL1_FSEL17__ALT0 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL1_FSEL17__ALT1 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL1_FSEL17__ALT2 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL1_FSEL17__ALT3 ((uint32_t) 0x00E00000)
/* FSEL18 - Function Select 18 bits (RW) */
#define GPIO_GPFSEL1_FSEL18_OFS (24)
#define GPIO_GPFSEL1_FSEL18_MASK ((uint32_t) 0x07000000)
#define GPIO_GPFSEL1_FSEL180 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL1_FSEL181 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL1_FSEL182 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL1_FSEL18_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL18_1 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL1_FSEL18_2 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL1_FSEL18_3 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL1_FSEL18_4 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL1_FSEL18_5 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL1_FSEL18_6 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL1_FSEL18_7 ((uint32_t) 0x07000000)
#define GPIO_GPFSEL1_FSEL18__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL18__OUTPUT ((uint32_t) 0x01000000)
#define GPIO_GPFSEL1_FSEL18__ALT5 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL1_FSEL18__ALT4 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL1_FSEL18__ALT0 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL1_FSEL18__ALT1 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL1_FSEL18__ALT2 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL1_FSEL18__ALT3 ((uint32_t) 0x07000000)
/* FSEL19 - Function Select 19 bits (RW) */
#define GPIO_GPFSEL1_FSEL19_OFS (27)
#define GPIO_GPFSEL1_FSEL19_MASK ((uint32_t) 0x38000000)
#define GPIO_GPFSEL1_FSEL190 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL1_FSEL191 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL1_FSEL192 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL1_FSEL19_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL19_1 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL1_FSEL19_2 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL1_FSEL19_3 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL1_FSEL19_4 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL1_FSEL19_5 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL1_FSEL19_6 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL1_FSEL19_7 ((uint32_t) 0x38000000)
#define GPIO_GPFSEL1_FSEL19__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL1_FSEL19__OUTPUT ((uint32_t) 0x08000000)
#define GPIO_GPFSEL1_FSEL19__ALT5 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL1_FSEL19__ALT4 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL1_FSEL19__ALT0 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL1_FSEL19__ALT1 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL1_FSEL19__ALT2 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL1_FSEL19__ALT3 ((uint32_t) 0x30000000)
/* GPIO Function Select 2 Register (RW) */
/* FSEL20 - Function Select 20 bits (RW) */
#define GPIO_GPFSEL2_FSEL20_OFS (0)
#define GPIO_GPFSEL2_FSEL20_MASK ((uint32_t) 0x00000007)
#define GPIO_GPFSEL2_FSEL200 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL2_FSEL201 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL2_FSEL202 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL2_FSEL20_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL20_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL2_FSEL20_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL2_FSEL20_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL2_FSEL20_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL2_FSEL20_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL2_FSEL20_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL2_FSEL20_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL2_FSEL20__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL20__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL2_FSEL20__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL2_FSEL20__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL2_FSEL20__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL2_FSEL20__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL2_FSEL20__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL2_FSEL20__ALT3 ((uint32_t) 0x00000007)
/* FSEL21 - Function Select 21 bits (RW) */
#define GPIO_GPFSEL2_FSEL21_OFS (3)
#define GPIO_GPFSEL2_FSEL21_MASK ((uint32_t) 0x00000038)
#define GPIO_GPFSEL2_FSEL210 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL2_FSEL211 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL2_FSEL212 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL2_FSEL21_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL21_1 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL2_FSEL21_2 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL2_FSEL21_3 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL2_FSEL21_4 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL2_FSEL21_5 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL2_FSEL21_6 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL2_FSEL21_7 ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL2_FSEL21__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL21__OUTPUT ((uint32_t) 0x00000040)
#define GPIO_GPFSEL2_FSEL21__ALT5 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL2_FSEL21__ALT4 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL2_FSEL21__ALT0 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL2_FSEL21__ALT1 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL2_FSEL21__ALT2 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL2_FSEL21__ALT3 ((uint32_t) 0x000001C0)
/* FSEL22 - Function Select 22 bits (RW) */
#define GPIO_GPFSEL2_FSEL22_OFS (6)
#define GPIO_GPFSEL2_FSEL22_MASK ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL2_FSEL220 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL2_FSEL221 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL2_FSEL222 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL2_FSEL22_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL22_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL2_FSEL22_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL2_FSEL22_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL2_FSEL22_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL2_FSEL22_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL2_FSEL22_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL2_FSEL22_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL2_FSEL22__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL22__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL2_FSEL22__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL2_FSEL22__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL2_FSEL22__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL2_FSEL22__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL2_FSEL22__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL2_FSEL22__ALT3 ((uint32_t) 0x00000007)
/* FSEL23 - Function Select 23 bits (RW) */
#define GPIO_GPFSEL2_FSEL23_OFS (9)
#define GPIO_GPFSEL2_FSEL23_MASK ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL2_FSEL230 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL2_FSEL231 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL2_FSEL232 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL2_FSEL23_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL23_1 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL2_FSEL23_2 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL2_FSEL23_3 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL2_FSEL23_4 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL2_FSEL23_5 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL2_FSEL23_6 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL2_FSEL23_7 ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL2_FSEL23__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL23__OUTPUT ((uint32_t) 0x0000200)
#define GPIO_GPFSEL2_FSEL23__ALT5 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL2_FSEL23__ALT4 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL2_FSEL23__ALT0 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL2_FSEL23__ALT1 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL2_FSEL23__ALT2 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL2_FSEL23__ALT3 ((uint32_t) 0x00000E00)
/* FSEL24 - Function Select 24 bits (RW) */
#define GPIO_GPFSEL2_FSEL24_OFS (12)
#define GPIO_GPFSEL2_FSEL24_MASK ((uint32_t) 0x00007000)
#define GPIO_GPFSEL2_FSEL240 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL2_FSEL241 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL2_FSEL242 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL2_FSEL24_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL24_1 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL2_FSEL24_2 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL2_FSEL24_3 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL2_FSEL24_4 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL2_FSEL24_5 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL2_FSEL24_6 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL2_FSEL24_7 ((uint32_t) 0x00007000)
#define GPIO_GPFSEL2_FSEL24__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL24__OUTPUT ((uint32_t) 0x00001000)
#define GPIO_GPFSEL2_FSEL24__ALT5 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL2_FSEL24__ALT4 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL2_FSEL24__ALT0 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL2_FSEL24__ALT1 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL2_FSEL24__ALT2 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL2_FSEL24__ALT3 ((uint32_t) 0x00007000)
/* FSEL25 - Function Select 25 bits (RW) */
#define GPIO_GPFSEL2_FSEL25_OFS (15)
#define GPIO_GPFSEL2_FSEL25_MASK ((uint32_t) 0x00038000)
#define GPIO_GPFSEL2_FSEL250 ((uint32_t) 0x00008000)
#define GPIO_GPFSEL2_FSEL251 ((uint32_t) 0x00010000)
#define GPIO_GPFSEL2_FSEL252 ((uint32_t) 0x00020000)
#define GPIO_GPFSEL2_FSEL25_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL25_1 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL2_FSEL25_2 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL2_FSEL25_3 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL2_FSEL25_4 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL2_FSEL25_5 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL2_FSEL25_6 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL2_FSEL25_7 ((uint32_t) 0x00380000)
#define GPIO_GPFSEL2_FSEL25__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL25__OUTPUT ((uint32_t) 0x00080000)
#define GPIO_GPFSEL2_FSEL25__ALT5 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL2_FSEL25__ALT4 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL2_FSEL25__ALT0 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL2_FSEL25__ALT1 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL2_FSEL25__ALT2 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL2_FSEL25__ALT3 ((uint32_t) 0x00380000)
/* FSEL26 - Function Select 26 bits (RW) */
#define GPIO_GPFSEL2_FSEL26_OFS (18)
#define GPIO_GPFSEL2_FSEL26_MASK ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL2_FSEL260 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL2_FSEL261 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL2_FSEL262 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL2_FSEL26_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL26_1 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL2_FSEL26_2 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL2_FSEL26_3 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL2_FSEL26_4 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL2_FSEL26_5 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL2_FSEL26_6 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL2_FSEL26_7 ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL2_FSEL26__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL26__OUTPUT ((uint32_t) 0x00040000)
#define GPIO_GPFSEL2_FSEL26__ALT5 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL2_FSEL26__ALT4 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL2_FSEL26__ALT0 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL2_FSEL26__ALT1 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL2_FSEL26__ALT2 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL2_FSEL26__ALT3 ((uint32_t) 0x001C0000)
/* FSEL27 - Function Select 27 bits (RW) */
#define GPIO_GPFSEL2_FSEL27_OFS (21)
#define GPIO_GPFSEL2_FSEL27_MASK ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL2_FSEL270 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL2_FSEL271 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL2_FSEL272 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL2_FSEL27_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL27_1 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL2_FSEL27_2 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL2_FSEL27_3 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL2_FSEL27_4 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL2_FSEL27_5 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL2_FSEL27_6 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL2_FSEL27_7 ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL2_FSEL27__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL27__OUTPUT ((uint32_t) 0x00200000)
#define GPIO_GPFSEL2_FSEL27__ALT5 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL2_FSEL27__ALT4 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL2_FSEL27__ALT0 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL2_FSEL27__ALT1 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL2_FSEL27__ALT2 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL2_FSEL27__ALT3 ((uint32_t) 0x00E00000)
/* FSEL28 - Function Select 28 bits (RW) */
#define GPIO_GPFSEL2_FSEL28_OFS (24)
#define GPIO_GPFSEL2_FSEL28_MASK ((uint32_t) 0x07000000)
#define GPIO_GPFSEL2_FSEL280 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL2_FSEL281 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL2_FSEL282 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL2_FSEL28_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL28_1 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL2_FSEL28_2 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL2_FSEL28_3 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL2_FSEL28_4 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL2_FSEL28_5 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL2_FSEL28_6 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL2_FSEL28_7 ((uint32_t) 0x07000000)
#define GPIO_GPFSEL2_FSEL28__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL28__OUTPUT ((uint32_t) 0x01000000)
#define GPIO_GPFSEL2_FSEL28__ALT5 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL2_FSEL28__ALT4 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL2_FSEL28__ALT0 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL2_FSEL28__ALT1 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL2_FSEL28__ALT2 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL2_FSEL28__ALT3 ((uint32_t) 0x07000000)
/* FSEL29 - Function Select 29 bits (RW) */
#define GPIO_GPFSEL2_FSEL29_OFS (27)
#define GPIO_GPFSEL2_FSEL29_MASK ((uint32_t) 0x38000000)
#define GPIO_GPFSEL2_FSEL290 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL2_FSEL291 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL2_FSEL292 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL2_FSEL29_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL29_1 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL2_FSEL29_2 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL2_FSEL29_3 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL2_FSEL29_4 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL2_FSEL29_5 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL2_FSEL29_6 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL2_FSEL29_7 ((uint32_t) 0x38000000)
#define GPIO_GPFSEL2_FSEL29__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL2_FSEL29__OUTPUT ((uint32_t) 0x08000000)
#define GPIO_GPFSEL2_FSEL29__ALT5 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL2_FSEL29__ALT4 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL2_FSEL29__ALT0 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL2_FSEL29__ALT1 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL2_FSEL29__ALT2 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL2_FSEL29__ALT3 ((uint32_t) 0x30000000)
/* GPIO Function Select 3 Register (RW) */
/* FSEL30 - Function Select 30 bits (RW) */
#define GPIO_GPFSEL3_FSEL30_OFS (0)
#define GPIO_GPFSEL3_FSEL30_MASK ((uint32_t) 0x00000007)
#define GPIO_GPFSEL3_FSEL300 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL3_FSEL301 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL3_FSEL302 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL3_FSEL30_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL30_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL3_FSEL30_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL3_FSEL30_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL3_FSEL30_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL3_FSEL30_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL3_FSEL30_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL3_FSEL30_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL3_FSEL30__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL30__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL3_FSEL30__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL3_FSEL30__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL3_FSEL30__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL3_FSEL30__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL3_FSEL30__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL3_FSEL30__ALT3 ((uint32_t) 0x00000007)
/* FSEL31 - Function Select 31 bits (RW) */
#define GPIO_GPFSEL3_FSEL31_OFS (3)
#define GPIO_GPFSEL3_FSEL31_MASK ((uint32_t) 0x00000038)
#define GPIO_GPFSEL3_FSEL310 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL3_FSEL311 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL3_FSEL312 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL3_FSEL31_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL31_1 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL3_FSEL31_2 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL3_FSEL31_3 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL3_FSEL31_4 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL3_FSEL31_5 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL3_FSEL31_6 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL3_FSEL31_7 ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL3_FSEL31__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL31__OUTPUT ((uint32_t) 0x00000040)
#define GPIO_GPFSEL3_FSEL31__ALT5 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL3_FSEL31__ALT4 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL3_FSEL31__ALT0 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL3_FSEL31__ALT1 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL3_FSEL31__ALT2 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL3_FSEL31__ALT3 ((uint32_t) 0x000001C0)
/* FSEL32 - Function Select 32 bits (RW) */
#define GPIO_GPFSEL3_FSEL32_OFS (6)
#define GPIO_GPFSEL3_FSEL32_MASK ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL3_FSEL320 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL3_FSEL321 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL3_FSEL322 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL3_FSEL32_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL32_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL3_FSEL32_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL3_FSEL32_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL3_FSEL32_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL3_FSEL32_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL3_FSEL32_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL3_FSEL32_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL3_FSEL32__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL32__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL3_FSEL32__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL3_FSEL32__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL3_FSEL32__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL3_FSEL32__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL3_FSEL32__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL3_FSEL32__ALT3 ((uint32_t) 0x00000007)
/* FSEL33 - Function Select 33 bits (RW) */
#define GPIO_GPFSEL3_FSEL33_OFS (9)
#define GPIO_GPFSEL3_FSEL33_MASK ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL3_FSEL330 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL3_FSEL331 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL3_FSEL332 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL3_FSEL33_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL33_1 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL3_FSEL33_2 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL3_FSEL33_3 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL3_FSEL33_4 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL3_FSEL33_5 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL3_FSEL33_6 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL3_FSEL33_7 ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL3_FSEL33__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL33__OUTPUT ((uint32_t) 0x0000200)
#define GPIO_GPFSEL3_FSEL33__ALT5 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL3_FSEL33__ALT4 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL3_FSEL33__ALT0 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL3_FSEL33__ALT1 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL3_FSEL33__ALT2 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL3_FSEL33__ALT3 ((uint32_t) 0x00000E00)
/* FSEL34 - Function Select 34 bits (RW) */
#define GPIO_GPFSEL3_FSEL34_OFS (12)
#define GPIO_GPFSEL3_FSEL34_MASK ((uint32_t) 0x00007000)
#define GPIO_GPFSEL3_FSEL340 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL3_FSEL341 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL3_FSEL342 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL3_FSEL34_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL34_1 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL3_FSEL34_2 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL3_FSEL34_3 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL3_FSEL34_4 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL3_FSEL34_5 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL3_FSEL34_6 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL3_FSEL34_7 ((uint32_t) 0x00007000)
#define GPIO_GPFSEL3_FSEL34__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL34__OUTPUT ((uint32_t) 0x00001000)
#define GPIO_GPFSEL3_FSEL34__ALT5 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL3_FSEL34__ALT4 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL3_FSEL34__ALT0 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL3_FSEL34__ALT1 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL3_FSEL34__ALT2 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL3_FSEL34__ALT3 ((uint32_t) 0x00007000)
/* FSEL35 - Function Select 35 bits (RW) */
#define GPIO_GPFSEL3_FSEL35_OFS (15)
#define GPIO_GPFSEL3_FSEL35_MASK ((uint32_t) 0x00038000)
#define GPIO_GPFSEL3_FSEL350 ((uint32_t) 0x00008000)
#define GPIO_GPFSEL3_FSEL351 ((uint32_t) 0x00010000)
#define GPIO_GPFSEL3_FSEL352 ((uint32_t) 0x00020000)
#define GPIO_GPFSEL3_FSEL35_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL35_1 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL3_FSEL35_2 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL3_FSEL35_3 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL3_FSEL35_4 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL3_FSEL35_5 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL3_FSEL35_6 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL3_FSEL35_7 ((uint32_t) 0x00380000)
#define GPIO_GPFSEL3_FSEL35__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL35__OUTPUT ((uint32_t) 0x00080000)
#define GPIO_GPFSEL3_FSEL35__ALT5 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL3_FSEL35__ALT4 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL3_FSEL35__ALT0 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL3_FSEL35__ALT1 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL3_FSEL35__ALT2 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL3_FSEL35__ALT3 ((uint32_t) 0x00380000)
/* FSEL36 - Function Select 36 bits (RW) */
#define GPIO_GPFSEL3_FSEL36_OFS (18)
#define GPIO_GPFSEL3_FSEL36_MASK ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL3_FSEL360 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL3_FSEL361 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL3_FSEL362 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL3_FSEL36_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL36_1 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL3_FSEL36_2 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL3_FSEL36_3 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL3_FSEL36_4 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL3_FSEL36_5 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL3_FSEL36_6 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL3_FSEL36_7 ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL3_FSEL36__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL36__OUTPUT ((uint32_t) 0x00040000)
#define GPIO_GPFSEL3_FSEL36__ALT5 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL3_FSEL36__ALT4 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL3_FSEL36__ALT0 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL3_FSEL36__ALT1 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL3_FSEL36__ALT2 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL3_FSEL36__ALT3 ((uint32_t) 0x001C0000)
/* FSEL37 - Function Select 37 bits (RW) */
#define GPIO_GPFSEL3_FSEL37_OFS (21)
#define GPIO_GPFSEL3_FSEL37_MASK ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL3_FSEL370 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL3_FSEL371 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL3_FSEL372 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL3_FSEL37_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL37_1 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL3_FSEL37_2 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL3_FSEL37_3 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL3_FSEL37_4 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL3_FSEL37_5 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL3_FSEL37_6 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL3_FSEL37_7 ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL3_FSEL37__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL37__OUTPUT ((uint32_t) 0x00200000)
#define GPIO_GPFSEL3_FSEL37__ALT5 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL3_FSEL37__ALT4 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL3_FSEL37__ALT0 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL3_FSEL37__ALT1 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL3_FSEL37__ALT2 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL3_FSEL37__ALT3 ((uint32_t) 0x00E00000)
/* FSEL38 - Function Select 38 bits (RW) */
#define GPIO_GPFSEL3_FSEL38_OFS (24)
#define GPIO_GPFSEL3_FSEL38_MASK ((uint32_t) 0x07000000)
#define GPIO_GPFSEL3_FSEL380 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL3_FSEL381 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL3_FSEL382 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL3_FSEL38_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL38_1 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL3_FSEL38_2 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL3_FSEL38_3 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL3_FSEL38_4 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL3_FSEL38_5 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL3_FSEL38_6 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL3_FSEL38_7 ((uint32_t) 0x07000000)
#define GPIO_GPFSEL3_FSEL38__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL38__OUTPUT ((uint32_t) 0x01000000)
#define GPIO_GPFSEL3_FSEL38__ALT5 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL3_FSEL38__ALT4 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL3_FSEL38__ALT0 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL3_FSEL38__ALT1 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL3_FSEL38__ALT2 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL3_FSEL38__ALT3 ((uint32_t) 0x07000000)
/* FSEL39 - Function Select 39 bits (RW) */
#define GPIO_GPFSEL3_FSEL39_OFS (27)
#define GPIO_GPFSEL3_FSEL39_MASK ((uint32_t) 0x38000000)
#define GPIO_GPFSEL3_FSEL390 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL3_FSEL391 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL3_FSEL392 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL3_FSEL39_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL39_1 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL3_FSEL39_2 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL3_FSEL39_3 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL3_FSEL39_4 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL3_FSEL39_5 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL3_FSEL39_6 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL3_FSEL39_7 ((uint32_t) 0x38000000)
#define GPIO_GPFSEL3_FSEL39__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL3_FSEL39__OUTPUT ((uint32_t) 0x08000000)
#define GPIO_GPFSEL3_FSEL39__ALT5 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL3_FSEL39__ALT4 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL3_FSEL39__ALT0 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL3_FSEL39__ALT1 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL3_FSEL39__ALT2 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL3_FSEL39__ALT3 ((uint32_t) 0x30000000)
/* GPIO Function Select 4 Register (RW) */
/* FSEL40 - Function Select 40 bits (RW) */
#define GPIO_GPFSEL4_FSEL40_OFS (0)
#define GPIO_GPFSEL4_FSEL40_MASK ((uint32_t) 0x00000007)
#define GPIO_GPFSEL4_FSEL400 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL4_FSEL401 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL4_FSEL402 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL4_FSEL40_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL40_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL4_FSEL40_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL4_FSEL40_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL4_FSEL40_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL4_FSEL40_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL4_FSEL40_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL4_FSEL40_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL4_FSEL40__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL40__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL4_FSEL40__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL4_FSEL40__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL4_FSEL40__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL4_FSEL40__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL4_FSEL40__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL4_FSEL40__ALT3 ((uint32_t) 0x00000007)
/* FSEL41 - Function Select 41 bits (RW) */
#define GPIO_GPFSEL4_FSEL41_OFS (3)
#define GPIO_GPFSEL4_FSEL41_MASK ((uint32_t) 0x00000038)
#define GPIO_GPFSEL4_FSEL410 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL4_FSEL411 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL4_FSEL412 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL4_FSEL41_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL41_1 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL4_FSEL41_2 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL4_FSEL41_3 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL4_FSEL41_4 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL4_FSEL41_5 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL4_FSEL41_6 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL4_FSEL41_7 ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL4_FSEL41__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL41__OUTPUT ((uint32_t) 0x00000040)
#define GPIO_GPFSEL4_FSEL41__ALT5 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL4_FSEL41__ALT4 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL4_FSEL41__ALT0 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL4_FSEL41__ALT1 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL4_FSEL41__ALT2 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL4_FSEL41__ALT3 ((uint32_t) 0x000001C0)
/* FSEL42 - Function Select 42 bits (RW) */
#define GPIO_GPFSEL4_FSEL42_OFS (6)
#define GPIO_GPFSEL4_FSEL42_MASK ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL4_FSEL420 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL4_FSEL421 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL4_FSEL422 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL4_FSEL42_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL42_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL4_FSEL42_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL4_FSEL42_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL4_FSEL42_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL4_FSEL42_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL4_FSEL42_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL4_FSEL42_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL4_FSEL42__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL42__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL4_FSEL42__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL4_FSEL42__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL4_FSEL42__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL4_FSEL42__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL4_FSEL42__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL4_FSEL42__ALT3 ((uint32_t) 0x00000007)
/* FSEL43 - Function Select 43 bits (RW) */
#define GPIO_GPFSEL4_FSEL43_OFS (9)
#define GPIO_GPFSEL4_FSEL43_MASK ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL4_FSEL430 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL4_FSEL431 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL4_FSEL432 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL4_FSEL43_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL43_1 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL4_FSEL43_2 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL4_FSEL43_3 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL4_FSEL43_4 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL4_FSEL43_5 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL4_FSEL43_6 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL4_FSEL43_7 ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL4_FSEL43__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL43__OUTPUT ((uint32_t) 0x0000200)
#define GPIO_GPFSEL4_FSEL43__ALT5 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL4_FSEL43__ALT4 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL4_FSEL43__ALT0 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL4_FSEL43__ALT1 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL4_FSEL43__ALT2 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL4_FSEL43__ALT3 ((uint32_t) 0x00000E00)
/* FSEL44 - Function Select 44 bits (RW) */
#define GPIO_GPFSEL4_FSEL44_OFS (12)
#define GPIO_GPFSEL4_FSEL44_MASK ((uint32_t) 0x00007000)
#define GPIO_GPFSEL4_FSEL440 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL4_FSEL441 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL4_FSEL442 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL4_FSEL44_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL44_1 ((uint32_t) 0x00001000)
#define GPIO_GPFSEL4_FSEL44_2 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL4_FSEL44_3 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL4_FSEL44_4 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL4_FSEL44_5 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL4_FSEL44_6 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL4_FSEL44_7 ((uint32_t) 0x00007000)
#define GPIO_GPFSEL4_FSEL44__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL44__OUTPUT ((uint32_t) 0x00001000)
#define GPIO_GPFSEL4_FSEL44__ALT5 ((uint32_t) 0x00002000)
#define GPIO_GPFSEL4_FSEL44__ALT4 ((uint32_t) 0x00003000)
#define GPIO_GPFSEL4_FSEL44__ALT0 ((uint32_t) 0x00004000)
#define GPIO_GPFSEL4_FSEL44__ALT1 ((uint32_t) 0x00005000)
#define GPIO_GPFSEL4_FSEL44__ALT2 ((uint32_t) 0x00006000)
#define GPIO_GPFSEL4_FSEL44__ALT3 ((uint32_t) 0x00007000)
/* FSEL45 - Function Select 45 bits (RW) */
#define GPIO_GPFSEL4_FSEL45_OFS (15)
#define GPIO_GPFSEL4_FSEL45_MASK ((uint32_t) 0x00038000)
#define GPIO_GPFSEL4_FSEL450 ((uint32_t) 0x00008000)
#define GPIO_GPFSEL4_FSEL451 ((uint32_t) 0x00010000)
#define GPIO_GPFSEL4_FSEL452 ((uint32_t) 0x00020000)
#define GPIO_GPFSEL4_FSEL45_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL45_1 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL4_FSEL45_2 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL4_FSEL45_3 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL4_FSEL45_4 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL4_FSEL45_5 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL4_FSEL45_6 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL4_FSEL45_7 ((uint32_t) 0x00380000)
#define GPIO_GPFSEL4_FSEL45__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL45__OUTPUT ((uint32_t) 0x00080000)
#define GPIO_GPFSEL4_FSEL45__ALT5 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL4_FSEL45__ALT4 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL4_FSEL45__ALT0 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL4_FSEL45__ALT1 ((uint32_t) 0x00280000)
#define GPIO_GPFSEL4_FSEL45__ALT2 ((uint32_t) 0x00300000)
#define GPIO_GPFSEL4_FSEL45__ALT3 ((uint32_t) 0x00380000)
/* FSEL46 - Function Select 46 bits (RW) */
#define GPIO_GPFSEL4_FSEL46_OFS (18)
#define GPIO_GPFSEL4_FSEL46_MASK ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL4_FSEL460 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL4_FSEL461 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL4_FSEL462 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL4_FSEL46_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL46_1 ((uint32_t) 0x00040000)
#define GPIO_GPFSEL4_FSEL46_2 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL4_FSEL46_3 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL4_FSEL46_4 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL4_FSEL46_5 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL4_FSEL46_6 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL4_FSEL46_7 ((uint32_t) 0x001C0000)
#define GPIO_GPFSEL4_FSEL46__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL46__OUTPUT ((uint32_t) 0x00040000)
#define GPIO_GPFSEL4_FSEL46__ALT5 ((uint32_t) 0x00080000)
#define GPIO_GPFSEL4_FSEL46__ALT4 ((uint32_t) 0x000C0000)
#define GPIO_GPFSEL4_FSEL46__ALT0 ((uint32_t) 0x00100000)
#define GPIO_GPFSEL4_FSEL46__ALT1 ((uint32_t) 0x00140000)
#define GPIO_GPFSEL4_FSEL46__ALT2 ((uint32_t) 0x00180000)
#define GPIO_GPFSEL4_FSEL46__ALT3 ((uint32_t) 0x001C0000)
/* FSEL47 - Function Select 47 bits (RW) */
#define GPIO_GPFSEL4_FSEL47_OFS (21)
#define GPIO_GPFSEL4_FSEL47_MASK ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL4_FSEL470 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL4_FSEL471 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL4_FSEL472 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL4_FSEL47_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL47_1 ((uint32_t) 0x00200000)
#define GPIO_GPFSEL4_FSEL47_2 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL4_FSEL47_3 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL4_FSEL47_4 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL4_FSEL47_5 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL4_FSEL47_6 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL4_FSEL47_7 ((uint32_t) 0x00E00000)
#define GPIO_GPFSEL4_FSEL47__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL47__OUTPUT ((uint32_t) 0x00200000)
#define GPIO_GPFSEL4_FSEL47__ALT5 ((uint32_t) 0x00400000)
#define GPIO_GPFSEL4_FSEL47__ALT4 ((uint32_t) 0x00600000)
#define GPIO_GPFSEL4_FSEL47__ALT0 ((uint32_t) 0x00800000)
#define GPIO_GPFSEL4_FSEL47__ALT1 ((uint32_t) 0x00A00000)
#define GPIO_GPFSEL4_FSEL47__ALT2 ((uint32_t) 0x00C00000)
#define GPIO_GPFSEL4_FSEL47__ALT3 ((uint32_t) 0x00E00000)
/* FSEL48 - Function Select 48 bits (RW) */
#define GPIO_GPFSEL4_FSEL48_OFS (24)
#define GPIO_GPFSEL4_FSEL48_MASK ((uint32_t) 0x07000000)
#define GPIO_GPFSEL4_FSEL480 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL4_FSEL481 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL4_FSEL482 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL4_FSEL48_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL48_1 ((uint32_t) 0x01000000)
#define GPIO_GPFSEL4_FSEL48_2 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL4_FSEL48_3 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL4_FSEL48_4 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL4_FSEL48_5 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL4_FSEL48_6 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL4_FSEL48_7 ((uint32_t) 0x07000000)
#define GPIO_GPFSEL4_FSEL48__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL48__OUTPUT ((uint32_t) 0x01000000)
#define GPIO_GPFSEL4_FSEL48__ALT5 ((uint32_t) 0x02000000)
#define GPIO_GPFSEL4_FSEL48__ALT4 ((uint32_t) 0x03000000)
#define GPIO_GPFSEL4_FSEL48__ALT0 ((uint32_t) 0x04000000)
#define GPIO_GPFSEL4_FSEL48__ALT1 ((uint32_t) 0x05000000)
#define GPIO_GPFSEL4_FSEL48__ALT2 ((uint32_t) 0x06000000)
#define GPIO_GPFSEL4_FSEL48__ALT3 ((uint32_t) 0x07000000)
/* FSEL49 - Function Select 49 bits (RW) */
#define GPIO_GPFSEL4_FSEL49_OFS (27)
#define GPIO_GPFSEL4_FSEL49_MASK ((uint32_t) 0x38000000)
#define GPIO_GPFSEL4_FSEL490 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL4_FSEL491 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL4_FSEL492 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL4_FSEL49_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL49_1 ((uint32_t) 0x08000000)
#define GPIO_GPFSEL4_FSEL49_2 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL4_FSEL49_3 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL4_FSEL49_4 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL4_FSEL49_5 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL4_FSEL49_6 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL4_FSEL49_7 ((uint32_t) 0x38000000)
#define GPIO_GPFSEL4_FSEL49__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL4_FSEL49__OUTPUT ((uint32_t) 0x08000000)
#define GPIO_GPFSEL4_FSEL49__ALT5 ((uint32_t) 0x10000000)
#define GPIO_GPFSEL4_FSEL49__ALT4 ((uint32_t) 0x18000000)
#define GPIO_GPFSEL4_FSEL49__ALT0 ((uint32_t) 0x20000000)
#define GPIO_GPFSEL4_FSEL49__ALT1 ((uint32_t) 0x28000000)
#define GPIO_GPFSEL4_FSEL49__ALT2 ((uint32_t) 0x30000000)
#define GPIO_GPFSEL4_FSEL49__ALT3 ((uint32_t) 0x30000000)
/* GPIO Function Select 5 Register (RW) */
/* FSEL50 - Function Select 50 bits (RW) */
#define GPIO_GPFSEL5_FSEL50_OFS (0)
#define GPIO_GPFSEL5_FSEL50_MASK ((uint32_t) 0x00000007)
#define GPIO_GPFSEL5_FSEL500 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL5_FSEL501 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL5_FSEL502 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL5_FSEL50_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL5_FSEL50_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL5_FSEL50_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL5_FSEL50_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL5_FSEL50_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL5_FSEL50_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL5_FSEL50_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL5_FSEL50_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL5_FSEL50__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL5_FSEL50__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL5_FSEL50__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL5_FSEL50__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL5_FSEL50__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL5_FSEL50__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL5_FSEL50__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL5_FSEL50__ALT3 ((uint32_t) 0x00000007)
/* FSEL51 - Function Select 51 bits (RW) */
#define GPIO_GPFSEL5_FSEL51_OFS (3)
#define GPIO_GPFSEL5_FSEL51_MASK ((uint32_t) 0x00000038)
#define GPIO_GPFSEL5_FSEL510 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL5_FSEL511 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL5_FSEL512 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL5_FSEL51_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL5_FSEL51_1 ((uint32_t) 0x00000040)
#define GPIO_GPFSEL5_FSEL51_2 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL5_FSEL51_3 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL5_FSEL51_4 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL5_FSEL51_5 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL5_FSEL51_6 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL5_FSEL51_7 ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL5_FSEL51__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL5_FSEL51__OUTPUT ((uint32_t) 0x00000040)
#define GPIO_GPFSEL5_FSEL51__ALT5 ((uint32_t) 0x00000080)
#define GPIO_GPFSEL5_FSEL51__ALT4 ((uint32_t) 0x000000C0)
#define GPIO_GPFSEL5_FSEL51__ALT0 ((uint32_t) 0x00000100)
#define GPIO_GPFSEL5_FSEL51__ALT1 ((uint32_t) 0x00000140)
#define GPIO_GPFSEL5_FSEL51__ALT2 ((uint32_t) 0x00000180)
#define GPIO_GPFSEL5_FSEL51__ALT3 ((uint32_t) 0x000001C0)
/* FSEL52 - Function Select 52 bits (RW) */
#define GPIO_GPFSEL5_FSEL52_OFS (6)
#define GPIO_GPFSEL5_FSEL52_MASK ((uint32_t) 0x000001C0)
#define GPIO_GPFSEL5_FSEL520 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL5_FSEL521 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL5_FSEL522 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL5_FSEL52_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL5_FSEL52_1 ((uint32_t) 0x00000001)
#define GPIO_GPFSEL5_FSEL52_2 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL5_FSEL52_3 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL5_FSEL52_4 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL5_FSEL52_5 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL5_FSEL52_6 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL5_FSEL52_7 ((uint32_t) 0x00000007)
#define GPIO_GPFSEL5_FSEL52__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL5_FSEL52__OUTPUT ((uint32_t) 0x00000001)
#define GPIO_GPFSEL5_FSEL52__ALT5 ((uint32_t) 0x00000002)
#define GPIO_GPFSEL5_FSEL52__ALT4 ((uint32_t) 0x00000003)
#define GPIO_GPFSEL5_FSEL52__ALT0 ((uint32_t) 0x00000004)
#define GPIO_GPFSEL5_FSEL52__ALT1 ((uint32_t) 0x00000005)
#define GPIO_GPFSEL5_FSEL52__ALT2 ((uint32_t) 0x00000006)
#define GPIO_GPFSEL5_FSEL52__ALT3 ((uint32_t) 0x00000007)
/* FSEL53 - Function Select 53 bits (RW) */
#define GPIO_GPFSEL5_FSEL53_OFS (9)
#define GPIO_GPFSEL5_FSEL53_MASK ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL5_FSEL530 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL5_FSEL531 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL5_FSEL532 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL5_FSEL53_0 ((uint32_t) 0x00000000)
#define GPIO_GPFSEL5_FSEL53_1 ((uint32_t) 0x00000200)
#define GPIO_GPFSEL5_FSEL53_2 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL5_FSEL53_3 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL5_FSEL53_4 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL5_FSEL53_5 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL5_FSEL53_6 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL5_FSEL53_7 ((uint32_t) 0x00000E00)
#define GPIO_GPFSEL5_FSEL53__INPUT ((uint32_t) 0x00000000)
#define GPIO_GPFSEL5_FSEL53__OUTPUT ((uint32_t) 0x0000200)
#define GPIO_GPFSEL5_FSEL53__ALT5 ((uint32_t) 0x00000400)
#define GPIO_GPFSEL5_FSEL53__ALT4 ((uint32_t) 0x00000600)
#define GPIO_GPFSEL5_FSEL53__ALT0 ((uint32_t) 0x00000800)
#define GPIO_GPFSEL5_FSEL53__ALT1 ((uint32_t) 0x00000A00)
#define GPIO_GPFSEL5_FSEL53__ALT2 ((uint32_t) 0x00000C00)
#define GPIO_GPFSEL5_FSEL53__ALT3 ((uint32_t) 0x00000E00)

/* GPIO Pin Output Set 0 Register (W) */
/* SETn (n=0..31) bits (W) */
#define GPIO_GPSET0_OFS (0)
#define GPIO_GPSET0_MASK ((uint32_t) 0xFFFFFFFF)
/* Set GPIO pin 0 (W) */
#define GPIO_GPSET00 ((uint32_t) 0x00000001)
#define GPIO_GPSET01 ((uint32_t) 0x00000002)
#define GPIO_GPSET02 ((uint32_t) 0x00000004)
#define GPIO_GPSET03 ((uint32_t) 0x00000008)
#define GPIO_GPSET04 ((uint32_t) 0x00000010)
#define GPIO_GPSET05 ((uint32_t) 0x00000020)
#define GPIO_GPSET06 ((uint32_t) 0x00000040)
#define GPIO_GPSET07 ((uint32_t) 0x00000080)
#define GPIO_GPSET08 ((uint32_t) 0x00000100)
#define GPIO_GPSET09 ((uint32_t) 0x00000200)
#define GPIO_GPSET010 ((uint32_t) 0x00000400)
#define GPIO_GPSET011 ((uint32_t) 0x00000800)
#define GPIO_GPSET012 ((uint32_t) 0x00001000)
#define GPIO_GPSET013 ((uint32_t) 0x00002000)
#define GPIO_GPSET014 ((uint32_t) 0x00004000)
#define GPIO_GPSET015 ((uint32_t) 0x00008000)
#define GPIO_GPSET016 ((uint32_t) 0x00010000)
#define GPIO_GPSET017 ((uint32_t) 0x00020000)
#define GPIO_GPSET018 ((uint32_t) 0x00040000)
#define GPIO_GPSET019 ((uint32_t) 0x00080000)
#define GPIO_GPSET020 ((uint32_t) 0x00100000)
#define GPIO_GPSET021 ((uint32_t) 0x00200000)
#define GPIO_GPSET022 ((uint32_t) 0x00400000)
#define GPIO_GPSET023 ((uint32_t) 0x00800000)
#define GPIO_GPSET024 ((uint32_t) 0x01000000)
#define GPIO_GPSET025 ((uint32_t) 0x02000000)
#define GPIO_GPSET026 ((uint32_t) 0x04000000)
#define GPIO_GPSET027 ((uint32_t) 0x08000000)
#define GPIO_GPSET028 ((uint32_t) 0x10000000)
#define GPIO_GPSET029 ((uint32_t) 0x20000000)
#define GPIO_GPSET030 ((uint32_t) 0x40000000)
#define GPIO_GPSET031 ((uint32_t) 0x80000000)

/* GPIO Pin Output Set 1 Register (W) */
/* SETn (n=0..31) bits (W) */
#define GPIO_GPSET1_OFS (0)
#define GPIO_GPSET1_MASK ((uint32_t) 0xFFFFFFFF)
/* Set GPIO pin 0 (W) */
#define GPIO_GPSET132 ((uint32_t) 0x00000001)
#define GPIO_GPSET133 ((uint32_t) 0x00000002)
#define GPIO_GPSET134 ((uint32_t) 0x00000004)
#define GPIO_GPSET135 ((uint32_t) 0x00000008)
#define GPIO_GPSET136 ((uint32_t) 0x00000010)
#define GPIO_GPSET137 ((uint32_t) 0x00000020)
#define GPIO_GPSET138 ((uint32_t) 0x00000040)
#define GPIO_GPSET139 ((uint32_t) 0x00000080)
#define GPIO_GPSET140 ((uint32_t) 0x00000100)
#define GPIO_GPSET141 ((uint32_t) 0x00000200)
#define GPIO_GPSET142 ((uint32_t) 0x00000400)
#define GPIO_GPSET143 ((uint32_t) 0x00000800)
#define GPIO_GPSET144 ((uint32_t) 0x00001000)
#define GPIO_GPSET145 ((uint32_t) 0x00002000)
#define GPIO_GPSET146 ((uint32_t) 0x00004000)
#define GPIO_GPSET147 ((uint32_t) 0x00008000)
#define GPIO_GPSET148 ((uint32_t) 0x00010000)
#define GPIO_GPSET149 ((uint32_t) 0x00020000)
#define GPIO_GPSET150 ((uint32_t) 0x00040000)
#define GPIO_GPSET151 ((uint32_t) 0x00080000)
#define GPIO_GPSET152 ((uint32_t) 0x00100000)
#define GPIO_GPSET153 ((uint32_t) 0x00200000)

/* GPIO Pin Output Clear 0 Register (W) */
/* CLRn (n=0..31) bits (W) */
#define GPIO_GPCLR0_OFS (0)
#define GPIO_GPCLR0_MASK ((uint32_t) 0xFFFFFFFF)
/* Clear GPIO pin 0 (W) */
#define GPIO_GPCLR00 ((uint32_t) 0x00000001)
#define GPIO_GPCLR01 ((uint32_t) 0x00000002)
#define GPIO_GPCLR02 ((uint32_t) 0x00000004)
#define GPIO_GPCLR03 ((uint32_t) 0x00000008)
#define GPIO_GPCLR04 ((uint32_t) 0x00000010)
#define GPIO_GPCLR05 ((uint32_t) 0x00000020)
#define GPIO_GPCLR06 ((uint32_t) 0x00000040)
#define GPIO_GPCLR07 ((uint32_t) 0x00000080)
#define GPIO_GPCLR08 ((uint32_t) 0x00000100)
#define GPIO_GPCLR09 ((uint32_t) 0x00000200)
#define GPIO_GPCLR010 ((uint32_t) 0x00000400)
#define GPIO_GPCLR011 ((uint32_t) 0x00000800)
#define GPIO_GPCLR012 ((uint32_t) 0x00001000)
#define GPIO_GPCLR013 ((uint32_t) 0x00002000)
#define GPIO_GPCLR014 ((uint32_t) 0x00004000)
#define GPIO_GPCLR015 ((uint32_t) 0x00008000)
#define GPIO_GPCLR016 ((uint32_t) 0x00010000)
#define GPIO_GPCLR017 ((uint32_t) 0x00020000)
#define GPIO_GPCLR018 ((uint32_t) 0x00040000)
#define GPIO_GPCLR019 ((uint32_t) 0x00080000)
#define GPIO_GPCLR020 ((uint32_t) 0x00100000)
#define GPIO_GPCLR021 ((uint32_t) 0x00200000)
#define GPIO_GPCLR022 ((uint32_t) 0x00400000)
#define GPIO_GPCLR023 ((uint32_t) 0x00800000)
#define GPIO_GPCLR024 ((uint32_t) 0x01000000)
#define GPIO_GPCLR025 ((uint32_t) 0x02000000)
#define GPIO_GPCLR026 ((uint32_t) 0x04000000)
#define GPIO_GPCLR027 ((uint32_t) 0x08000000)
#define GPIO_GPCLR028 ((uint32_t) 0x10000000)
#define GPIO_GPCLR029 ((uint32_t) 0x20000000)
#define GPIO_GPCLR030 ((uint32_t) 0x40000000)
#define GPIO_GPCLR031 ((uint32_t) 0x80000000)

/* GPIO Pin Output Clear 1 Register (W) */
/* CLRn (n=0..31) bits (W) */
#define GPIO_GPCLR1_OFS (0)
#define GPIO_GPCLR1_MASK ((uint32_t) 0xFFFFFFFF)
/* Clear GPIO pin 0 (W) */
#define GPIO_GPCLR132 ((uint32_t) 0x00000001)
#define GPIO_GPCLR133 ((uint32_t) 0x00000002)
#define GPIO_GPCLR134 ((uint32_t) 0x00000004)
#define GPIO_GPCLR135 ((uint32_t) 0x00000008)
#define GPIO_GPCLR136 ((uint32_t) 0x00000010)
#define GPIO_GPCLR137 ((uint32_t) 0x00000020)
#define GPIO_GPCLR138 ((uint32_t) 0x00000040)
#define GPIO_GPCLR139 ((uint32_t) 0x00000080)
#define GPIO_GPCLR140 ((uint32_t) 0x00000100)
#define GPIO_GPCLR141 ((uint32_t) 0x00000200)
#define GPIO_GPCLR142 ((uint32_t) 0x00000400)
#define GPIO_GPCLR143 ((uint32_t) 0x00000800)
#define GPIO_GPCLR144 ((uint32_t) 0x00001000)
#define GPIO_GPCLR145 ((uint32_t) 0x00002000)
#define GPIO_GPCLR146 ((uint32_t) 0x00004000)
#define GPIO_GPCLR147 ((uint32_t) 0x00008000)
#define GPIO_GPCLR148 ((uint32_t) 0x00010000)
#define GPIO_GPCLR149 ((uint32_t) 0x00020000)
#define GPIO_GPCLR150 ((uint32_t) 0x00040000)
#define GPIO_GPCLR151 ((uint32_t) 0x00080000)
#define GPIO_GPCLR152 ((uint32_t) 0x00100000)
#define GPIO_GPCLR153 ((uint32_t) 0x00200000)

/* GPIO Pin Level 0 Register (R) */
/* Read value of the GPIO pin 0..31 */
#define GPIO_GPLEV0_OFS (0)
#define GPIO_GPLEV0_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPLEV00 ((uint32_t) 0x00000001)
#define GPIO_GPLEV01 ((uint32_t) 0x00000002)
#define GPIO_GPLEV02 ((uint32_t) 0x00000004)
#define GPIO_GPLEV03 ((uint32_t) 0x00000008)
#define GPIO_GPLEV04 ((uint32_t) 0x00000010)
#define GPIO_GPLEV05 ((uint32_t) 0x00000020)
#define GPIO_GPLEV06 ((uint32_t) 0x00000040)
#define GPIO_GPLEV07 ((uint32_t) 0x00000080)
#define GPIO_GPLEV08 ((uint32_t) 0x00000100)
#define GPIO_GPLEV09 ((uint32_t) 0x00000200)
#define GPIO_GPLEV010 ((uint32_t) 0x00000400)
#define GPIO_GPLEV011 ((uint32_t) 0x00000800)
#define GPIO_GPLEV012 ((uint32_t) 0x00001000)
#define GPIO_GPLEV013 ((uint32_t) 0x00002000)
#define GPIO_GPLEV014 ((uint32_t) 0x00004000)
#define GPIO_GPLEV015 ((uint32_t) 0x00008000)
#define GPIO_GPLEV016 ((uint32_t) 0x00010000)
#define GPIO_GPLEV017 ((uint32_t) 0x00020000)
#define GPIO_GPLEV018 ((uint32_t) 0x00040000)
#define GPIO_GPLEV019 ((uint32_t) 0x00080000)
#define GPIO_GPLEV020 ((uint32_t) 0x00100000)
#define GPIO_GPLEV021 ((uint32_t) 0x00200000)
#define GPIO_GPLEV022 ((uint32_t) 0x00400000)
#define GPIO_GPLEV023 ((uint32_t) 0x00800000)
#define GPIO_GPLEV024 ((uint32_t) 0x01000000)
#define GPIO_GPLEV025 ((uint32_t) 0x02000000)
#define GPIO_GPLEV026 ((uint32_t) 0x04000000)
#define GPIO_GPLEV027 ((uint32_t) 0x08000000)
#define GPIO_GPLEV028 ((uint32_t) 0x10000000)
#define GPIO_GPLEV029 ((uint32_t) 0x20000000)
#define GPIO_GPLEV030 ((uint32_t) 0x40000000)
#define GPIO_GPLEV031 ((uint32_t) 0x80000000)

/* GPIO Pin Level 1 Register (R) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPLEV1_OFS (0)
#define GPIO_GPLEV1_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPLEV132 ((uint32_t) 0x00000001)
#define GPIO_GPLEV133 ((uint32_t) 0x00000002)
#define GPIO_GPLEV134 ((uint32_t) 0x00000004)
#define GPIO_GPLEV135 ((uint32_t) 0x00000008)
#define GPIO_GPLEV136 ((uint32_t) 0x00000010)
#define GPIO_GPLEV137 ((uint32_t) 0x00000020)
#define GPIO_GPLEV138 ((uint32_t) 0x00000040)
#define GPIO_GPLEV139 ((uint32_t) 0x00000080)
#define GPIO_GPLEV140 ((uint32_t) 0x00000100)
#define GPIO_GPLEV141 ((uint32_t) 0x00000200)
#define GPIO_GPLEV142 ((uint32_t) 0x00000400)
#define GPIO_GPLEV143 ((uint32_t) 0x00000800)
#define GPIO_GPLEV144 ((uint32_t) 0x00001000)
#define GPIO_GPLEV145 ((uint32_t) 0x00002000)
#define GPIO_GPLEV146 ((uint32_t) 0x00004000)
#define GPIO_GPLEV147 ((uint32_t) 0x00008000)
#define GPIO_GPLEV148 ((uint32_t) 0x00010000)
#define GPIO_GPLEV149 ((uint32_t) 0x00020000)
#define GPIO_GPLEV150 ((uint32_t) 0x00040000)
#define GPIO_GPLEV151 ((uint32_t) 0x00080000)
#define GPIO_GPLEV152 ((uint32_t) 0x00100000)
#define GPIO_GPLEV153 ((uint32_t) 0x00200000)

/* GPIO Pin Event Detect Status 0 Register (RW) */
#define GPIO_GPLEV0_OFS (0)
#define GPIO_GPLEV0_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPLEV00 ((uint32_t) 0x00000001)
#define GPIO_GPLEV01 ((uint32_t) 0x00000002)
#define GPIO_GPLEV02 ((uint32_t) 0x00000004)
#define GPIO_GPLEV03 ((uint32_t) 0x00000008)
#define GPIO_GPLEV04 ((uint32_t) 0x00000010)
#define GPIO_GPLEV05 ((uint32_t) 0x00000020)
#define GPIO_GPLEV06 ((uint32_t) 0x00000040)
#define GPIO_GPLEV07 ((uint32_t) 0x00000080)
#define GPIO_GPLEV08 ((uint32_t) 0x00000100)
#define GPIO_GPLEV09 ((uint32_t) 0x00000200)
#define GPIO_GPLEV010 ((uint32_t) 0x00000400)
#define GPIO_GPLEV011 ((uint32_t) 0x00000800)
#define GPIO_GPLEV012 ((uint32_t) 0x00001000)
#define GPIO_GPLEV013 ((uint32_t) 0x00002000)
#define GPIO_GPLEV014 ((uint32_t) 0x00004000)
#define GPIO_GPLEV015 ((uint32_t) 0x00008000)
#define GPIO_GPLEV016 ((uint32_t) 0x00010000)
#define GPIO_GPLEV017 ((uint32_t) 0x00020000)
#define GPIO_GPLEV018 ((uint32_t) 0x00040000)
#define GPIO_GPLEV019 ((uint32_t) 0x00080000)
#define GPIO_GPLEV020 ((uint32_t) 0x00100000)
#define GPIO_GPLEV021 ((uint32_t) 0x00200000)
#define GPIO_GPLEV022 ((uint32_t) 0x00400000)
#define GPIO_GPLEV023 ((uint32_t) 0x00800000)
#define GPIO_GPLEV024 ((uint32_t) 0x01000000)
#define GPIO_GPLEV025 ((uint32_t) 0x02000000)
#define GPIO_GPLEV026 ((uint32_t) 0x04000000)
#define GPIO_GPLEV027 ((uint32_t) 0x08000000)
#define GPIO_GPLEV028 ((uint32_t) 0x10000000)
#define GPIO_GPLEV029 ((uint32_t) 0x20000000)
#define GPIO_GPLEV030 ((uint32_t) 0x40000000)
#define GPIO_GPLEV031 ((uint32_t) 0x80000000)

/* GPIO Pin Event Detect Status 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPLEV1_OFS (0)
#define GPIO_GPLEV1_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPLEV132 ((uint32_t) 0x00000001)
#define GPIO_GPLEV133 ((uint32_t) 0x00000002)
#define GPIO_GPLEV134 ((uint32_t) 0x00000004)
#define GPIO_GPLEV135 ((uint32_t) 0x00000008)
#define GPIO_GPLEV136 ((uint32_t) 0x00000010)
#define GPIO_GPLEV137 ((uint32_t) 0x00000020)
#define GPIO_GPLEV138 ((uint32_t) 0x00000040)
#define GPIO_GPLEV139 ((uint32_t) 0x00000080)
#define GPIO_GPLEV140 ((uint32_t) 0x00000100)
#define GPIO_GPLEV141 ((uint32_t) 0x00000200)
#define GPIO_GPLEV142 ((uint32_t) 0x00000400)
#define GPIO_GPLEV143 ((uint32_t) 0x00000800)
#define GPIO_GPLEV144 ((uint32_t) 0x00001000)
#define GPIO_GPLEV145 ((uint32_t) 0x00002000)
#define GPIO_GPLEV146 ((uint32_t) 0x00004000)
#define GPIO_GPLEV147 ((uint32_t) 0x00008000)
#define GPIO_GPLEV148 ((uint32_t) 0x00010000)
#define GPIO_GPLEV149 ((uint32_t) 0x00020000)
#define GPIO_GPLEV150 ((uint32_t) 0x00040000)
#define GPIO_GPLEV151 ((uint32_t) 0x00080000)
#define GPIO_GPLEV152 ((uint32_t) 0x00100000)
#define GPIO_GPLEV153 ((uint32_t) 0x00200000)

/* GPIO Pin Rising Edge Detect Enable 0 Register (RW) */
#define GPIO_GPREN0_OFS (0)
#define GPIO_GPREN0_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPREN00 ((uint32_t) 0x00000001)
#define GPIO_GPREN01 ((uint32_t) 0x00000002)
#define GPIO_GPREN02 ((uint32_t) 0x00000004)
#define GPIO_GPREN03 ((uint32_t) 0x00000008)
#define GPIO_GPREN04 ((uint32_t) 0x00000010)
#define GPIO_GPREN05 ((uint32_t) 0x00000020)
#define GPIO_GPREN06 ((uint32_t) 0x00000040)
#define GPIO_GPREN07 ((uint32_t) 0x00000080)
#define GPIO_GPREN08 ((uint32_t) 0x00000100)
#define GPIO_GPREN09 ((uint32_t) 0x00000200)
#define GPIO_GPREN010 ((uint32_t) 0x00000400)
#define GPIO_GPREN011 ((uint32_t) 0x00000800)
#define GPIO_GPREN012 ((uint32_t) 0x00001000)
#define GPIO_GPREN013 ((uint32_t) 0x00002000)
#define GPIO_GPREN014 ((uint32_t) 0x00004000)
#define GPIO_GPREN015 ((uint32_t) 0x00008000)
#define GPIO_GPREN016 ((uint32_t) 0x00010000)
#define GPIO_GPREN017 ((uint32_t) 0x00020000)
#define GPIO_GPREN018 ((uint32_t) 0x00040000)
#define GPIO_GPREN019 ((uint32_t) 0x00080000)
#define GPIO_GPREN020 ((uint32_t) 0x00100000)
#define GPIO_GPREN021 ((uint32_t) 0x00200000)
#define GPIO_GPREN022 ((uint32_t) 0x00400000)
#define GPIO_GPREN023 ((uint32_t) 0x00800000)
#define GPIO_GPREN024 ((uint32_t) 0x01000000)
#define GPIO_GPREN025 ((uint32_t) 0x02000000)
#define GPIO_GPREN026 ((uint32_t) 0x04000000)
#define GPIO_GPREN027 ((uint32_t) 0x08000000)
#define GPIO_GPREN028 ((uint32_t) 0x10000000)
#define GPIO_GPREN029 ((uint32_t) 0x20000000)
#define GPIO_GPREN030 ((uint32_t) 0x40000000)
#define GPIO_GPREN031 ((uint32_t) 0x80000000)

/* GPIO Pin Rising Edge Detect Enable 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPREN1_OFS (0)
#define GPIO_GPREN1_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPREN132 ((uint32_t) 0x00000001)
#define GPIO_GPREN133 ((uint32_t) 0x00000002)
#define GPIO_GPREN134 ((uint32_t) 0x00000004)
#define GPIO_GPREN135 ((uint32_t) 0x00000008)
#define GPIO_GPREN136 ((uint32_t) 0x00000010)
#define GPIO_GPREN137 ((uint32_t) 0x00000020)
#define GPIO_GPREN138 ((uint32_t) 0x00000040)
#define GPIO_GPREN139 ((uint32_t) 0x00000080)
#define GPIO_GPREN140 ((uint32_t) 0x00000100)
#define GPIO_GPREN141 ((uint32_t) 0x00000200)
#define GPIO_GPREN142 ((uint32_t) 0x00000400)
#define GPIO_GPREN143 ((uint32_t) 0x00000800)
#define GPIO_GPREN144 ((uint32_t) 0x00001000)
#define GPIO_GPREN145 ((uint32_t) 0x00002000)
#define GPIO_GPREN146 ((uint32_t) 0x00004000)
#define GPIO_GPREN147 ((uint32_t) 0x00008000)
#define GPIO_GPREN148 ((uint32_t) 0x00010000)
#define GPIO_GPREN149 ((uint32_t) 0x00020000)
#define GPIO_GPREN150 ((uint32_t) 0x00040000)
#define GPIO_GPREN151 ((uint32_t) 0x00080000)
#define GPIO_GPREN152 ((uint32_t) 0x00100000)
#define GPIO_GPREN153 ((uint32_t) 0x00200000)

/* GPIO Pin Falling Edge Detect Enable 0 Register (RW) */
#define GPIO_GPFEN0_OFS (0)
#define GPIO_GPFEN0_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPFEN00 ((uint32_t) 0x00000001)
#define GPIO_GPFEN01 ((uint32_t) 0x00000002)
#define GPIO_GPFEN02 ((uint32_t) 0x00000004)
#define GPIO_GPFEN03 ((uint32_t) 0x00000008)
#define GPIO_GPFEN04 ((uint32_t) 0x00000010)
#define GPIO_GPFEN05 ((uint32_t) 0x00000020)
#define GPIO_GPFEN06 ((uint32_t) 0x00000040)
#define GPIO_GPFEN07 ((uint32_t) 0x00000080)
#define GPIO_GPFEN08 ((uint32_t) 0x00000100)
#define GPIO_GPFEN09 ((uint32_t) 0x00000200)
#define GPIO_GPFEN010 ((uint32_t) 0x00000400)
#define GPIO_GPFEN011 ((uint32_t) 0x00000800)
#define GPIO_GPFEN012 ((uint32_t) 0x00001000)
#define GPIO_GPFEN013 ((uint32_t) 0x00002000)
#define GPIO_GPFEN014 ((uint32_t) 0x00004000)
#define GPIO_GPFEN015 ((uint32_t) 0x00008000)
#define GPIO_GPFEN016 ((uint32_t) 0x00010000)
#define GPIO_GPFEN017 ((uint32_t) 0x00020000)
#define GPIO_GPFEN018 ((uint32_t) 0x00040000)
#define GPIO_GPFEN019 ((uint32_t) 0x00080000)
#define GPIO_GPFEN020 ((uint32_t) 0x00100000)
#define GPIO_GPFEN021 ((uint32_t) 0x00200000)
#define GPIO_GPFEN022 ((uint32_t) 0x00400000)
#define GPIO_GPFEN023 ((uint32_t) 0x00800000)
#define GPIO_GPFEN024 ((uint32_t) 0x01000000)
#define GPIO_GPFEN025 ((uint32_t) 0x02000000)
#define GPIO_GPFEN026 ((uint32_t) 0x04000000)
#define GPIO_GPFEN027 ((uint32_t) 0x08000000)
#define GPIO_GPFEN028 ((uint32_t) 0x10000000)
#define GPIO_GPFEN029 ((uint32_t) 0x20000000)
#define GPIO_GPFEN030 ((uint32_t) 0x40000000)
#define GPIO_GPFEN031 ((uint32_t) 0x80000000)

/* GPIO Pin Falling Edge Detect Enable 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPFEN1_OFS (0)
#define GPIO_GPFEN1_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPFEN132 ((uint32_t) 0x00000001)
#define GPIO_GPFEN133 ((uint32_t) 0x00000002)
#define GPIO_GPFEN134 ((uint32_t) 0x00000004)
#define GPIO_GPFEN135 ((uint32_t) 0x00000008)
#define GPIO_GPFEN136 ((uint32_t) 0x00000010)
#define GPIO_GPFEN137 ((uint32_t) 0x00000020)
#define GPIO_GPFEN138 ((uint32_t) 0x00000040)
#define GPIO_GPFEN139 ((uint32_t) 0x00000080)
#define GPIO_GPFEN140 ((uint32_t) 0x00000100)
#define GPIO_GPFEN141 ((uint32_t) 0x00000200)
#define GPIO_GPFEN142 ((uint32_t) 0x00000400)
#define GPIO_GPFEN143 ((uint32_t) 0x00000800)
#define GPIO_GPFEN144 ((uint32_t) 0x00001000)
#define GPIO_GPFEN145 ((uint32_t) 0x00002000)
#define GPIO_GPFEN146 ((uint32_t) 0x00004000)
#define GPIO_GPFEN147 ((uint32_t) 0x00008000)
#define GPIO_GPFEN148 ((uint32_t) 0x00010000)
#define GPIO_GPFEN149 ((uint32_t) 0x00020000)
#define GPIO_GPFEN150 ((uint32_t) 0x00040000)
#define GPIO_GPFEN151 ((uint32_t) 0x00080000)
#define GPIO_GPFEN152 ((uint32_t) 0x00100000)
#define GPIO_GPFEN153 ((uint32_t) 0x00200000)

/* GPIO Pin High Detect Enable 0 Register (RW) */
#define GPIO_GPHEN0_OFS (0)
#define GPIO_GPHEN0_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPHEN00 ((uint32_t) 0x00000001)
#define GPIO_GPHEN01 ((uint32_t) 0x00000002)
#define GPIO_GPHEN02 ((uint32_t) 0x00000004)
#define GPIO_GPHEN03 ((uint32_t) 0x00000008)
#define GPIO_GPHEN04 ((uint32_t) 0x00000010)
#define GPIO_GPHEN05 ((uint32_t) 0x00000020)
#define GPIO_GPHEN06 ((uint32_t) 0x00000040)
#define GPIO_GPHEN07 ((uint32_t) 0x00000080)
#define GPIO_GPHEN08 ((uint32_t) 0x00000100)
#define GPIO_GPHEN09 ((uint32_t) 0x00000200)
#define GPIO_GPHEN010 ((uint32_t) 0x00000400)
#define GPIO_GPHEN011 ((uint32_t) 0x00000800)
#define GPIO_GPHEN012 ((uint32_t) 0x00001000)
#define GPIO_GPHEN013 ((uint32_t) 0x00002000)
#define GPIO_GPHEN014 ((uint32_t) 0x00004000)
#define GPIO_GPHEN015 ((uint32_t) 0x00008000)
#define GPIO_GPHEN016 ((uint32_t) 0x00010000)
#define GPIO_GPHEN017 ((uint32_t) 0x00020000)
#define GPIO_GPHEN018 ((uint32_t) 0x00040000)
#define GPIO_GPHEN019 ((uint32_t) 0x00080000)
#define GPIO_GPHEN020 ((uint32_t) 0x00100000)
#define GPIO_GPHEN021 ((uint32_t) 0x00200000)
#define GPIO_GPHEN022 ((uint32_t) 0x00400000)
#define GPIO_GPHEN023 ((uint32_t) 0x00800000)
#define GPIO_GPHEN024 ((uint32_t) 0x01000000)
#define GPIO_GPHEN025 ((uint32_t) 0x02000000)
#define GPIO_GPHEN026 ((uint32_t) 0x04000000)
#define GPIO_GPHEN027 ((uint32_t) 0x08000000)
#define GPIO_GPHEN028 ((uint32_t) 0x10000000)
#define GPIO_GPHEN029 ((uint32_t) 0x20000000)
#define GPIO_GPHEN030 ((uint32_t) 0x40000000)
#define GPIO_GPHEN031 ((uint32_t) 0x80000000)

/* GPIO Pin High Detect Enable 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPHEN1_OFS (0)
#define GPIO_GPHEN1_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPHEN132 ((uint32_t) 0x00000001)
#define GPIO_GPHEN133 ((uint32_t) 0x00000002)
#define GPIO_GPHEN134 ((uint32_t) 0x00000004)
#define GPIO_GPHEN135 ((uint32_t) 0x00000008)
#define GPIO_GPHEN136 ((uint32_t) 0x00000010)
#define GPIO_GPHEN137 ((uint32_t) 0x00000020)
#define GPIO_GPHEN138 ((uint32_t) 0x00000040)
#define GPIO_GPHEN139 ((uint32_t) 0x00000080)
#define GPIO_GPHEN140 ((uint32_t) 0x00000100)
#define GPIO_GPHEN141 ((uint32_t) 0x00000200)
#define GPIO_GPHEN142 ((uint32_t) 0x00000400)
#define GPIO_GPHEN143 ((uint32_t) 0x00000800)
#define GPIO_GPHEN144 ((uint32_t) 0x00001000)
#define GPIO_GPHEN145 ((uint32_t) 0x00002000)
#define GPIO_GPHEN146 ((uint32_t) 0x00004000)
#define GPIO_GPHEN147 ((uint32_t) 0x00008000)
#define GPIO_GPHEN148 ((uint32_t) 0x00010000)
#define GPIO_GPHEN149 ((uint32_t) 0x00020000)
#define GPIO_GPHEN150 ((uint32_t) 0x00040000)
#define GPIO_GPHEN151 ((uint32_t) 0x00080000)
#define GPIO_GPHEN152 ((uint32_t) 0x00100000)
#define GPIO_GPHEN153 ((uint32_t) 0x00200000)

/* GPIO Pin Low Detect Enable 0 Register (RW) */
#define GPIO_GPLEN0_OFS (0)
#define GPIO_GPLEN0_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPLEN00 ((uint32_t) 0x00000001)
#define GPIO_GPLEN01 ((uint32_t) 0x00000002)
#define GPIO_GPLEN02 ((uint32_t) 0x00000004)
#define GPIO_GPLEN03 ((uint32_t) 0x00000008)
#define GPIO_GPLEN04 ((uint32_t) 0x00000010)
#define GPIO_GPLEN05 ((uint32_t) 0x00000020)
#define GPIO_GPLEN06 ((uint32_t) 0x00000040)
#define GPIO_GPLEN07 ((uint32_t) 0x00000080)
#define GPIO_GPLEN08 ((uint32_t) 0x00000100)
#define GPIO_GPLEN09 ((uint32_t) 0x00000200)
#define GPIO_GPLEN010 ((uint32_t) 0x00000400)
#define GPIO_GPLEN011 ((uint32_t) 0x00000800)
#define GPIO_GPLEN012 ((uint32_t) 0x00001000)
#define GPIO_GPLEN013 ((uint32_t) 0x00002000)
#define GPIO_GPLEN014 ((uint32_t) 0x00004000)
#define GPIO_GPLEN015 ((uint32_t) 0x00008000)
#define GPIO_GPLEN016 ((uint32_t) 0x00010000)
#define GPIO_GPLEN017 ((uint32_t) 0x00020000)
#define GPIO_GPLEN018 ((uint32_t) 0x00040000)
#define GPIO_GPLEN019 ((uint32_t) 0x00080000)
#define GPIO_GPLEN020 ((uint32_t) 0x00100000)
#define GPIO_GPLEN021 ((uint32_t) 0x00200000)
#define GPIO_GPLEN022 ((uint32_t) 0x00400000)
#define GPIO_GPLEN023 ((uint32_t) 0x00800000)
#define GPIO_GPLEN024 ((uint32_t) 0x01000000)
#define GPIO_GPLEN025 ((uint32_t) 0x02000000)
#define GPIO_GPLEN026 ((uint32_t) 0x04000000)
#define GPIO_GPLEN027 ((uint32_t) 0x08000000)
#define GPIO_GPLEN028 ((uint32_t) 0x10000000)
#define GPIO_GPLEN029 ((uint32_t) 0x20000000)
#define GPIO_GPLEN030 ((uint32_t) 0x40000000)
#define GPIO_GPLEN031 ((uint32_t) 0x80000000)

/* GPIO Pin Low Detect Enable 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPLEN1_OFS (0)
#define GPIO_GPLEN1_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPLEN132 ((uint32_t) 0x00000001)
#define GPIO_GPLEN133 ((uint32_t) 0x00000002)
#define GPIO_GPLEN134 ((uint32_t) 0x00000004)
#define GPIO_GPLEN135 ((uint32_t) 0x00000008)
#define GPIO_GPLEN136 ((uint32_t) 0x00000010)
#define GPIO_GPLEN137 ((uint32_t) 0x00000020)
#define GPIO_GPLEN138 ((uint32_t) 0x00000040)
#define GPIO_GPLEN139 ((uint32_t) 0x00000080)
#define GPIO_GPLEN140 ((uint32_t) 0x00000100)
#define GPIO_GPLEN141 ((uint32_t) 0x00000200)
#define GPIO_GPLEN142 ((uint32_t) 0x00000400)
#define GPIO_GPLEN143 ((uint32_t) 0x00000800)
#define GPIO_GPLEN144 ((uint32_t) 0x00001000)
#define GPIO_GPLEN145 ((uint32_t) 0x00002000)
#define GPIO_GPLEN146 ((uint32_t) 0x00004000)
#define GPIO_GPLEN147 ((uint32_t) 0x00008000)
#define GPIO_GPLEN148 ((uint32_t) 0x00010000)
#define GPIO_GPLEN149 ((uint32_t) 0x00020000)
#define GPIO_GPLEN150 ((uint32_t) 0x00040000)
#define GPIO_GPLEN151 ((uint32_t) 0x00080000)
#define GPIO_GPLEN152 ((uint32_t) 0x00100000)
#define GPIO_GPLEN153 ((uint32_t) 0x00200000)

/* GPIO Pin Async. Rising Edge Detect 0 Register (RW) */
#define GPIO_GPAREN0_OFS (0)
#define GPIO_GPAREN0_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPAREN00 ((uint32_t) 0x00000001)
#define GPIO_GPAREN01 ((uint32_t) 0x00000002)
#define GPIO_GPAREN02 ((uint32_t) 0x00000004)
#define GPIO_GPAREN03 ((uint32_t) 0x00000008)
#define GPIO_GPAREN04 ((uint32_t) 0x00000010)
#define GPIO_GPAREN05 ((uint32_t) 0x00000020)
#define GPIO_GPAREN06 ((uint32_t) 0x00000040)
#define GPIO_GPAREN07 ((uint32_t) 0x00000080)
#define GPIO_GPAREN08 ((uint32_t) 0x00000100)
#define GPIO_GPAREN09 ((uint32_t) 0x00000200)
#define GPIO_GPAREN010 ((uint32_t) 0x00000400)
#define GPIO_GPAREN011 ((uint32_t) 0x00000800)
#define GPIO_GPAREN012 ((uint32_t) 0x00001000)
#define GPIO_GPAREN013 ((uint32_t) 0x00002000)
#define GPIO_GPAREN014 ((uint32_t) 0x00004000)
#define GPIO_GPAREN015 ((uint32_t) 0x00008000)
#define GPIO_GPAREN016 ((uint32_t) 0x00010000)
#define GPIO_GPAREN017 ((uint32_t) 0x00020000)
#define GPIO_GPAREN018 ((uint32_t) 0x00040000)
#define GPIO_GPAREN019 ((uint32_t) 0x00080000)
#define GPIO_GPAREN020 ((uint32_t) 0x00100000)
#define GPIO_GPAREN021 ((uint32_t) 0x00200000)
#define GPIO_GPAREN022 ((uint32_t) 0x00400000)
#define GPIO_GPAREN023 ((uint32_t) 0x00800000)
#define GPIO_GPAREN024 ((uint32_t) 0x01000000)
#define GPIO_GPAREN025 ((uint32_t) 0x02000000)
#define GPIO_GPAREN026 ((uint32_t) 0x04000000)
#define GPIO_GPAREN027 ((uint32_t) 0x08000000)
#define GPIO_GPAREN028 ((uint32_t) 0x10000000)
#define GPIO_GPAREN029 ((uint32_t) 0x20000000)
#define GPIO_GPAREN030 ((uint32_t) 0x40000000)
#define GPIO_GPAREN031 ((uint32_t) 0x80000000)

/* GPIO Pin Async. Rising Edge Detect 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPAREN1_OFS (0)
#define GPIO_GPAREN1_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPAREN132 ((uint32_t) 0x00000001)
#define GPIO_GPAREN133 ((uint32_t) 0x00000002)
#define GPIO_GPAREN134 ((uint32_t) 0x00000004)
#define GPIO_GPAREN135 ((uint32_t) 0x00000008)
#define GPIO_GPAREN136 ((uint32_t) 0x00000010)
#define GPIO_GPAREN137 ((uint32_t) 0x00000020)
#define GPIO_GPAREN138 ((uint32_t) 0x00000040)
#define GPIO_GPAREN139 ((uint32_t) 0x00000080)
#define GPIO_GPAREN140 ((uint32_t) 0x00000100)
#define GPIO_GPAREN141 ((uint32_t) 0x00000200)
#define GPIO_GPAREN142 ((uint32_t) 0x00000400)
#define GPIO_GPAREN143 ((uint32_t) 0x00000800)
#define GPIO_GPAREN144 ((uint32_t) 0x00001000)
#define GPIO_GPAREN145 ((uint32_t) 0x00002000)
#define GPIO_GPAREN146 ((uint32_t) 0x00004000)
#define GPIO_GPAREN147 ((uint32_t) 0x00008000)
#define GPIO_GPAREN148 ((uint32_t) 0x00010000)
#define GPIO_GPAREN149 ((uint32_t) 0x00020000)
#define GPIO_GPAREN150 ((uint32_t) 0x00040000)
#define GPIO_GPAREN151 ((uint32_t) 0x00080000)
#define GPIO_GPAREN152 ((uint32_t) 0x00100000)
#define GPIO_GPAREN153 ((uint32_t) 0x00200000)

/* GPIO Pin Async. Falling Edge Detect 0 Register (RW) */
#define GPIO_GPAFEN0_OFS (0)
#define GPIO_GPAFEN0_MASK ((uint32_t) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPAFEN00 ((uint32_t) 0x00000001)
#define GPIO_GPAFEN01 ((uint32_t) 0x00000002)
#define GPIO_GPAFEN02 ((uint32_t) 0x00000004)
#define GPIO_GPAFEN03 ((uint32_t) 0x00000008)
#define GPIO_GPAFEN04 ((uint32_t) 0x00000010)
#define GPIO_GPAFEN05 ((uint32_t) 0x00000020)
#define GPIO_GPAFEN06 ((uint32_t) 0x00000040)
#define GPIO_GPAFEN07 ((uint32_t) 0x00000080)
#define GPIO_GPAFEN08 ((uint32_t) 0x00000100)
#define GPIO_GPAFEN09 ((uint32_t) 0x00000200)
#define GPIO_GPAFEN010 ((uint32_t) 0x00000400)
#define GPIO_GPAFEN011 ((uint32_t) 0x00000800)
#define GPIO_GPAFEN012 ((uint32_t) 0x00001000)
#define GPIO_GPAFEN013 ((uint32_t) 0x00002000)
#define GPIO_GPAFEN014 ((uint32_t) 0x00004000)
#define GPIO_GPAFEN015 ((uint32_t) 0x00008000)
#define GPIO_GPAFEN016 ((uint32_t) 0x00010000)
#define GPIO_GPAFEN017 ((uint32_t) 0x00020000)
#define GPIO_GPAFEN018 ((uint32_t) 0x00040000)
#define GPIO_GPAFEN019 ((uint32_t) 0x00080000)
#define GPIO_GPAFEN020 ((uint32_t) 0x00100000)
#define GPIO_GPAFEN021 ((uint32_t) 0x00200000)
#define GPIO_GPAFEN022 ((uint32_t) 0x00400000)
#define GPIO_GPAFEN023 ((uint32_t) 0x00800000)
#define GPIO_GPAFEN024 ((uint32_t) 0x01000000)
#define GPIO_GPAFEN025 ((uint32_t) 0x02000000)
#define GPIO_GPAFEN026 ((uint32_t) 0x04000000)
#define GPIO_GPAFEN027 ((uint32_t) 0x08000000)
#define GPIO_GPAFEN028 ((uint32_t) 0x10000000)
#define GPIO_GPAFEN029 ((uint32_t) 0x20000000)
#define GPIO_GPAFEN030 ((uint32_t) 0x40000000)
#define GPIO_GPAFEN031 ((uint32_t) 0x80000000)

/* GPIO Pin Async. Falling Edge Detect 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPAFEN1_OFS (0)
#define GPIO_GPAFEN1_MASK ((uint32_t) 0xFFFFFFFF)
/* Read the value of GPIO pin 32 (R) */
#define GPIO_GPAFEN132 ((uint32_t) 0x00000001)
#define GPIO_GPAFEN133 ((uint32_t) 0x00000002)
#define GPIO_GPAFEN134 ((uint32_t) 0x00000004)
#define GPIO_GPAFEN135 ((uint32_t) 0x00000008)
#define GPIO_GPAFEN136 ((uint32_t) 0x00000010)
#define GPIO_GPAFEN137 ((uint32_t) 0x00000020)
#define GPIO_GPAFEN138 ((uint32_t) 0x00000040)
#define GPIO_GPAFEN139 ((uint32_t) 0x00000080)
#define GPIO_GPAFEN140 ((uint32_t) 0x00000100)
#define GPIO_GPAFEN141 ((uint32_t) 0x00000200)
#define GPIO_GPAFEN142 ((uint32_t) 0x00000400)
#define GPIO_GPAFEN143 ((uint32_t) 0x00000800)
#define GPIO_GPAFEN144 ((uint32_t) 0x00001000)
#define GPIO_GPAFEN145 ((uint32_t) 0x00002000)
#define GPIO_GPAFEN146 ((uint32_t) 0x00004000)
#define GPIO_GPAFEN147 ((uint32_t) 0x00008000)
#define GPIO_GPAFEN148 ((uint32_t) 0x00010000)
#define GPIO_GPAFEN149 ((uint32_t) 0x00020000)
#define GPIO_GPAFEN150 ((uint32_t) 0x00040000)
#define GPIO_GPAFEN151 ((uint32_t) 0x00080000)
#define GPIO_GPAFEN152 ((uint32_t) 0x00100000)
#define GPIO_GPAFEN153 ((uint32_t) 0x00200000)

/* GPIO Pin Pull-up/down Enable Register (W) */
#define GPIO_GPPUD_PUD_OFS (0)
#define GPIO_GPPUD_PUD_MASK ((uint32_t) 0x00000003)
#define GPIO_GPPUD_PUD0 ((uint32_t) 0x00000001)
#define GPIO_GPPUD_PUD1 ((uint32_t) 0x00000002)
#define GPIO_GPPUD_PUD_0 ((uint32_t) 0x00000000)
#define GPIO_GPPUD_PUD_1 ((uint32_t) 0x00000001)
#define GPIO_GPPUD_PUD_2 ((uint32_t) 0x00000002)
#define GPIO_GPPUD_PUD_3 ((uint32_t) 0x00000003)
#define GPIO_GPPUD_PUD__OFF ((uint32_t) 0x00000000)
#define GPIO_GPPUD_PUD__PULLDOWN_ENABLE ((uint32_t) 0x00000001)
#define GPIO_GPPUD_PUD__PULLUP_ENABLE ((uint32_t) 0x00000002)
#define GPIO_GPPUD_PUD__RESERVED ((uint32_t) 0x00000003)

/* GPIO Pin Pull-up/down Enable Clock 0 Register (W) */
#define GPIO_GPPUDCLK0_OFS (0)
#define GPIO_GPPUDCLK0_MASK ((uint32_t) 0xFFFFFFFF)
/* Clock the control signal into GPIO pad pull-up/down logic 0 (W) */
#define GPIO_GPPUDCLK00 ((uint32_t) 0x00000001)
#define GPIO_GPPUDCLK01 ((uint32_t) 0x00000002)
#define GPIO_GPPUDCLK02 ((uint32_t) 0x00000004)
#define GPIO_GPPUDCLK03 ((uint32_t) 0x00000008)
#define GPIO_GPPUDCLK04 ((uint32_t) 0x00000010)
#define GPIO_GPPUDCLK05 ((uint32_t) 0x00000020)
#define GPIO_GPPUDCLK06 ((uint32_t) 0x00000040)
#define GPIO_GPPUDCLK07 ((uint32_t) 0x00000080)
#define GPIO_GPPUDCLK08 ((uint32_t) 0x00000100)
#define GPIO_GPPUDCLK09 ((uint32_t) 0x00000200)
#define GPIO_GPPUDCLK010 ((uint32_t) 0x00000400)
#define GPIO_GPPUDCLK011 ((uint32_t) 0x00000800)
#define GPIO_GPPUDCLK012 ((uint32_t) 0x00001000)
#define GPIO_GPPUDCLK013 ((uint32_t) 0x00002000)
#define GPIO_GPPUDCLK014 ((uint32_t) 0x00004000)
#define GPIO_GPPUDCLK015 ((uint32_t) 0x00008000)
#define GPIO_GPPUDCLK016 ((uint32_t) 0x00010000)
#define GPIO_GPPUDCLK017 ((uint32_t) 0x00020000)
#define GPIO_GPPUDCLK018 ((uint32_t) 0x00040000)
#define GPIO_GPPUDCLK019 ((uint32_t) 0x00080000)
#define GPIO_GPPUDCLK020 ((uint32_t) 0x00100000)
#define GPIO_GPPUDCLK021 ((uint32_t) 0x00200000)
#define GPIO_GPPUDCLK022 ((uint32_t) 0x00400000)
#define GPIO_GPPUDCLK023 ((uint32_t) 0x00800000)
#define GPIO_GPPUDCLK024 ((uint32_t) 0x01000000)
#define GPIO_GPPUDCLK025 ((uint32_t) 0x02000000)
#define GPIO_GPPUDCLK026 ((uint32_t) 0x04000000)
#define GPIO_GPPUDCLK027 ((uint32_t) 0x08000000)
#define GPIO_GPPUDCLK028 ((uint32_t) 0x10000000)
#define GPIO_GPPUDCLK029 ((uint32_t) 0x20000000)
#define GPIO_GPPUDCLK030 ((uint32_t) 0x40000000)
#define GPIO_GPPUDCLK031 ((uint32_t) 0x80000000)

/* GPIO Pin Pull-up/down Enable Clock 1 Register (W) */
#define GPIO_GPPUDCLK1_OFS (0)
#define GPIO_GPPUDCLK1_MASK ((uint32_t) 0xFFFFFFFF)
/* Clock the control signal into GPIO pad pull-up/down logic 0 (W) */
#define GPIO_GPPUDCLK132 ((uint32_t) 0x00000001)
#define GPIO_GPPUDCLK133 ((uint32_t) 0x00000002)
#define GPIO_GPPUDCLK134 ((uint32_t) 0x00000004)
#define GPIO_GPPUDCLK135 ((uint32_t) 0x00000008)
#define GPIO_GPPUDCLK136 ((uint32_t) 0x00000010)
#define GPIO_GPPUDCLK137 ((uint32_t) 0x00000020)
#define GPIO_GPPUDCLK138 ((uint32_t) 0x00000040)
#define GPIO_GPPUDCLK139 ((uint32_t) 0x00000080)
#define GPIO_GPPUDCLK140 ((uint32_t) 0x00000100)
#define GPIO_GPPUDCLK141 ((uint32_t) 0x00000200)
#define GPIO_GPPUDCLK142 ((uint32_t) 0x00000400)
#define GPIO_GPPUDCLK143 ((uint32_t) 0x00000800)
#define GPIO_GPPUDCLK144 ((uint32_t) 0x00001000)
#define GPIO_GPPUDCLK145 ((uint32_t) 0x00002000)
#define GPIO_GPPUDCLK146 ((uint32_t) 0x00004000)
#define GPIO_GPPUDCLK147 ((uint32_t) 0x00008000)
#define GPIO_GPPUDCLK148 ((uint32_t) 0x00010000)
#define GPIO_GPPUDCLK149 ((uint32_t) 0x00020000)
#define GPIO_GPPUDCLK150 ((uint32_t) 0x00040000)
#define GPIO_GPPUDCLK151 ((uint32_t) 0x00080000)
#define GPIO_GPPUDCLK152 ((uint32_t) 0x00100000)
#define GPIO_GPPUDCLK153 ((uint32_t) 0x00200000)

/* GPIO Pads control bits */
/* GPIO Pads 0-27 Control register */
/* Drive strength bits (RW) */
#define GPIO_PADS0_27_DRIVE_OFS (0)
#define GPIO_PADS0_27_DRIVE_MASK ((uint32_t) 0x00000007)
#define GPIO_PADS0_27_DRIVE0 ((uint32_t) 0x00000001)
#define GPIO_PADS0_27_DRIVE1 ((uint32_t) 0x00000002)
#define GPIO_PADS0_27_DRIVE2 ((uint32_t) 0x00000004)
#define GPIO_PADS0_27_DRIVE_0 ((uint32_t) 0x00000000)
#define GPIO_PADS0_27_DRIVE_1 ((uint32_t) 0x00000001)
#define GPIO_PADS0_27_DRIVE_2 ((uint32_t) 0x00000002)
#define GPIO_PADS0_27_DRIVE_3 ((uint32_t) 0x00000003)
#define GPIO_PADS0_27_DRIVE_4 ((uint32_t) 0x00000004)
#define GPIO_PADS0_27_DRIVE_5 ((uint32_t) 0x00000005)
#define GPIO_PADS0_27_DRIVE_6 ((uint32_t) 0x00000006)
#define GPIO_PADS0_27_DRIVE_7 ((uint32_t) 0x00000007)
#define GPIO_PADS0_27_DRIVE__2MILLIAMPS ((uint32_t) 0x00000000)
#define GPIO_PADS0_27_DRIVE__4MILLIAMPS ((uint32_t) 0x00000001)
#define GPIO_PADS0_27_DRIVE__6MILLIAMPS ((uint32_t) 0x00000002)
#define GPIO_PADS0_27_DRIVE__8MILLIAMPS ((uint32_t) 0x00000003)
#define GPIO_PADS0_27_DRIVE__10MILLIAMPS ((uint32_t) 0x00000004)
#define GPIO_PADS0_27_DRIVE__12MILLIAMPS ((uint32_t) 0x00000005)
#define GPIO_PADS0_27_DRIVE__14MILLIAMPS ((uint32_t) 0x00000006)
#define GPIO_PADS0_27_DRIVE__16MILLIAMPS ((uint32_t) 0x00000007)
/* Enable input hysteresis bit (RW) */
#define GPIO_PADS0_27_HYSTERESIS_OFS (3)
#define GPIO_PADS0_27_HYSTERESIS ((uint32_t) 0x00000008)
/* Slew Rate limit bit (RW) */
#define GPIO_PADS0_27_SLEW_RATE_OFS (4)
#define GPIO_PADS0_27_SLEW_RATE ((uint32_t) 0x00000010)
/* Password (0x5A) bits (RW) */
#define GPIO_PADS0_27_PASSWD_OFS (24)
#define GPIO_PADS0_27_PASSWD_MASK ((uint32_t) 0xFF000000)
/* GPIO Pads 28-45 Control register */
/* Drive strength bits (RW) */
#define GPIO_PADS28_45_DRIVE_OFS (0)
#define GPIO_PADS28_45_DRIVE_MASK ((uint32_t) 0x00000007)
#define GPIO_PADS28_45_DRIVE0 ((uint32_t) 0x00000001)
#define GPIO_PADS28_45_DRIVE1 ((uint32_t) 0x00000002)
#define GPIO_PADS28_45_DRIVE2 ((uint32_t) 0x00000004)
#define GPIO_PADS28_45_DRIVE_0 ((uint32_t) 0x00000000)
#define GPIO_PADS28_45_DRIVE_1 ((uint32_t) 0x00000001)
#define GPIO_PADS28_45_DRIVE_2 ((uint32_t) 0x00000002)
#define GPIO_PADS28_45_DRIVE_3 ((uint32_t) 0x00000003)
#define GPIO_PADS28_45_DRIVE_4 ((uint32_t) 0x00000004)
#define GPIO_PADS28_45_DRIVE_5 ((uint32_t) 0x00000005)
#define GPIO_PADS28_45_DRIVE_6 ((uint32_t) 0x00000006)
#define GPIO_PADS28_45_DRIVE_7 ((uint32_t) 0x00000007)
#define GPIO_PADS28_45_DRIVE__2MILLIAMPS ((uint32_t) 0x00000000)
#define GPIO_PADS28_45_DRIVE__4MILLIAMPS ((uint32_t) 0x00000001)
#define GPIO_PADS28_45_DRIVE__6MILLIAMPS ((uint32_t) 0x00000002)
#define GPIO_PADS28_45_DRIVE__8MILLIAMPS ((uint32_t) 0x00000003)
#define GPIO_PADS28_45_DRIVE__10MILLIAMPS ((uint32_t) 0x00000004)
#define GPIO_PADS28_45_DRIVE__12MILLIAMPS ((uint32_t) 0x00000005)
#define GPIO_PADS28_45_DRIVE__14MILLIAMPS ((uint32_t) 0x00000006)
#define GPIO_PADS28_45_DRIVE__16MILLIAMPS ((uint32_t) 0x00000007)
/* Enable input hysteresis bit (RW) */
#define GPIO_PADS28_45_HYSTERESIS_OFS (3)
#define GPIO_PADS28_45_HYSTERESIS ((uint32_t) 0x00000008)
/* Slew Rate limit bit (RW) */
#define GPIO_PADS28_45_SLEW_RATE_OFS (4)
#define GPIO_PADS28_45_SLEW_RATE ((uint32_t) 0x00000010)
/* Password (0x5A) bits (RW) */
#define GPIO_PADS28_45_PASSWD_OFS (24)
#define GPIO_PADS28_45_PASSWD_MASK ((uint32_t) 0xFF000000)

/* GPIO Pads 46-53 Control register */
/* Drive strength bits (RW) */
#define GPIO_PADS46_53_DRIVE_OFS (0)
#define GPIO_PADS46_53_DRIVE_MASK ((uint32_t) 0x00000007)
#define GPIO_PADS46_53_DRIVE0 ((uint32_t) 0x00000001)
#define GPIO_PADS46_53_DRIVE1 ((uint32_t) 0x00000002)
#define GPIO_PADS46_53_DRIVE2 ((uint32_t) 0x00000004)
#define GPIO_PADS46_53_DRIVE_0 ((uint32_t) 0x00000000)
#define GPIO_PADS46_53_DRIVE_1 ((uint32_t) 0x00000001)
#define GPIO_PADS46_53_DRIVE_2 ((uint32_t) 0x00000002)
#define GPIO_PADS46_53_DRIVE_3 ((uint32_t) 0x00000003)
#define GPIO_PADS46_53_DRIVE_4 ((uint32_t) 0x00000004)
#define GPIO_PADS46_53_DRIVE_5 ((uint32_t) 0x00000005)
#define GPIO_PADS46_53_DRIVE_6 ((uint32_t) 0x00000006)
#define GPIO_PADS46_53_DRIVE_7 ((uint32_t) 0x00000007)
#define GPIO_PADS46_53_DRIVE__2MILLIAMPS ((uint32_t) 0x00000000)
#define GPIO_PADS46_53_DRIVE__4MILLIAMPS ((uint32_t) 0x00000001)
#define GPIO_PADS46_53_DRIVE__6MILLIAMPS ((uint32_t) 0x00000002)
#define GPIO_PADS46_53_DRIVE__8MILLIAMPS ((uint32_t) 0x00000003)
#define GPIO_PADS46_53_DRIVE__10MILLIAMPS ((uint32_t) 0x00000004)
#define GPIO_PADS46_53_DRIVE__12MILLIAMPS ((uint32_t) 0x00000005)
#define GPIO_PADS46_53_DRIVE__14MILLIAMPS ((uint32_t) 0x00000006)
#define GPIO_PADS46_53_DRIVE__16MILLIAMPS ((uint32_t) 0x00000007)
/* Enable input hysteresis bit (RW) */
#define GPIO_PADS46_53_HYSTERESIS_OFS (3)
#define GPIO_PADS46_53_HYSTERESIS ((uint32_t) 0x00000008)
/* Slew Rate limit bit (RW) */
#define GPIO_PADS46_53_SLEW_RATE_OFS (4)
#define GPIO_PADS46_53_SLEW_RATE ((uint32_t) 0x00000010)
/* Password (0x5A) bits (RW) */
#define GPIO_PADS46_53_PASSWD_OFS (24)
#define GPIO_PADS46_53_PASSWD_MASK ((uint32_t) 0xFF000000)

/* CM Bits */
/* Clock Manager General Purpose Clocks Control Register 0 (RW) */
/* SRC - Clock source (RW) */
#define CM_GPCTL0_SRC_OFS (0)
#define CM_GPCTL0_SRC_MASK ((uint32_t) 0x0000000F)
#define CM_GPCTL0_SRC0 ((uint32_t) 0x00000001)
#define CM_GPCTL0_SRC1 ((uint32_t) 0x00000002)
#define CM_GPCTL0_SRC2 ((uint32_t) 0x00000004)
#define CM_GPCTL0_SRC3 ((uint32_t) 0x00000008)
#define CM_GPCTL0_SRC_0 ((uint32_t) 0x00000000)
#define CM_GPCTL0_SRC_1 ((uint32_t) 0x00000001)
#define CM_GPCTL0_SRC_2 ((uint32_t) 0x00000002)
#define CM_GPCTL0_SRC_3 ((uint32_t) 0x00000003)
#define CM_GPCTL0_SRC_4 ((uint32_t) 0x00000004)
#define CM_GPCTL0_SRC_5 ((uint32_t) 0x00000005)
#define CM_GPCTL0_SRC_6 ((uint32_t) 0x00000006)
#define CM_GPCTL0_SRC_7 ((uint32_t) 0x00000007)
#define CM_GPCTL0_SRC_8 ((uint32_t) 0x00000008)
#define CM_GPCTL0_SRC_9 ((uint32_t) 0x00000009)
#define CM_GPCTL0_SRC_10 ((uint32_t) 0x0000000A)
#define CM_GPCTL0_SRC_11 ((uint32_t) 0x0000000B)
#define CM_GPCTL0_SRC_12 ((uint32_t) 0x0000000C)
#define CM_GPCTL0_SRC_13 ((uint32_t) 0x0000000D)
#define CM_GPCTL0_SRC_14 ((uint32_t) 0x0000000E)
#define CM_GPCTL0_SRC_15 ((uint32_t) 0x0000000F)
#define CM_GPCTL0_SRC__GND ((uint32_t) 0x00000000)
#define CM_GPCTL0_SRC__OSCILLATOR ((uint32_t) 0x00000001)
#define CM_GPCTL0_SRC__TESTDEBUG0 ((uint32_t) 0x00000002)
#define CM_GPCTL0_SRC__TESTDEBUG1 ((uint32_t) 0x00000003)
#define CM_GPCTL0_SRC__PLLA ((uint32_t) 0x00000004)
#define CM_GPCTL0_SRC__PLLC ((uint32_t) 0x00000005)
#define CM_GPCTL0_SRC__PLLD ((uint32_t) 0x00000006)
#define CM_GPCTL0_SRC__HDMI_AUXILIARY ((uint32_t) 0x00000007)
/* ENAB - Enable the clock generator (RW) */
#define CM_GPCTL0_ENAB_OFS (4)
#define CM_GPCTL0_ENAB ((uint32_t) 0x00000010)
/* KILL - Kill the clock generator (RW) */
#define CM_GPCTL0_KILL_OFS (5)
#define CM_GPCTL0_KILL ((uint32_t) 0x00000020)
/* BUSY - Clock generator is running (R) */
#define CM_GPCTL0_BUSY_OFS (7)
#define CM_GPCTL0_BUSY ((uint32_t) 0x00000080)
/* FLIP - Invert the clock generator output (RW) */
#define CM_GPCTL0_FLIP_OFS (8)
#define CM_GPCTL0_FLIP ((uint32_t) 0x00000100)
/* MASH - MASH control (RW) */
#define CM_GPCTL0_MASH_OFS (9)
#define CM_GPCTL0_MASH_MASK ((uint32_t) 0x00000600)
#define CM_GPCTL0_MASH0 ((uint32_t) 0x00000200)
#define CM_GPCTL0_MASH1 ((uint32_t) 0x00000400)
#define CM_GPCTL0_MASH_0 ((uint32_t) 0x00000000)
#define CM_GPCTL0_MASH_1 ((uint32_t) 0x00000200)
#define CM_GPCTL0_MASH_2 ((uint32_t) 0x00000400)
#define CM_GPCTL0_MASH_3 ((uint32_t) 0x00000600)
#define CM_GPCTL0_MASH__INTEGER_DIVISION ((uint32_t) 0x00000000)
#define CM_GPCTL0_MASH__1STAGE_MASH ((uint32_t) 0x00000200)
#define CM_GPCTL0_MASH__2STAGE_MASH ((uint32_t) 0x00000400)
#define CM_GPCTL0_MASH__3STAGE_MASH ((uint32_t) 0x00000600)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPCTL0_PASSWD_OFS (24)
#define CM_GPCTL0_PASSWD_MASK ((uint32_t) 0xFF000000)
#define CM_GPCTL0_PASSWD0 ((uint32_t) 0x01000000)
#define CM_GPCTL0_PASSWD1 ((uint32_t) 0x02000000)
#define CM_GPCTL0_PASSWD2 ((uint32_t) 0x04000000)
#define CM_GPCTL0_PASSWD3 ((uint32_t) 0x08000000)
#define CM_GPCTL0_PASSWD4 ((uint32_t) 0x10000000)
#define CM_GPCTL0_PASSWD5 ((uint32_t) 0x20000000)
#define CM_GPCTL0_PASSWD6 ((uint32_t) 0x40000000)
#define CM_GPCTL0_PASSWD7 ((uint32_t) 0x80000000)

/* Clock Manager General Purpose Clocks Control Register 1 (RW) */
/* SRC - Clock source (RW) */
#define CM_GPCTL1_SRC_OFS (0)
#define CM_GPCTL1_SRC_MASK ((uint32_t) 0x0000000F)
#define CM_GPCTL1_SRC0 ((uint32_t) 0x00000001)
#define CM_GPCTL1_SRC1 ((uint32_t) 0x00000002)
#define CM_GPCTL1_SRC2 ((uint32_t) 0x00000004)
#define CM_GPCTL1_SRC3 ((uint32_t) 0x00000008)
#define CM_GPCTL1_SRC_0 ((uint32_t) 0x00000000)
#define CM_GPCTL1_SRC_1 ((uint32_t) 0x00000001)
#define CM_GPCTL1_SRC_2 ((uint32_t) 0x00000002)
#define CM_GPCTL1_SRC_3 ((uint32_t) 0x00000003)
#define CM_GPCTL1_SRC_4 ((uint32_t) 0x00000004)
#define CM_GPCTL1_SRC_5 ((uint32_t) 0x00000005)
#define CM_GPCTL1_SRC_6 ((uint32_t) 0x00000006)
#define CM_GPCTL1_SRC_7 ((uint32_t) 0x00000007)
#define CM_GPCTL1_SRC_8 ((uint32_t) 0x00000008)
#define CM_GPCTL1_SRC_9 ((uint32_t) 0x00000009)
#define CM_GPCTL1_SRC_10 ((uint32_t) 0x0000000A)
#define CM_GPCTL1_SRC_11 ((uint32_t) 0x0000000B)
#define CM_GPCTL1_SRC_12 ((uint32_t) 0x0000000C)
#define CM_GPCTL1_SRC_13 ((uint32_t) 0x0000000D)
#define CM_GPCTL1_SRC_14 ((uint32_t) 0x0000000E)
#define CM_GPCTL1_SRC_15 ((uint32_t) 0x0000000F)
#define CM_GPCTL1_SRC__GND ((uint32_t) 0x00000000)
#define CM_GPCTL1_SRC__OSCILLATOR ((uint32_t) 0x00000001)
#define CM_GPCTL1_SRC__TESTDEBUG0 ((uint32_t) 0x00000002)
#define CM_GPCTL1_SRC__TESTDEBUG1 ((uint32_t) 0x00000003)
#define CM_GPCTL1_SRC__PLLA ((uint32_t) 0x00000004)
#define CM_GPCTL1_SRC__PLLC ((uint32_t) 0x00000005)
#define CM_GPCTL1_SRC__PLLD ((uint32_t) 0x00000006)
#define CM_GPCTL1_SRC__HDMI_AUXILIARY ((uint32_t) 0x00000007)
/* ENAB - Enable the clock generator (RW) */
#define CM_GPCTL1_ENAB_OFS (4)
#define CM_GPCTL1_ENAB ((uint32_t) 0x00000010)
/* KILL - Kill the clock generator (RW) */
#define CM_GPCTL1_KILL_OFS (5)
#define CM_GPCTL1_KILL ((uint32_t) 0x00000020)
/* BUSY - Clock generator is running (R) */
#define CM_GPCTL1_BUSY_OFS (7)
#define CM_GPCTL1_BUSY ((uint32_t) 0x00000080)
/* FLIP - Invert the clock generator output (RW) */
#define CM_GPCTL1_FLIP_OFS (8)
#define CM_GPCTL1_FLIP ((uint32_t) 0x00000100)
/* MASH - MASH control (RW) */
#define CM_GPCTL1_MASH_OFS (9)
#define CM_GPCTL1_MASH_MASK ((uint32_t) 0x00000600)
#define CM_GPCTL1_MASH0 ((uint32_t) 0x00000200)
#define CM_GPCTL1_MASH1 ((uint32_t) 0x00000400)
#define CM_GPCTL1_MASH_0 ((uint32_t) 0x00000000)
#define CM_GPCTL1_MASH_1 ((uint32_t) 0x00000200)
#define CM_GPCTL1_MASH_2 ((uint32_t) 0x00000400)
#define CM_GPCTL1_MASH_3 ((uint32_t) 0x00000600)
#define CM_GPCTL1_MASH__INTEGER_DIVISION ((uint32_t) 0x00000000)
#define CM_GPCTL1_MASH__1STAGE_MASH ((uint32_t) 0x00000200)
#define CM_GPCTL1_MASH__2STAGE_MASH ((uint32_t) 0x00000400)
#define CM_GPCTL1_MASH__3STAGE_MASH ((uint32_t) 0x00000600)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPCTL1_PASSWD_OFS (24)
#define CM_GPCTL1_PASSWD_MASK ((uint32_t) 0xFF000000)
#define CM_GPCTL1_PASSWD0 ((uint32_t) 0x01000000)
#define CM_GPCTL1_PASSWD1 ((uint32_t) 0x02000000)
#define CM_GPCTL1_PASSWD2 ((uint32_t) 0x04000000)
#define CM_GPCTL1_PASSWD3 ((uint32_t) 0x08000000)
#define CM_GPCTL1_PASSWD4 ((uint32_t) 0x10000000)
#define CM_GPCTL1_PASSWD5 ((uint32_t) 0x20000000)
#define CM_GPCTL1_PASSWD6 ((uint32_t) 0x40000000)
#define CM_GPCTL1_PASSWD7 ((uint32_t) 0x80000000)

/* Clock Manager General Purpose Clocks Control Register 2 (RW) */
/* SRC - Clock source (RW) */
#define CM_GPCTL2_SRC_OFS (0)
#define CM_GPCTL2_SRC_MASK ((uint32_t) 0x0000000F)
#define CM_GPCTL2_SRC0 ((uint32_t) 0x00000001)
#define CM_GPCTL2_SRC1 ((uint32_t) 0x00000002)
#define CM_GPCTL2_SRC2 ((uint32_t) 0x00000004)
#define CM_GPCTL2_SRC3 ((uint32_t) 0x00000008)
#define CM_GPCTL2_SRC_0 ((uint32_t) 0x00000000)
#define CM_GPCTL2_SRC_1 ((uint32_t) 0x00000001)
#define CM_GPCTL2_SRC_2 ((uint32_t) 0x00000002)
#define CM_GPCTL2_SRC_3 ((uint32_t) 0x00000003)
#define CM_GPCTL2_SRC_4 ((uint32_t) 0x00000004)
#define CM_GPCTL2_SRC_5 ((uint32_t) 0x00000005)
#define CM_GPCTL2_SRC_6 ((uint32_t) 0x00000006)
#define CM_GPCTL2_SRC_7 ((uint32_t) 0x00000007)
#define CM_GPCTL2_SRC_8 ((uint32_t) 0x00000008)
#define CM_GPCTL2_SRC_9 ((uint32_t) 0x00000009)
#define CM_GPCTL2_SRC_10 ((uint32_t) 0x0000000A)
#define CM_GPCTL2_SRC_11 ((uint32_t) 0x0000000B)
#define CM_GPCTL2_SRC_12 ((uint32_t) 0x0000000C)
#define CM_GPCTL2_SRC_13 ((uint32_t) 0x0000000D)
#define CM_GPCTL2_SRC_14 ((uint32_t) 0x0000000E)
#define CM_GPCTL2_SRC_15 ((uint32_t) 0x0000000F)
#define CM_GPCTL2_SRC__GND ((uint32_t) 0x00000000)
#define CM_GPCTL2_SRC__OSCILLATOR ((uint32_t) 0x00000001)
#define CM_GPCTL2_SRC__TESTDEBUG0 ((uint32_t) 0x00000002)
#define CM_GPCTL2_SRC__TESTDEBUG1 ((uint32_t) 0x00000003)
#define CM_GPCTL2_SRC__PLLA ((uint32_t) 0x00000004)
#define CM_GPCTL2_SRC__PLLC ((uint32_t) 0x00000005)
#define CM_GPCTL2_SRC__PLLD ((uint32_t) 0x00000006)
#define CM_GPCTL2_SRC__HDMI_AUXILIARY ((uint32_t) 0x00000007)
/* ENAB - Enable the clock generator (RW) */
#define CM_GPCTL2_ENAB_OFS (4)
#define CM_GPCTL2_ENAB ((uint32_t) 0x00000010)
/* KILL - Kill the clock generator (RW) */
#define CM_GPCTL2_KILL_OFS (5)
#define CM_GPCTL2_KILL ((uint32_t) 0x00000020)
/* BUSY - Clock generator is running (R) */
#define CM_GPCTL2_BUSY_OFS (7)
#define CM_GPCTL2_BUSY ((uint32_t) 0x00000080)
/* FLIP - Invert the clock generator output (RW) */
#define CM_GPCTL2_FLIP_OFS (8)
#define CM_GPCTL2_FLIP ((uint32_t) 0x00000100)
/* MASH - MASH control (RW) */
#define CM_GPCTL2_MASH_OFS (9)
#define CM_GPCTL2_MASH_MASK ((uint32_t) 0x00000600)
#define CM_GPCTL2_MASH0 ((uint32_t) 0x00000200)
#define CM_GPCTL2_MASH1 ((uint32_t) 0x00000400)
#define CM_GPCTL2_MASH_0 ((uint32_t) 0x00000000)
#define CM_GPCTL2_MASH_1 ((uint32_t) 0x00000200)
#define CM_GPCTL2_MASH_2 ((uint32_t) 0x00000400)
#define CM_GPCTL2_MASH_3 ((uint32_t) 0x00000600)
#define CM_GPCTL2_MASH__INTEGER_DIVISION ((uint32_t) 0x00000000)
#define CM_GPCTL2_MASH__1STAGE_MASH ((uint32_t) 0x00000200)
#define CM_GPCTL2_MASH__2STAGE_MASH ((uint32_t) 0x00000400)
#define CM_GPCTL2_MASH__3STAGE_MASH ((uint32_t) 0x00000600)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPCTL2_PASSWD_OFS (24)
#define CM_GPCTL2_PASSWD_MASK ((uint32_t) 0xFF000000)
#define CM_GPCTL2_PASSWD0 ((uint32_t) 0x01000000)
#define CM_GPCTL2_PASSWD1 ((uint32_t) 0x02000000)
#define CM_GPCTL2_PASSWD2 ((uint32_t) 0x04000000)
#define CM_GPCTL2_PASSWD3 ((uint32_t) 0x08000000)
#define CM_GPCTL2_PASSWD4 ((uint32_t) 0x10000000)
#define CM_GPCTL2_PASSWD5 ((uint32_t) 0x20000000)
#define CM_GPCTL2_PASSWD6 ((uint32_t) 0x40000000)
#define CM_GPCTL2_PASSWD7 ((uint32_t) 0x80000000)

/* Clock Manager General Purpose Clock Divisors Register 0 (RW) */
/* DIVF - Fractional part of divisor (RW) */
#define CM_GPDIV0_DIVF_OFS (0)
#define CM_GPDIV0_DIVF_MASK ((uint32_t) 0x00000FFF)
#define CM_GPDIV0_DIVF0 ((uint32_t) 0x00000001)
#define CM_GPDIV0_DIVF1 ((uint32_t) 0x00000002)
#define CM_GPDIV0_DIVF2 ((uint32_t) 0x00000004)
#define CM_GPDIV0_DIVF3 ((uint32_t) 0x00000008)
#define CM_GPDIV0_DIVF4 ((uint32_t) 0x00000010)
#define CM_GPDIV0_DIVF5 ((uint32_t) 0x00000020)
#define CM_GPDIV0_DIVF6 ((uint32_t) 0x00000040)
#define CM_GPDIV0_DIVF7 ((uint32_t) 0x00000080)
#define CM_GPDIV0_DIVF8 ((uint32_t) 0x00000100)
#define CM_GPDIV0_DIVF9 ((uint32_t) 0x00000200)
#define CM_GPDIV0_DIVF10 ((uint32_t) 0x00000400)
#define CM_GPDIV0_DIVF11 ((uint32_t) 0x00000800)
/* DIVI - Integer part of divisor (RW) */
#define CM_GPDIV0_DIVI_OFS (12)
#define CM_GPDIV0_DIVI_MASK ((uint32_t) 0x00FFF000)
#define CM_GPDIV0_DIVI0 ((uint32_t) 0x00001000)
#define CM_GPDIV0_DIVI1 ((uint32_t) 0x00002000)
#define CM_GPDIV0_DIVI2 ((uint32_t) 0x00004000)
#define CM_GPDIV0_DIVI3 ((uint32_t) 0x00008000)
#define CM_GPDIV0_DIVI4 ((uint32_t) 0x00010000)
#define CM_GPDIV0_DIVI5 ((uint32_t) 0x00020000)
#define CM_GPDIV0_DIVI6 ((uint32_t) 0x00040000)
#define CM_GPDIV0_DIVI7 ((uint32_t) 0x00080000)
#define CM_GPDIV0_DIVI8 ((uint32_t) 0x00100000)
#define CM_GPDIV0_DIVI9 ((uint32_t) 0x00200000)
#define CM_GPDIV0_DIVI10 ((uint32_t) 0x00400000)
#define CM_GPDIV0_DIVI11 ((uint32_t) 0x00800000)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPDIV0_PASSWD_OFS (24)
#define CM_GPDIV0_PASSWD_MASK ((uint32_t) 0xFF000000)
#define CM_GPDIV0_PASSWD0 ((uint32_t) 0x01000000)
#define CM_GPDIV0_PASSWD1 ((uint32_t) 0x02000000)
#define CM_GPDIV0_PASSWD2 ((uint32_t) 0x04000000)
#define CM_GPDIV0_PASSWD3 ((uint32_t) 0x08000000)
#define CM_GPDIV0_PASSWD4 ((uint32_t) 0x10000000)
#define CM_GPDIV0_PASSWD5 ((uint32_t) 0x20000000)
#define CM_GPDIV0_PASSWD6 ((uint32_t) 0x40000000)
#define CM_GPDIV0_PASSWD7 ((uint32_t) 0x80000000)
/* Clock Manager General Purpose Clock Divisors Register 1 (RW) */
/* DIVF - Fractional part of divisor (RW) */
#define CM_GPDIV1_DIVF_OFS (0)
#define CM_GPDIV1_DIVF_MASK ((uint32_t) 0x00000FFF)
#define CM_GPDIV1_DIVF0 ((uint32_t) 0x00000001)
#define CM_GPDIV1_DIVF1 ((uint32_t) 0x00000002)
#define CM_GPDIV1_DIVF2 ((uint32_t) 0x00000004)
#define CM_GPDIV1_DIVF3 ((uint32_t) 0x00000008)
#define CM_GPDIV1_DIVF4 ((uint32_t) 0x00000010)
#define CM_GPDIV1_DIVF5 ((uint32_t) 0x00000020)
#define CM_GPDIV1_DIVF6 ((uint32_t) 0x00000040)
#define CM_GPDIV1_DIVF7 ((uint32_t) 0x00000080)
#define CM_GPDIV1_DIVF8 ((uint32_t) 0x00000100)
#define CM_GPDIV1_DIVF9 ((uint32_t) 0x00000200)
#define CM_GPDIV1_DIVF10 ((uint32_t) 0x00000400)
#define CM_GPDIV1_DIVF11 ((uint32_t) 0x00000800)
/* DIVI - Integer part of divisor (RW) */
#define CM_GPDIV1_DIVI_OFS (12)
#define CM_GPDIV1_DIVI_MASK ((uint32_t) 0x00FFF000)
#define CM_GPDIV1_DIVI0 ((uint32_t) 0x00001000)
#define CM_GPDIV1_DIVI1 ((uint32_t) 0x00002000)
#define CM_GPDIV1_DIVI2 ((uint32_t) 0x00004000)
#define CM_GPDIV1_DIVI3 ((uint32_t) 0x00008000)
#define CM_GPDIV1_DIVI4 ((uint32_t) 0x00010000)
#define CM_GPDIV1_DIVI5 ((uint32_t) 0x00020000)
#define CM_GPDIV1_DIVI6 ((uint32_t) 0x00040000)
#define CM_GPDIV1_DIVI7 ((uint32_t) 0x00080000)
#define CM_GPDIV1_DIVI8 ((uint32_t) 0x00100000)
#define CM_GPDIV1_DIVI9 ((uint32_t) 0x00200000)
#define CM_GPDIV1_DIVI10 ((uint32_t) 0x00400000)
#define CM_GPDIV1_DIVI11 ((uint32_t) 0x00800000)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPDIV1_PASSWD_OFS (24)
#define CM_GPDIV1_PASSWD_MASK ((uint32_t) 0xFF000000)
#define CM_GPDIV1_PASSWD0 ((uint32_t) 0x01000000)
#define CM_GPDIV1_PASSWD1 ((uint32_t) 0x02000000)
#define CM_GPDIV1_PASSWD2 ((uint32_t) 0x04000000)
#define CM_GPDIV1_PASSWD3 ((uint32_t) 0x08000000)
#define CM_GPDIV1_PASSWD4 ((uint32_t) 0x10000000)
#define CM_GPDIV1_PASSWD5 ((uint32_t) 0x20000000)
#define CM_GPDIV1_PASSWD6 ((uint32_t) 0x40000000)
#define CM_GPDIV1_PASSWD7 ((uint32_t) 0x80000000)
/* Clock Manager General Purpose Clock Divisors Register 2 (RW) */
/* DIVF - Fractional part of divisor (RW) */
#define CM_GPDIV2_DIVF_OFS (0)
#define CM_GPDIV2_DIVF_MASK ((uint32_t) 0x00000FFF)
#define CM_GPDIV2_DIVF0 ((uint32_t) 0x00000001)
#define CM_GPDIV2_DIVF1 ((uint32_t) 0x00000002)
#define CM_GPDIV2_DIVF2 ((uint32_t) 0x00000004)
#define CM_GPDIV2_DIVF3 ((uint32_t) 0x00000008)
#define CM_GPDIV2_DIVF4 ((uint32_t) 0x00000010)
#define CM_GPDIV2_DIVF5 ((uint32_t) 0x00000020)
#define CM_GPDIV2_DIVF6 ((uint32_t) 0x00000040)
#define CM_GPDIV2_DIVF7 ((uint32_t) 0x00000080)
#define CM_GPDIV2_DIVF8 ((uint32_t) 0x00000100)
#define CM_GPDIV2_DIVF9 ((uint32_t) 0x00000200)
#define CM_GPDIV2_DIVF10 ((uint32_t) 0x00000400)
#define CM_GPDIV2_DIVF11 ((uint32_t) 0x00000800)
/* DIVI - Integer part of divisor (RW) */
#define CM_GPDIV2_DIVI_OFS (12)
#define CM_GPDIV2_DIVI_MASK ((uint32_t) 0x00FFF000)
#define CM_GPDIV2_DIVI0 ((uint32_t) 0x00001000)
#define CM_GPDIV2_DIVI1 ((uint32_t) 0x00002000)
#define CM_GPDIV2_DIVI2 ((uint32_t) 0x00004000)
#define CM_GPDIV2_DIVI3 ((uint32_t) 0x00008000)
#define CM_GPDIV2_DIVI4 ((uint32_t) 0x00010000)
#define CM_GPDIV2_DIVI5 ((uint32_t) 0x00020000)
#define CM_GPDIV2_DIVI6 ((uint32_t) 0x00040000)
#define CM_GPDIV2_DIVI7 ((uint32_t) 0x00080000)
#define CM_GPDIV2_DIVI8 ((uint32_t) 0x00100000)
#define CM_GPDIV2_DIVI9 ((uint32_t) 0x00200000)
#define CM_GPDIV2_DIVI10 ((uint32_t) 0x00400000)
#define CM_GPDIV2_DIVI11 ((uint32_t) 0x00800000)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPDIV2_PASSWD_OFS (24)
#define CM_GPDIV2_PASSWD_MASK ((uint32_t) 0xFF000000)
#define CM_GPDIV2_PASSWD0 ((uint32_t) 0x01000000)
#define CM_GPDIV2_PASSWD1 ((uint32_t) 0x02000000)
#define CM_GPDIV2_PASSWD2 ((uint32_t) 0x04000000)
#define CM_GPDIV2_PASSWD3 ((uint32_t) 0x08000000)
#define CM_GPDIV2_PASSWD4 ((uint32_t) 0x10000000)
#define CM_GPDIV2_PASSWD5 ((uint32_t) 0x20000000)
#define CM_GPDIV2_PASSWD6 ((uint32_t) 0x40000000)
#define CM_GPDIV2_PASSWD7 ((uint32_t) 0x80000000)

/* Interrupt Controller Bits */
/* IRQ Basic Pending bits (R) */
#define IRQ_BP_ARM_TIMER_OFS (0)
#define IRQ_BP_ARM_TIMER ((uint32_t) 0x00000001)
#define IRQ_BP_ARM_MAILBOX_OFS (1)
#define IRQ_BP_ARM_MAILBOX ((uint32_t) 0x00000002)
#define IRQ_BP_ARM_DOORBELL_0_OFS (2)
#define IRQ_BP_ARM_DOORBELL_0 ((uint32_t) 0x00000004)
#define IRQ_BP_ARM_DOORBELL_1_OFS (3)
#define IRQ_BP_ARM_DOORBELL_1 ((uint32_t) 0x00000008)
#define IRQ_BP_GPU0_HALTED_OFS (4)
#define IRQ_BP_GPU0_HALTED ((uint32_t) 0x00000010)
#define IRQ_BP_GPU1_HALTED_OFS (5)
#define IRQ_BP_GPU1_HALTED ((uint32_t) 0x00000020)
#define IRQ_BP_ILLEGAL_ACCESS_TYPE_1_OFS (6)
#define IRQ_BP_ILLEGAL_ACCESS_TYPE_1 ((uint32_t) 0x00000040)
#define IRQ_BP_ILLEGAL_ACCESS_TYPE_0_OFS (7)
#define IRQ_BP_ILLEGAL_ACCESS_TYPE_0 ((uint32_t) 0x00000080)
#define IRQ_BP_BITS_SET_IN_PR1_OFS (8)
#define IRQ_BP_BITS_SET_IN_PR1 ((uint32_t) 0x00000100)
#define IRQ_BP_BITS_SET_IN_PR2_OFS (9)
#define IRQ_BP_BITS_SET_IN_PR2 ((uint32_t) 0x00000200)
#define IRQ_BP_GPU_IRQ7_OFS (10)
#define IRQ_BP_GPU_IRQ7 ((uint32_t) 0x00000400)
#define IRQ_BP_GPU_IRQ9_OFS (11)
#define IRQ_BP_GPU_IRQ9 ((uint32_t) 0x00000800)
#define IRQ_BP_GPU_IRQ10_OFS (12)
#define IRQ_BP_GPU_IRQ10 ((uint32_t) 0x00001000)
#define IRQ_BP_GPU_IRQ18_OFS (13)
#define IRQ_BP_GPU_IRQ18 ((uint32_t) 0x00002000)
#define IRQ_BP_GPU_IRQ19_OFS (14)
#define IRQ_BP_GPU_IRQ19 ((uint32_t) 0x00004000)
#define IRQ_BP_GPU_IRQ53_OFS (15)
#define IRQ_BP_GPU_IRQ53 ((uint32_t) 0x00008000)
#define IRQ_BP_GPU_IRQ54_OFS (16)
#define IRQ_BP_GPU_IRQ54 ((uint32_t) 0x00010000)
#define IRQ_BP_GPU_IRQ55_OFS (17)
#define IRQ_BP_GPU_IRQ55 ((uint32_t) 0x00020000)
#define IRQ_BP_GPU_IRQ56_OFS (18)
#define IRQ_BP_GPU_IRQ56 ((uint32_t) 0x00040000)
#define IRQ_BP_GPU_IRQ57_OFS (19)
#define IRQ_BP_GPU_IRQ57 ((uint32_t) 0x00080000)
#define IRQ_BP_GPU_IRQ60_OFS (20)
#define IRQ_BP_GPU_IRQ60 ((uint32_t) 0x00100000)
/* IRQ pending 1 (R) */
#define IRQ_PENDING1_SRC_OFS (0)
#define IRQ_PENDING1_SRC_MASK ((uint32_t) 0xFFFFFFFF)
#define IRQ_PENDING10 ((uint32_t) 0x00000001)
#define IRQ_PENDING11 ((uint32_t) 0x00000002)
#define IRQ_PENDING12 ((uint32_t) 0x00000004)
#define IRQ_PENDING13 ((uint32_t) 0x00000008)
#define IRQ_PENDING14 ((uint32_t) 0x00000010)
#define IRQ_PENDING15 ((uint32_t) 0x00000020)
#define IRQ_PENDING16 ((uint32_t) 0x00000040)
#define IRQ_PENDING17 ((uint32_t) 0x00000080)
#define IRQ_PENDING18 ((uint32_t) 0x00000100)
#define IRQ_PENDING19 ((uint32_t) 0x00000200)
#define IRQ_PENDING110 ((uint32_t) 0x00000400)
#define IRQ_PENDING111 ((uint32_t) 0x00000800)
#define IRQ_PENDING112 ((uint32_t) 0x00001000)
#define IRQ_PENDING113 ((uint32_t) 0x00002000)
#define IRQ_PENDING114 ((uint32_t) 0x00004000)
#define IRQ_PENDING115 ((uint32_t) 0x00008000)
#define IRQ_PENDING116 ((uint32_t) 0x00010000)
#define IRQ_PENDING117 ((uint32_t) 0x00020000)
#define IRQ_PENDING118 ((uint32_t) 0x00040000)
#define IRQ_PENDING119 ((uint32_t) 0x00080000)
#define IRQ_PENDING120 ((uint32_t) 0x00100000)
#define IRQ_PENDING121 ((uint32_t) 0x00200000)
#define IRQ_PENDING122 ((uint32_t) 0x00400000)
#define IRQ_PENDING123 ((uint32_t) 0x00800000)
#define IRQ_PENDING124 ((uint32_t) 0x01000000)
#define IRQ_PENDING125 ((uint32_t) 0x02000000)
#define IRQ_PENDING126 ((uint32_t) 0x04000000)
#define IRQ_PENDING127 ((uint32_t) 0x08000000)
#define IRQ_PENDING128 ((uint32_t) 0x10000000)
#define IRQ_PENDING129 ((uint32_t) 0x20000000)
#define IRQ_PENDING130 ((uint32_t) 0x40000000)
#define IRQ_PENDING131 ((uint32_t) 0x80000000)
#define IRQ_PENDING1_0 ((uint32_t) 0x00000001)
#define IRQ_PENDING1_1 ((uint32_t) 0x00000002)
#define IRQ_PENDING1_2 ((uint32_t) 0x00000004)
#define IRQ_PENDING1_3 ((uint32_t) 0x00000008)
#define IRQ_PENDING1_4 ((uint32_t) 0x00000010)
#define IRQ_PENDING1_5 ((uint32_t) 0x00000020)
#define IRQ_PENDING1_6 ((uint32_t) 0x00000040)
#define IRQ_PENDING1_7 ((uint32_t) 0x00000080)
#define IRQ_PENDING1_8 ((uint32_t) 0x00000100)
#define IRQ_PENDING1_9 ((uint32_t) 0x00000200)
#define IRQ_PENDING1_10 ((uint32_t) 0x00000400)
#define IRQ_PENDING1_11 ((uint32_t) 0x00000800)
#define IRQ_PENDING1_12 ((uint32_t) 0x00001000)
#define IRQ_PENDING1_13 ((uint32_t) 0x00002000)
#define IRQ_PENDING1_14 ((uint32_t) 0x00004000)
#define IRQ_PENDING1_15 ((uint32_t) 0x00008000)
#define IRQ_PENDING1_16 ((uint32_t) 0x00010000)
#define IRQ_PENDING1_17 ((uint32_t) 0x00020000)
#define IRQ_PENDING1_18 ((uint32_t) 0x00040000)
#define IRQ_PENDING1_19 ((uint32_t) 0x00080000)
#define IRQ_PENDING1_20 ((uint32_t) 0x00100000)
#define IRQ_PENDING1_21 ((uint32_t) 0x00200000)
#define IRQ_PENDING1_22 ((uint32_t) 0x00400000)
#define IRQ_PENDING1_23 ((uint32_t) 0x00800000)
#define IRQ_PENDING1_24 ((uint32_t) 0x01000000)
#define IRQ_PENDING1_25 ((uint32_t) 0x02000000)
#define IRQ_PENDING1_26 ((uint32_t) 0x04000000)
#define IRQ_PENDING1_27 ((uint32_t) 0x08000000)
#define IRQ_PENDING1_28 ((uint32_t) 0x10000000)
#define IRQ_PENDING1_29 ((uint32_t) 0x20000000)
#define IRQ_PENDING1_30 ((uint32_t) 0x40000000)
#define IRQ_PENDING1_31 ((uint32_t) 0x80000000)
#define IRQ_PENDING1__SYSTMR_MATCH_1 ((uint32_t) 0x00000001)
#define IRQ_PENDING1__SYSTMR_MATCH_3 ((uint32_t) 0x00000008)
#define IRQ_PENDING1__USB_CONTROLLER ((uint32_t) 0x00000200)
#define IRQ_PENDING1__AUX_INT ((uint32_t) 0x20000000)

/* IRQ pending 2 (R) */
#define IRQ_PENDING2_SRC_OFS (0)
#define IRQ_PENDING2_SRC_MASK ((uint32_t) 0xFFFFFFFF)
#define IRQ_PENDING20 ((uint32_t) 0x00000001)
#define IRQ_PENDING21 ((uint32_t) 0x00000002)
#define IRQ_PENDING22 ((uint32_t) 0x00000004)
#define IRQ_PENDING23 ((uint32_t) 0x00000008)
#define IRQ_PENDING24 ((uint32_t) 0x00000010)
#define IRQ_PENDING25 ((uint32_t) 0x00000020)
#define IRQ_PENDING26 ((uint32_t) 0x00000040)
#define IRQ_PENDING27 ((uint32_t) 0x00000080)
#define IRQ_PENDING28 ((uint32_t) 0x00000100)
#define IRQ_PENDING29 ((uint32_t) 0x00000200)
#define IRQ_PENDING210 ((uint32_t) 0x00000400)
#define IRQ_PENDING211 ((uint32_t) 0x00000800)
#define IRQ_PENDING212 ((uint32_t) 0x00001000)
#define IRQ_PENDING213 ((uint32_t) 0x00002000)
#define IRQ_PENDING214 ((uint32_t) 0x00004000)
#define IRQ_PENDING215 ((uint32_t) 0x00008000)
#define IRQ_PENDING216 ((uint32_t) 0x00010000)
#define IRQ_PENDING217 ((uint32_t) 0x00020000)
#define IRQ_PENDING218 ((uint32_t) 0x00040000)
#define IRQ_PENDING219 ((uint32_t) 0x00080000)
#define IRQ_PENDING220 ((uint32_t) 0x00100000)
#define IRQ_PENDING221 ((uint32_t) 0x00200000)
#define IRQ_PENDING222 ((uint32_t) 0x00400000)
#define IRQ_PENDING223 ((uint32_t) 0x00800000)
#define IRQ_PENDING224 ((uint32_t) 0x01000000)
#define IRQ_PENDING225 ((uint32_t) 0x02000000)
#define IRQ_PENDING226 ((uint32_t) 0x04000000)
#define IRQ_PENDING227 ((uint32_t) 0x08000000)
#define IRQ_PENDING228 ((uint32_t) 0x10000000)
#define IRQ_PENDING229 ((uint32_t) 0x20000000)
#define IRQ_PENDING230 ((uint32_t) 0x40000000)
#define IRQ_PENDING231 ((uint32_t) 0x80000000)
#define IRQ_PENDING2_0 ((uint32_t) 0x00000001)
#define IRQ_PENDING2_1 ((uint32_t) 0x00000002)
#define IRQ_PENDING2_2 ((uint32_t) 0x00000004)
#define IRQ_PENDING2_3 ((uint32_t) 0x00000008)
#define IRQ_PENDING2_4 ((uint32_t) 0x00000010)
#define IRQ_PENDING2_5 ((uint32_t) 0x00000020)
#define IRQ_PENDING2_6 ((uint32_t) 0x00000040)
#define IRQ_PENDING2_7 ((uint32_t) 0x00000080)
#define IRQ_PENDING2_8 ((uint32_t) 0x00000100)
#define IRQ_PENDING2_9 ((uint32_t) 0x00000200)
#define IRQ_PENDING2_10 ((uint32_t) 0x00000400)
#define IRQ_PENDING2_11 ((uint32_t) 0x00000800)
#define IRQ_PENDING2_12 ((uint32_t) 0x00001000)
#define IRQ_PENDING2_13 ((uint32_t) 0x00002000)
#define IRQ_PENDING2_14 ((uint32_t) 0x00004000)
#define IRQ_PENDING2_15 ((uint32_t) 0x00008000)
#define IRQ_PENDING2_16 ((uint32_t) 0x00010000)
#define IRQ_PENDING2_17 ((uint32_t) 0x00020000)
#define IRQ_PENDING2_18 ((uint32_t) 0x00040000)
#define IRQ_PENDING2_19 ((uint32_t) 0x00080000)
#define IRQ_PENDING2_20 ((uint32_t) 0x00100000)
#define IRQ_PENDING2_21 ((uint32_t) 0x00200000)
#define IRQ_PENDING2_22 ((uint32_t) 0x00400000)
#define IRQ_PENDING2_23 ((uint32_t) 0x00800000)
#define IRQ_PENDING2_24 ((uint32_t) 0x01000000)
#define IRQ_PENDING2_25 ((uint32_t) 0x02000000)
#define IRQ_PENDING2_26 ((uint32_t) 0x04000000)
#define IRQ_PENDING2_27 ((uint32_t) 0x08000000)
#define IRQ_PENDING2_28 ((uint32_t) 0x10000000)
#define IRQ_PENDING2_29 ((uint32_t) 0x20000000)
#define IRQ_PENDING2_30 ((uint32_t) 0x40000000)
#define IRQ_PENDING2_31 ((uint32_t) 0x80000000)
#define IRQ_PENDING2__I2C_SPI_SLV_INT ((uint32_t) 0x00000800)
#define IRQ_PENDING2__PWA0 ((uint32_t) 0x00002000)
#define IRQ_PENDING2__PWA1 ((uint32_t) 0x00004000)
#define IRQ_PENDING2__SMI ((uint32_t) 0x00010000)
#define IRQ_PENDING2__GPIO_INT_0 ((uint32_t) 0x00020000)
#define IRQ_PENDING2__GPIO_INT_1 ((uint32_t) 0x00040000)
#define IRQ_PENDING2__GPIO_INT_2 ((uint32_t) 0x00080000)
#define IRQ_PENDING2__GPIO_INT_3 ((uint32_t) 0x00100000)
#define IRQ_PENDING2__I2C_INT ((uint32_t) 0x00200000)
#define IRQ_PENDING2__SPI_INT ((uint32_t) 0x00400000)
#define IRQ_PENDING2__PCM_INT ((uint32_t) 0x00800000)
#define IRQ_PENDING2__UART_INT ((uint32_t) 0x02000000)

/* FIQ control (RW) */
#define IRQ_FIQ_SRC_OFS (0)
#define IRQ_FIQ_SRC_MASK ((uint32_t) 0x0000007F)
#define IRQ_FIQ_SRC0 ((uint32_t) 0x00000001)
#define IRQ_FIQ_SRC1 ((uint32_t) 0x00000002)
#define IRQ_FIQ_SRC2 ((uint32_t) 0x00000004)
#define IRQ_FIQ_SRC3 ((uint32_t) 0x00000008)
#define IRQ_FIQ_SRC4 ((uint32_t) 0x00000010)
#define IRQ_FIQ_SRC5 ((uint32_t) 0x00000020)
#define IRQ_FIQ_SRC6 ((uint32_t) 0x00000040)
#define IRQ_FIQ_SRC_0 ((uint32_t) 0x00000000)
#define IRQ_FIQ_SRC_1 ((uint32_t) 0x00000001)
#define IRQ_FIQ_SRC_2 ((uint32_t) 0x00000002)
#define IRQ_FIQ_SRC_3 ((uint32_t) 0x00000003)
#define IRQ_FIQ_SRC_4 ((uint32_t) 0x00000004)
#define IRQ_FIQ_SRC_5 ((uint32_t) 0x00000005)
#define IRQ_FIQ_SRC_6 ((uint32_t) 0x00000006)
#define IRQ_FIQ_SRC_7 ((uint32_t) 0x00000007)
#define IRQ_FIQ_SRC_8 ((uint32_t) 0x00000008)
#define IRQ_FIQ_SRC_9 ((uint32_t) 0x00000009)
#define IRQ_FIQ_SRC_10 ((uint32_t) 0x0000000A)
#define IRQ_FIQ_SRC_11 ((uint32_t) 0x0000000B)
#define IRQ_FIQ_SRC_12 ((uint32_t) 0x0000000C)
#define IRQ_FIQ_SRC_13 ((uint32_t) 0x0000000D)
#define IRQ_FIQ_SRC_14 ((uint32_t) 0x0000000E)
#define IRQ_FIQ_SRC_15 ((uint32_t) 0x0000000F)
#define IRQ_FIQ_SRC_16 ((uint32_t) 0x00000010)
#define IRQ_FIQ_SRC_17 ((uint32_t) 0x00000011)
#define IRQ_FIQ_SRC_18 ((uint32_t) 0x00000012)
#define IRQ_FIQ_SRC_19 ((uint32_t) 0x00000013)
#define IRQ_FIQ_SRC_20 ((uint32_t) 0x00000014)
#define IRQ_FIQ_SRC_21 ((uint32_t) 0x00000015)
#define IRQ_FIQ_SRC_22 ((uint32_t) 0x00000016)
#define IRQ_FIQ_SRC_23 ((uint32_t) 0x00000017)
#define IRQ_FIQ_SRC_24 ((uint32_t) 0x00000018)
#define IRQ_FIQ_SRC_25 ((uint32_t) 0x00000019)
#define IRQ_FIQ_SRC_26 ((uint32_t) 0x0000001A)
#define IRQ_FIQ_SRC_27 ((uint32_t) 0x0000001B)
#define IRQ_FIQ_SRC_28 ((uint32_t) 0x0000001C)
#define IRQ_FIQ_SRC_29 ((uint32_t) 0x0000001D)
#define IRQ_FIQ_SRC_30 ((uint32_t) 0x0000001E)
#define IRQ_FIQ_SRC_31 ((uint32_t) 0x0000001F)
#define IRQ_FIQ_SRC_32 ((uint32_t) 0x00000020)
#define IRQ_FIQ_SRC_33 ((uint32_t) 0x00000021)
#define IRQ_FIQ_SRC_34 ((uint32_t) 0x00000022)
#define IRQ_FIQ_SRC_35 ((uint32_t) 0x00000023)
#define IRQ_FIQ_SRC_36 ((uint32_t) 0x00000024)
#define IRQ_FIQ_SRC_37 ((uint32_t) 0x00000025)
#define IRQ_FIQ_SRC_38 ((uint32_t) 0x00000026)
#define IRQ_FIQ_SRC_39 ((uint32_t) 0x00000027)
#define IRQ_FIQ_SRC_40 ((uint32_t) 0x00000028)
#define IRQ_FIQ_SRC_41 ((uint32_t) 0x00000029)
#define IRQ_FIQ_SRC_42 ((uint32_t) 0x0000002A)
#define IRQ_FIQ_SRC_43 ((uint32_t) 0x0000002B)
#define IRQ_FIQ_SRC_44 ((uint32_t) 0x0000002C)
#define IRQ_FIQ_SRC_45 ((uint32_t) 0x0000002D)
#define IRQ_FIQ_SRC_46 ((uint32_t) 0x0000002E)
#define IRQ_FIQ_SRC_47 ((uint32_t) 0x0000002F)
#define IRQ_FIQ_SRC_48 ((uint32_t) 0x00000030)
#define IRQ_FIQ_SRC_49 ((uint32_t) 0x00000031)
#define IRQ_FIQ_SRC_50 ((uint32_t) 0x00000032)
#define IRQ_FIQ_SRC_51 ((uint32_t) 0x00000033)
#define IRQ_FIQ_SRC_52 ((uint32_t) 0x00000034)
#define IRQ_FIQ_SRC_53 ((uint32_t) 0x00000035)
#define IRQ_FIQ_SRC_54 ((uint32_t) 0x00000036)
#define IRQ_FIQ_SRC_55 ((uint32_t) 0x00000037)
#define IRQ_FIQ_SRC_56 ((uint32_t) 0x00000038)
#define IRQ_FIQ_SRC_57 ((uint32_t) 0x00000039)
#define IRQ_FIQ_SRC_58 ((uint32_t) 0x0000003A)
#define IRQ_FIQ_SRC_59 ((uint32_t) 0x0000003B)
#define IRQ_FIQ_SRC_60 ((uint32_t) 0x0000003C)
#define IRQ_FIQ_SRC_61 ((uint32_t) 0x0000003D)
#define IRQ_FIQ_SRC_62 ((uint32_t) 0x0000003E)
#define IRQ_FIQ_SRC_63 ((uint32_t) 0x0000003F)
#define IRQ_FIQ_SRC_64 ((uint32_t) 0x00000040)
#define IRQ_FIQ_SRC_65 ((uint32_t) 0x00000041)
#define IRQ_FIQ_SRC_66 ((uint32_t) 0x00000042)
#define IRQ_FIQ_SRC_67 ((uint32_t) 0x00000043)
#define IRQ_FIQ_SRC_68 ((uint32_t) 0x00000044)
#define IRQ_FIQ_SRC_69 ((uint32_t) 0x00000045)
#define IRQ_FIQ_SRC_70 ((uint32_t) 0x00000046)
#define IRQ_FIQ_SRC_71 ((uint32_t) 0x00000047)
/* 72-127 Do Not Use */
#define IRQ_FIQ_SRC_72 ((uint32_t) 0x00000048)
#define IRQ_FIQ_SRC_73 ((uint32_t) 0x00000049)
#define IRQ_FIQ_SRC_74 ((uint32_t) 0x0000004A)
#define IRQ_FIQ_SRC_75 ((uint32_t) 0x0000004B)
#define IRQ_FIQ_SRC_76 ((uint32_t) 0x0000004C)
#define IRQ_FIQ_SRC_77 ((uint32_t) 0x0000004D)
#define IRQ_FIQ_SRC_78 ((uint32_t) 0x0000004E)
#define IRQ_FIQ_SRC_79 ((uint32_t) 0x0000004F)
#define IRQ_FIQ_SRC_80 ((uint32_t) 0x00000050)
#define IRQ_FIQ_SRC_81 ((uint32_t) 0x00000051)
#define IRQ_FIQ_SRC_82 ((uint32_t) 0x00000052)
#define IRQ_FIQ_SRC_83 ((uint32_t) 0x00000053)
#define IRQ_FIQ_SRC_84 ((uint32_t) 0x00000054)
#define IRQ_FIQ_SRC_85 ((uint32_t) 0x00000055)
#define IRQ_FIQ_SRC_86 ((uint32_t) 0x00000056)
#define IRQ_FIQ_SRC_87 ((uint32_t) 0x00000057)
#define IRQ_FIQ_SRC_88 ((uint32_t) 0x00000058)
#define IRQ_FIQ_SRC_89 ((uint32_t) 0x00000059)
#define IRQ_FIQ_SRC_90 ((uint32_t) 0x0000005A)
#define IRQ_FIQ_SRC_91 ((uint32_t) 0x0000005B)
#define IRQ_FIQ_SRC_92 ((uint32_t) 0x0000005C)
#define IRQ_FIQ_SRC_93 ((uint32_t) 0x0000005D)
#define IRQ_FIQ_SRC_94 ((uint32_t) 0x0000005E)
#define IRQ_FIQ_SRC_95 ((uint32_t) 0x0000005F)
#define IRQ_FIQ_SRC_96 ((uint32_t) 0x00000060)
#define IRQ_FIQ_SRC_97 ((uint32_t) 0x00000061)
#define IRQ_FIQ_SRC_98 ((uint32_t) 0x00000062)
#define IRQ_FIQ_SRC_99 ((uint32_t) 0x00000063)
#define IRQ_FIQ_SRC_100 ((uint32_t) 0x00000064)
#define IRQ_FIQ_SRC_101 ((uint32_t) 0x00000065)
#define IRQ_FIQ_SRC_102 ((uint32_t) 0x00000066)
#define IRQ_FIQ_SRC_103 ((uint32_t) 0x00000067)
#define IRQ_FIQ_SRC_104 ((uint32_t) 0x00000068)
#define IRQ_FIQ_SRC_105 ((uint32_t) 0x00000069)
#define IRQ_FIQ_SRC_106 ((uint32_t) 0x0000006A)
#define IRQ_FIQ_SRC_107 ((uint32_t) 0x0000006B)
#define IRQ_FIQ_SRC_108 ((uint32_t) 0x0000006C)
#define IRQ_FIQ_SRC_109 ((uint32_t) 0x0000006D)
#define IRQ_FIQ_SRC_110 ((uint32_t) 0x0000006E)
#define IRQ_FIQ_SRC_111 ((uint32_t) 0x0000006F)
#define IRQ_FIQ_SRC_112 ((uint32_t) 0x00000070)
#define IRQ_FIQ_SRC_113 ((uint32_t) 0x00000071)
#define IRQ_FIQ_SRC_114 ((uint32_t) 0x00000072)
#define IRQ_FIQ_SRC_115 ((uint32_t) 0x00000073)
#define IRQ_FIQ_SRC_116 ((uint32_t) 0x00000074)
#define IRQ_FIQ_SRC_117 ((uint32_t) 0x00000075)
#define IRQ_FIQ_SRC_118 ((uint32_t) 0x00000076)
#define IRQ_FIQ_SRC_119 ((uint32_t) 0x00000077)
#define IRQ_FIQ_SRC_120 ((uint32_t) 0x00000078)
#define IRQ_FIQ_SRC_121 ((uint32_t) 0x00000079)
#define IRQ_FIQ_SRC_122 ((uint32_t) 0x0000007A)
#define IRQ_FIQ_SRC_123 ((uint32_t) 0x0000007B)
#define IRQ_FIQ_SRC_124 ((uint32_t) 0x0000007C)
#define IRQ_FIQ_SRC_125 ((uint32_t) 0x0000007D)
#define IRQ_FIQ_SRC_126 ((uint32_t) 0x0000007E)
#define IRQ_FIQ_SRC_127 ((uint32_t) 0x0000007F)
#define IRQ_FIQ_SRC__SYSTMR_MATCH_1 ((uint32_t) 0x00000001)
#define IRQ_FIQ_SRC__SYSTMR_MATCH_3 ((uint32_t) 0x00000003)
#define IRQ_FIQ_SRC__USB_CONTROLLER ((uint32_t) 0x00000009)
#define IRQ_FIQ_SRC__AUX_INT ((uint32_t) 0x0000001D)
#define IRQ_FIQ_SRC__I2C_SPI_SLV_INT ((uint32_t) 0x0000002B)
#define IRQ_FIQ_SRC__PWA0 ((uint32_t) 0x0000002D)
#define IRQ_FIQ_SRC__PWA1 ((uint32_t) 0x0000002E)
#define IRQ_FIQ_SRC__SMI ((uint32_t) 0x00000030)
#define IRQ_FIQ_SRC__GPIO_INT_0 ((uint32_t) 0x00000031)
#define IRQ_FIQ_SRC__GPIO_INT_1 ((uint32_t) 0x00000032)
#define IRQ_FIQ_SRC__GPIO_INT_2 ((uint32_t) 0x00000033)
#define IRQ_FIQ_SRC__GPIO_INT_3 ((uint32_t) 0x00000034)
#define IRQ_FIQ_SRC__I2C_INT ((uint32_t) 0x00000035)
#define IRQ_FIQ_SRC__SPI_INT ((uint32_t) 0x00000036)
#define IRQ_FIQ_SRC__PCM_INT ((uint32_t) 0x00000037)
#define IRQ_FIQ_SRC__UART_INT ((uint32_t) 0x00000039)
#define IRQ_FIQ_SRC__ARM_TIMER ((uint32_t) 0x00000040)
#define IRQ_FIQ_SRC__ARM_MAILBOX ((uint32_t) 0x00000041)
#define IRQ_FIQ_SRC__ARM_DOORBELL_0 ((uint32_t) 0x00000042)
#define IRQ_FIQ_SRC__ARM_DOORBELL_1 ((uint32_t) 0x00000043)
#define IRQ_FIQ_SRC__GPU0_HALTED ((uint32_t) 0x00000044)
#define IRQ_FIQ_SRC__GPU1_HALTED ((uint32_t) 0x00000045)
#define IRQ_FIQ_SRC__ILLEGAL_ACCESS_T1 ((uint32_t) 0x00000046)
#define IRQ_FIQ_SRC__ILLEGAL_ACCESS_T2 ((uint32_t) 0x00000047)
#define IRQ_FIQ_ENABLE_OFS (7)
#define IRQ_FIQ_ENABLE ((uint32_t) 0x00000080)

/* Enable IRQs 1 (W) */
#define IRQ_IER1_OFS (0)
#define IRQ_IER1_MASK ((uint32_t) 0xFFFFFFFF)
#define IRQ_IER10 ((uint32_t) 0x00000001)
#define IRQ_IER11 ((uint32_t) 0x00000002)
#define IRQ_IER12 ((uint32_t) 0x00000004)
#define IRQ_IER13 ((uint32_t) 0x00000008)
#define IRQ_IER14 ((uint32_t) 0x00000010)
#define IRQ_IER15 ((uint32_t) 0x00000020)
#define IRQ_IER16 ((uint32_t) 0x00000040)
#define IRQ_IER17 ((uint32_t) 0x00000080)
#define IRQ_IER18 ((uint32_t) 0x00000100)
#define IRQ_IER19 ((uint32_t) 0x00000200)
#define IRQ_IER110 ((uint32_t) 0x00000400)
#define IRQ_IER111 ((uint32_t) 0x00000800)
#define IRQ_IER112 ((uint32_t) 0x00001000)
#define IRQ_IER113 ((uint32_t) 0x00002000)
#define IRQ_IER114 ((uint32_t) 0x00004000)
#define IRQ_IER115 ((uint32_t) 0x00008000)
#define IRQ_IER116 ((uint32_t) 0x00010000)
#define IRQ_IER117 ((uint32_t) 0x00020000)
#define IRQ_IER118 ((uint32_t) 0x00040000)
#define IRQ_IER119 ((uint32_t) 0x00080000)
#define IRQ_IER120 ((uint32_t) 0x00100000)
#define IRQ_IER121 ((uint32_t) 0x00200000)
#define IRQ_IER122 ((uint32_t) 0x00400000)
#define IRQ_IER123 ((uint32_t) 0x00800000)
#define IRQ_IER124 ((uint32_t) 0x01000000)
#define IRQ_IER125 ((uint32_t) 0x02000000)
#define IRQ_IER126 ((uint32_t) 0x04000000)
#define IRQ_IER127 ((uint32_t) 0x08000000)
#define IRQ_IER128 ((uint32_t) 0x10000000)
#define IRQ_IER129 ((uint32_t) 0x20000000)
#define IRQ_IER130 ((uint32_t) 0x40000000)
#define IRQ_IER131 ((uint32_t) 0x80000000)
#define IRQ_IER1_0 ((uint32_t) 0x00000001)
#define IRQ_IER1_1 ((uint32_t) 0x00000002)
#define IRQ_IER1_2 ((uint32_t) 0x00000004)
#define IRQ_IER1_3 ((uint32_t) 0x00000008)
#define IRQ_IER1_4 ((uint32_t) 0x00000010)
#define IRQ_IER1_5 ((uint32_t) 0x00000020)
#define IRQ_IER1_6 ((uint32_t) 0x00000040)
#define IRQ_IER1_7 ((uint32_t) 0x00000080)
#define IRQ_IER1_8 ((uint32_t) 0x00000100)
#define IRQ_IER1_9 ((uint32_t) 0x00000200)
#define IRQ_IER1_10 ((uint32_t) 0x00000400)
#define IRQ_IER1_11 ((uint32_t) 0x00000800)
#define IRQ_IER1_12 ((uint32_t) 0x00001000)
#define IRQ_IER1_13 ((uint32_t) 0x00002000)
#define IRQ_IER1_14 ((uint32_t) 0x00004000)
#define IRQ_IER1_15 ((uint32_t) 0x00008000)
#define IRQ_IER1_16 ((uint32_t) 0x00010000)
#define IRQ_IER1_17 ((uint32_t) 0x00020000)
#define IRQ_IER1_18 ((uint32_t) 0x00040000)
#define IRQ_IER1_19 ((uint32_t) 0x00080000)
#define IRQ_IER1_20 ((uint32_t) 0x00100000)
#define IRQ_IER1_21 ((uint32_t) 0x00200000)
#define IRQ_IER1_22 ((uint32_t) 0x00400000)
#define IRQ_IER1_23 ((uint32_t) 0x00800000)
#define IRQ_IER1_24 ((uint32_t) 0x01000000)
#define IRQ_IER1_25 ((uint32_t) 0x02000000)
#define IRQ_IER1_26 ((uint32_t) 0x04000000)
#define IRQ_IER1_27 ((uint32_t) 0x08000000)
#define IRQ_IER1_28 ((uint32_t) 0x10000000)
#define IRQ_IER1_29 ((uint32_t) 0x20000000)
#define IRQ_IER1_30 ((uint32_t) 0x40000000)
#define IRQ_IER1_31 ((uint32_t) 0x80000000)
#define IRQ_IER1__SYSTMR_MATCH_1 ((uint32_t) 0x00000001)
#define IRQ_IER1__SYSTMR_MATCH_3 ((uint32_t) 0x00000008)
#define IRQ_IER1__USB_CONTROLLER ((uint32_t) 0x00000200)
#define IRQ_IER1__AUX_INT ((uint32_t) 0x20000000)
/* Enable IRQs 2 (W) */
#define IRQ_IER2_OFS (0)
#define IRQ_IER2_MASK ((uint32_t) 0xFFFFFFFF)
#define IRQ_IER20 ((uint32_t) 0x00000001)
#define IRQ_IER21 ((uint32_t) 0x00000002)
#define IRQ_IER22 ((uint32_t) 0x00000004)
#define IRQ_IER23 ((uint32_t) 0x00000008)
#define IRQ_IER24 ((uint32_t) 0x00000010)
#define IRQ_IER25 ((uint32_t) 0x00000020)
#define IRQ_IER26 ((uint32_t) 0x00000040)
#define IRQ_IER27 ((uint32_t) 0x00000080)
#define IRQ_IER28 ((uint32_t) 0x00000100)
#define IRQ_IER29 ((uint32_t) 0x00000200)
#define IRQ_IER210 ((uint32_t) 0x00000400)
#define IRQ_IER211 ((uint32_t) 0x00000800)
#define IRQ_IER212 ((uint32_t) 0x00001000)
#define IRQ_IER213 ((uint32_t) 0x00002000)
#define IRQ_IER214 ((uint32_t) 0x00004000)
#define IRQ_IER215 ((uint32_t) 0x00008000)
#define IRQ_IER216 ((uint32_t) 0x00010000)
#define IRQ_IER217 ((uint32_t) 0x00020000)
#define IRQ_IER218 ((uint32_t) 0x00040000)
#define IRQ_IER219 ((uint32_t) 0x00080000)
#define IRQ_IER220 ((uint32_t) 0x00100000)
#define IRQ_IER221 ((uint32_t) 0x00200000)
#define IRQ_IER222 ((uint32_t) 0x00400000)
#define IRQ_IER223 ((uint32_t) 0x00800000)
#define IRQ_IER224 ((uint32_t) 0x01000000)
#define IRQ_IER225 ((uint32_t) 0x02000000)
#define IRQ_IER226 ((uint32_t) 0x04000000)
#define IRQ_IER227 ((uint32_t) 0x08000000)
#define IRQ_IER228 ((uint32_t) 0x10000000)
#define IRQ_IER229 ((uint32_t) 0x20000000)
#define IRQ_IER230 ((uint32_t) 0x40000000)
#define IRQ_IER231 ((uint32_t) 0x80000000)
#define IRQ_IER2_0 ((uint32_t) 0x00000001)
#define IRQ_IER2_1 ((uint32_t) 0x00000002)
#define IRQ_IER2_2 ((uint32_t) 0x00000004)
#define IRQ_IER2_3 ((uint32_t) 0x00000008)
#define IRQ_IER2_4 ((uint32_t) 0x00000010)
#define IRQ_IER2_5 ((uint32_t) 0x00000020)
#define IRQ_IER2_6 ((uint32_t) 0x00000040)
#define IRQ_IER2_7 ((uint32_t) 0x00000080)
#define IRQ_IER2_8 ((uint32_t) 0x00000100)
#define IRQ_IER2_9 ((uint32_t) 0x00000200)
#define IRQ_IER2_10 ((uint32_t) 0x00000400)
#define IRQ_IER2_11 ((uint32_t) 0x00000800)
#define IRQ_IER2_12 ((uint32_t) 0x00001000)
#define IRQ_IER2_13 ((uint32_t) 0x00002000)
#define IRQ_IER2_14 ((uint32_t) 0x00004000)
#define IRQ_IER2_15 ((uint32_t) 0x00008000)
#define IRQ_IER2_16 ((uint32_t) 0x00010000)
#define IRQ_IER2_17 ((uint32_t) 0x00020000)
#define IRQ_IER2_18 ((uint32_t) 0x00040000)
#define IRQ_IER2_19 ((uint32_t) 0x00080000)
#define IRQ_IER2_20 ((uint32_t) 0x00100000)
#define IRQ_IER2_21 ((uint32_t) 0x00200000)
#define IRQ_IER2_22 ((uint32_t) 0x00400000)
#define IRQ_IER2_23 ((uint32_t) 0x00800000)
#define IRQ_IER2_24 ((uint32_t) 0x01000000)
#define IRQ_IER2_25 ((uint32_t) 0x02000000)
#define IRQ_IER2_26 ((uint32_t) 0x04000000)
#define IRQ_IER2_27 ((uint32_t) 0x08000000)
#define IRQ_IER2_28 ((uint32_t) 0x10000000)
#define IRQ_IER2_29 ((uint32_t) 0x20000000)
#define IRQ_IER2_30 ((uint32_t) 0x40000000)
#define IRQ_IER2_31 ((uint32_t) 0x80000000)
#define IRQ_IER2__I2C_SPI_SLV_INT ((uint32_t) 0x00000800)
#define IRQ_IER2__PWA0 ((uint32_t) 0x00002000)
#define IRQ_IER2__PWA1 ((uint32_t) 0x00004000)
#define IRQ_IER2__SMI ((uint32_t) 0x00010000)
#define IRQ_IER2__GPIO_INT_0 ((uint32_t) 0x00020000)
#define IRQ_IER2__GPIO_INT_1 ((uint32_t) 0x00040000)
#define IRQ_IER2__GPIO_INT_2 ((uint32_t) 0x00080000)
#define IRQ_IER2__GPIO_INT_3 ((uint32_t) 0x00100000)
#define IRQ_IER2__I2C_INT ((uint32_t) 0x00200000)
#define IRQ_IER2__SPI_INT ((uint32_t) 0x00400000)
#define IRQ_IER2__PCM_INT ((uint32_t) 0x00800000)
#define IRQ_IER2__UART_INT ((uint32_t) 0x02000000)

/* Enable Basic IRQs (RW) */
#define IRQ_BASIC_IRQ_ENABLE_ARM_TIMER_OFS (0)
#define IRQ_BASIC_IRQ_ENABLE_ARM_TIMER ((uint32_t) 0x00000001)
#define IRQ_BASIC_IRQ_ENABLE_ARM_MAILBOX_OFS (1)
#define IRQ_BASIC_IRQ_ENABLE_ARM_MAILBOX ((uint32_t) 0x00000002)
#define IRQ_BASIC_IRQ_ENABLE_ARM_DOORBELL_0_OFS (2)
#define IRQ_BASIC_IRQ_ENABLE_ARM_DOORBELL_0 ((uint32_t) 0x00000004)
#define IRQ_BASIC_IRQ_ENABLE_ARM_DOORBELL_1_OFS (3)
#define IRQ_BASIC_IRQ_ENABLE_ARM_DOORBELL_1 ((uint32_t) 0x00000008)
#define IRQ_BASIC_IRQ_ENABLE_GPU0_HALTED_OFS (4)
#define IRQ_BASIC_IRQ_ENABLE_GPU0_HALTED ((uint32_t) 0x00000010)
#define IRQ_BASIC_IRQ_ENABLE_GPU1_HALTED_OFS (5)
#define IRQ_BASIC_IRQ_ENABLE_GPU1_HALTED ((uint32_t) 0x00000020)
#define IRQ_BASIC_IRQ_ENABLE_ACCESS_ERROR_T1_OFS (6)
#define IRQ_BASIC_IRQ_ENABLE_ACCESS_ERROR_T1 ((uint32_t) 0x00000040)
#define IRQ_BASIC_IRQ_ENABLE_ACCESS_ERROR_T0_OFS (7)
#define IRQ_BASIC_IRQ_ENABLE_ACCESS_ERROR_T0 ((uint32_t) 0x00000080)

/* Disable IRQs 1 */
#define IRQ_IDR1_OFS (0)
#define IRQ_IDR1_MASK ((uint32_t) 0xFFFFFFFF)
#define IRQ_IDR10 ((uint32_t) 0x00000001)
#define IRQ_IDR11 ((uint32_t) 0x00000002)
#define IRQ_IDR12 ((uint32_t) 0x00000004)
#define IRQ_IDR13 ((uint32_t) 0x00000008)
#define IRQ_IDR14 ((uint32_t) 0x00000010)
#define IRQ_IDR15 ((uint32_t) 0x00000020)
#define IRQ_IDR16 ((uint32_t) 0x00000040)
#define IRQ_IDR17 ((uint32_t) 0x00000080)
#define IRQ_IDR18 ((uint32_t) 0x00000100)
#define IRQ_IDR19 ((uint32_t) 0x00000200)
#define IRQ_IDR110 ((uint32_t) 0x00000400)
#define IRQ_IDR111 ((uint32_t) 0x00000800)
#define IRQ_IDR112 ((uint32_t) 0x00001000)
#define IRQ_IDR113 ((uint32_t) 0x00002000)
#define IRQ_IDR114 ((uint32_t) 0x00004000)
#define IRQ_IDR115 ((uint32_t) 0x00008000)
#define IRQ_IDR116 ((uint32_t) 0x00010000)
#define IRQ_IDR117 ((uint32_t) 0x00020000)
#define IRQ_IDR118 ((uint32_t) 0x00040000)
#define IRQ_IDR119 ((uint32_t) 0x00080000)
#define IRQ_IDR120 ((uint32_t) 0x00100000)
#define IRQ_IDR121 ((uint32_t) 0x00200000)
#define IRQ_IDR122 ((uint32_t) 0x00400000)
#define IRQ_IDR123 ((uint32_t) 0x00800000)
#define IRQ_IDR124 ((uint32_t) 0x01000000)
#define IRQ_IDR125 ((uint32_t) 0x02000000)
#define IRQ_IDR126 ((uint32_t) 0x04000000)
#define IRQ_IDR127 ((uint32_t) 0x08000000)
#define IRQ_IDR128 ((uint32_t) 0x10000000)
#define IRQ_IDR129 ((uint32_t) 0x20000000)
#define IRQ_IDR130 ((uint32_t) 0x40000000)
#define IRQ_IDR131 ((uint32_t) 0x80000000)
#define IRQ_IDR1_0 ((uint32_t) 0x00000001)
#define IRQ_IDR1_1 ((uint32_t) 0x00000002)
#define IRQ_IDR1_2 ((uint32_t) 0x00000004)
#define IRQ_IDR1_3 ((uint32_t) 0x00000008)
#define IRQ_IDR1_4 ((uint32_t) 0x00000010)
#define IRQ_IDR1_5 ((uint32_t) 0x00000020)
#define IRQ_IDR1_6 ((uint32_t) 0x00000040)
#define IRQ_IDR1_7 ((uint32_t) 0x00000080)
#define IRQ_IDR1_8 ((uint32_t) 0x00000100)
#define IRQ_IDR1_9 ((uint32_t) 0x00000200)
#define IRQ_IDR1_10 ((uint32_t) 0x00000400)
#define IRQ_IDR1_11 ((uint32_t) 0x00000800)
#define IRQ_IDR1_12 ((uint32_t) 0x00001000)
#define IRQ_IDR1_13 ((uint32_t) 0x00002000)
#define IRQ_IDR1_14 ((uint32_t) 0x00004000)
#define IRQ_IDR1_15 ((uint32_t) 0x00008000)
#define IRQ_IDR1_16 ((uint32_t) 0x00010000)
#define IRQ_IDR1_17 ((uint32_t) 0x00020000)
#define IRQ_IDR1_18 ((uint32_t) 0x00040000)
#define IRQ_IDR1_19 ((uint32_t) 0x00080000)
#define IRQ_IDR1_20 ((uint32_t) 0x00100000)
#define IRQ_IDR1_21 ((uint32_t) 0x00200000)
#define IRQ_IDR1_22 ((uint32_t) 0x00400000)
#define IRQ_IDR1_23 ((uint32_t) 0x00800000)
#define IRQ_IDR1_24 ((uint32_t) 0x01000000)
#define IRQ_IDR1_25 ((uint32_t) 0x02000000)
#define IRQ_IDR1_26 ((uint32_t) 0x04000000)
#define IRQ_IDR1_27 ((uint32_t) 0x08000000)
#define IRQ_IDR1_28 ((uint32_t) 0x10000000)
#define IRQ_IDR1_29 ((uint32_t) 0x20000000)
#define IRQ_IDR1_30 ((uint32_t) 0x40000000)
#define IRQ_IDR1_31 ((uint32_t) 0x80000000)
#define IRQ_IDR1__SYSTMR_MATCH_1 ((uint32_t) 0x00000001)
#define IRQ_IDR1__SYSTMR_MATCH_3 ((uint32_t) 0x00000008)
#define IRQ_IDR1__USB_CONTROLLER ((uint32_t) 0x00000200)
#define IRQ_IDR1__AUX_INT ((uint32_t) 0x20000000)

/* Disable IRQs 2 (RW) */
#define IRQ_IDR2_OFS (0)
#define IRQ_IDR2_MASK ((uint32_t) 0xFFFFFFFF)
#define IRQ_IDR20 ((uint32_t) 0x00000001)
#define IRQ_IDR21 ((uint32_t) 0x00000002)
#define IRQ_IDR22 ((uint32_t) 0x00000004)
#define IRQ_IDR23 ((uint32_t) 0x00000008)
#define IRQ_IDR24 ((uint32_t) 0x00000010)
#define IRQ_IDR25 ((uint32_t) 0x00000020)
#define IRQ_IDR26 ((uint32_t) 0x00000040)
#define IRQ_IDR27 ((uint32_t) 0x00000080)
#define IRQ_IDR28 ((uint32_t) 0x00000100)
#define IRQ_IDR29 ((uint32_t) 0x00000200)
#define IRQ_IDR210 ((uint32_t) 0x00000400)
#define IRQ_IDR211 ((uint32_t) 0x00000800)
#define IRQ_IDR212 ((uint32_t) 0x00001000)
#define IRQ_IDR213 ((uint32_t) 0x00002000)
#define IRQ_IDR214 ((uint32_t) 0x00004000)
#define IRQ_IDR215 ((uint32_t) 0x00008000)
#define IRQ_IDR216 ((uint32_t) 0x00010000)
#define IRQ_IDR217 ((uint32_t) 0x00020000)
#define IRQ_IDR218 ((uint32_t) 0x00040000)
#define IRQ_IDR219 ((uint32_t) 0x00080000)
#define IRQ_IDR220 ((uint32_t) 0x00100000)
#define IRQ_IDR221 ((uint32_t) 0x00200000)
#define IRQ_IDR222 ((uint32_t) 0x00400000)
#define IRQ_IDR223 ((uint32_t) 0x00800000)
#define IRQ_IDR224 ((uint32_t) 0x01000000)
#define IRQ_IDR225 ((uint32_t) 0x02000000)
#define IRQ_IDR226 ((uint32_t) 0x04000000)
#define IRQ_IDR227 ((uint32_t) 0x08000000)
#define IRQ_IDR228 ((uint32_t) 0x10000000)
#define IRQ_IDR229 ((uint32_t) 0x20000000)
#define IRQ_IDR230 ((uint32_t) 0x40000000)
#define IRQ_IDR231 ((uint32_t) 0x80000000)
#define IRQ_IDR2_0 ((uint32_t) 0x00000001)
#define IRQ_IDR2_1 ((uint32_t) 0x00000002)
#define IRQ_IDR2_2 ((uint32_t) 0x00000004)
#define IRQ_IDR2_3 ((uint32_t) 0x00000008)
#define IRQ_IDR2_4 ((uint32_t) 0x00000010)
#define IRQ_IDR2_5 ((uint32_t) 0x00000020)
#define IRQ_IDR2_6 ((uint32_t) 0x00000040)
#define IRQ_IDR2_7 ((uint32_t) 0x00000080)
#define IRQ_IDR2_8 ((uint32_t) 0x00000100)
#define IRQ_IDR2_9 ((uint32_t) 0x00000200)
#define IRQ_IDR2_10 ((uint32_t) 0x00000400)
#define IRQ_IDR2_11 ((uint32_t) 0x00000800)
#define IRQ_IDR2_12 ((uint32_t) 0x00001000)
#define IRQ_IDR2_13 ((uint32_t) 0x00002000)
#define IRQ_IDR2_14 ((uint32_t) 0x00004000)
#define IRQ_IDR2_15 ((uint32_t) 0x00008000)
#define IRQ_IDR2_16 ((uint32_t) 0x00010000)
#define IRQ_IDR2_17 ((uint32_t) 0x00020000)
#define IRQ_IDR2_18 ((uint32_t) 0x00040000)
#define IRQ_IDR2_19 ((uint32_t) 0x00080000)
#define IRQ_IDR2_20 ((uint32_t) 0x00100000)
#define IRQ_IDR2_21 ((uint32_t) 0x00200000)
#define IRQ_IDR2_22 ((uint32_t) 0x00400000)
#define IRQ_IDR2_23 ((uint32_t) 0x00800000)
#define IRQ_IDR2_24 ((uint32_t) 0x01000000)
#define IRQ_IDR2_25 ((uint32_t) 0x02000000)
#define IRQ_IDR2_26 ((uint32_t) 0x04000000)
#define IRQ_IDR2_27 ((uint32_t) 0x08000000)
#define IRQ_IDR2_28 ((uint32_t) 0x10000000)
#define IRQ_IDR2_29 ((uint32_t) 0x20000000)
#define IRQ_IDR2_30 ((uint32_t) 0x40000000)
#define IRQ_IDR2_31 ((uint32_t) 0x80000000)
#define IRQ_IDR2__I2C_SPI_SLV_INT ((uint32_t) 0x00000800)
#define IRQ_IDR2__PWA0 ((uint32_t) 0x00002000)
#define IRQ_IDR2__PWA1 ((uint32_t) 0x00004000)
#define IRQ_IDR2__SMI ((uint32_t) 0x00010000)
#define IRQ_IDR2__GPIO_INT_0 ((uint32_t) 0x00020000)
#define IRQ_IDR2__GPIO_INT_1 ((uint32_t) 0x00040000)
#define IRQ_IDR2__GPIO_INT_2 ((uint32_t) 0x00080000)
#define IRQ_IDR2__GPIO_INT_3 ((uint32_t) 0x00100000)
#define IRQ_IDR2__I2C_INT ((uint32_t) 0x00200000)
#define IRQ_IDR2__SPI_INT ((uint32_t) 0x00400000)
#define IRQ_IDR2__PCM_INT ((uint32_t) 0x00800000)
#define IRQ_IDR2__UART_INT ((uint32_t) 0x02000000)
/* Disable Basic IRQs (RW) */
#define IRQ_BASIC_IRQ_DISABLE_ARM_TIMER_OFS (0)
#define IRQ_BASIC_IRQ_DISABLE_ARM_TIMER ((uint32_t) 0x00000001)
#define IRQ_BASIC_IRQ_DISABLE_ARM_MAILBOX_OFS (1)
#define IRQ_BASIC_IRQ_DISABLE_ARM_MAILBOX ((uint32_t) 0x00000002)
#define IRQ_BASIC_IRQ_DISABLE_ARM_DOORBELL_0_OFS (2)
#define IRQ_BASIC_IRQ_DISABLE_ARM_DOORBELL_0 ((uint32_t) 0x00000004)
#define IRQ_BASIC_IRQ_DISABLE_ARM_DOORBELL_1_OFS (3)
#define IRQ_BASIC_IRQ_DISABLE_ARM_DOORBELL_1 ((uint32_t) 0x00000008)
#define IRQ_BASIC_IRQ_DISABLE_GPU0_HALTED_OFS (4)
#define IRQ_BASIC_IRQ_DISABLE_GPU0_HALTED ((uint32_t) 0x00000010)
#define IRQ_BASIC_IRQ_DISABLE_GPU1_HALTED_OFS (5)
#define IRQ_BASIC_IRQ_DISABLE_GPU1_HALTED ((uint32_t) 0x00000020)
#define IRQ_BASIC_IRQ_DISABLE_ACCESS_ERROR_T1_OFS (6)
#define IRQ_BASIC_IRQ_DISABLE_ACCESS_ERROR_T1 ((uint32_t) 0x00000040)
#define IRQ_BASIC_IRQ_DISABLE_ACCESS_ERROR_T0_OFS (7)
#define IRQ_BASIC_IRQ_DISABLE_ACCESS_ERROR_T0 ((uint32_t) 0x00000080)

/* AUX Bits */
/* AUX_IRQ - Auxiliary Interrupt Status */
/* AUX_ENABLES - Auxiliary enables */
/* AUX_MU_IO_REG - Mini UART I/O Data */
/* AUX_MU_IER_REG - Mini UART Interrupt Enable */
/* AUX_MU_LCR_REG - Mini UART Inerrupt Identify */
/* AUX_MU_MCR_REG - Mini UART Line Control */
/* AUX_MU_LSR_REG - Mini UART Line Status */
/* AUX_MU_MSR_REG - Mini UART Modem Status */
/* AUX_MU_SCRATCH - Mini UART Scratch */
/* AUX_MU_CNTL_REG - Mini UART Extra Control */
/* AUX_MU_STAT_REG - Mini UART Extra Status */
/* AUX_MU_BAUD_REG - Mini UART Baudrate */
/* AUX_SPI0_CNTL0_REG - SPI1 Control Register 0 */
/* AUX_SPI0_CNTL1_REG - SPI1 Control Register 1 */
/* AUX_SPI0_STAT_REG - SPI1 Status */
/* AUX_SPI0_IO_REG - SPI1 Data */
/* AUX_SPI0_PEEK_REG - SPI1 Peek */
/* AUX_SPI1_CNTL0_REG - SPI2 Control Register 0 */
/* AUX_SPI1_CNTL1_REG - SPI2 Control Register 1 */
/* AUX_SPI1_STAT_REG - SPI2 Status */
/* AUX_SPI1_IO_REG - SPI2 Data */
/* AUX_SPI1_PEEK_REG - SPI2 Peek */

/* SPI Bits */

/* PL011 UART Bits */

/* System Timer Bits */
/* 4 32-bit Timers and 1 64-bit Free-running counter */
/* CS - System Timer Control/Status Register (RW) */
#define SYSTIMER_CS_M0_OFS (0)
#define SYSTIMER_CS_M0 ((uint32_t) 0x00000001)
#define SYSTIMER_CS_M1_OFS (1)
#define SYSTIMER_CS_M1 ((uint32_t) 0x00000002)
#define SYSTIMER_CS_M2_OFS (2)
#define SYSTIMER_CS_M2 ((uint32_t) 0x00000004)
#define SYSTIMER_CS_M3_OFS (3)
#define SYSTIMER_CS_M3 ((uint32_t) 0x00000008)
/* CLO - System Timer Counter Lower 32 bits (R) */
#define SYSTIMER_CLO_CNT_OFS (0)
#define SYSTIMER_CLO_CNT_MASK ((uint32_t) 0xFFFFFFFF)
/* CHI - System Timer Counter Higher 32 bits (R) */
#define SYSTIMER_CHI_CNT_OFS (0)
#define SYSTIMER_CHI_CNT_MASK ((uint32_t) 0xFFFFFFFF)
/* C0 - System Timer Compare 0 */
#define SYSTIMER_C0_CMP_OFS (0)
#define SYSTIMER_C0_CMP_MASK ((uint32_t) 0xFFFFFFFF)
/* C1 - System Timer Compare 1 */
#define SYSTIMER_C1_CMP_OFS (0)
#define SYSTIMER_C1_CMP_MASK ((uint32_t) 0xFFFFFFFF)
/* C2 - System Timer Compare 2 */
#define SYSTIMER_C2_CMP_OFS (0)
#define SYSTIMER_C2_CMP_MASK ((uint32_t) 0xFFFFFFFF)
/* C3 - System Timer Compare 3 */
#define SYSTIMER_C3_CMP_OFS (0)
#define SYSTIMER_C3_CMP_MASK ((uint32_t) 0xFFFFFFFF)

/* Timer (ARM side) Register bits */
/* This Timer and free-running counter are running */
/* from the APB clock */
/* Load Register (W) */
#define ARMTIMER_LDR_OFS (0)
#define ARMTIMER_LDR_MASK ((uint32_t) 0xFFFFFFFF)

/* Value Register (R) */
#define ARMTIMER_RDR_OFS (0)
#define ARMTIMER_RDR_MASK ((uint32_t) 0xFFFFFFFF)

/* Control Register (RW) */
#define ARMTIMER_CTL_COUNTER_LEN_OFS (1)
#define ARMTIMER_CTL_COUNTER_LEN ((uint32_t) 0x00000002)
#define ARMTIMER_CTL_PRESCALE_OFS (2)
#define ARMTIMER_CTL_PRESCALE_MASK ((uint32_t) 0x0000000C)
#define ARMTIMER_CTL_PRESCALE0 ((uint32_t) 0x00000004)
#define ARMTIMER_CTL_PRESCALE1 ((uint32_t) 0x00000008)
#define ARMTIMER_CTL_PRESCALE_0 ((uint32_t) 0x00000000)
#define ARMTIMER_CTL_PRESCALE_1 ((uint32_t) 0x00000004)
#define ARMTIMER_CTL_PRESCALE_2 ((uint32_t) 0x00000008)
#define ARMTIMER_CTL_PRESCALE_3 ((uint32_t) 0x0000000C)
#define ARMTIMER_CTL_PRESCALE__NO_PRESCALE ((uint32_t) 0x00000000)
#define ARMTIMER_CTL_PRESCALE__16 ((uint32_t) 0x00000004)
#define ARMTIMER_CTL_PRESCALE__256 ((uint32_t) 0x00000008)
/* Undefined in SP804 */
#define ARMTIMER_CTL_PRESCALE__NO_PRESCALE1 ((uint32_t) 0x0000000C)
#define ARMTIMER_CTL_TIMER_IRQ_EN_OFS (5)
#define ARMTIMER_CTL_TIMER_IRQ_EN ((uint32_t) 0x00000020)
#define ARMTIMER_CTL_TIMER_EN_OFS (7)
#define ARMTIMER_CTL_TIMER_EN ((uint32_t) 0x00000080)
/* Undefined in SP804 */
#define ARMTIMER_CTL_DEBUG_HALTED_MODE_OFS (8)
#define ARMTIMER_CTL_DEBUG_HALTED_MODE ((uint32_t) 0x00000100)
/* Undefined in SP804 */
#define ARMTIMER_CTL_FRCTR_EN_OFS (9)
#define ARMTIMER_CTL_FRCTR_EN ((uint32_t) 0x00000200)
#define ARMTIMER_CTL_FRCTR_PRESCALE_OFS (16)
#define ARMTIMER_CTL_FRCTR_PRESCALE0 ((uint32_t) 0x00010000)
#define ARMTIMER_CTL_FRCTR_PRESCALE1 ((uint32_t) 0x00020000)
#define ARMTIMER_CTL_FRCTR_PRESCALE2 ((uint32_t) 0x00040000)
#define ARMTIMER_CTL_FRCTR_PRESCALE3 ((uint32_t) 0x00080000)
#define ARMTIMER_CTL_FRCTR_PRESCALE4 ((uint32_t) 0x00100000)
#define ARMTIMER_CTL_FRCTR_PRESCALE5 ((uint32_t) 0x00200000)
#define ARMTIMER_CTL_FRCTR_PRESCALE6 ((uint32_t) 0x00400000)
#define ARMTIMER_CTL_FRCTR_PRESCALE7 ((uint32_t) 0x00800000)
/* IRQ Clear/Ack Register (W) */
#define ARMTIMER_IRQ_CLR_OFS (0)
#define ARMTIMER_IRQ_CLR_MASK ((uint32_t) 0xFFFFFFFF)
/* RAW IRQ Register (R) */
#define ARMTIMER_IRQ_RAW_STATUS_OFS (0)
#define ARMTIMER_IRQ_RAW_STATUS ((uint32_t) 0x00000001)
/* Masked IRQ Register (R) */
#define ARMTIMER_IRQ_MASKED_STATUS_OFS (0)
#define ARMTIMER_IRQ_MASKED_STATUS ((uint32_t) 0x00000001)
/* Reload Regster (W) */
#define ARMTIMER_RLDR_OFS (0)
#define ARMTIMER_RLDR_MASK ((uint32_t) 0xFFFFFFFF)
/* Pre-divider Register (RW) */
#define ARMTIMER_PRE_DIVIDER_OFS (0)
#define ARMTIMER_PRE_DIVIDER_MASK ((uint32_t) 0x000003FF)
/* 32-bit Free-running counter Register (R) */
#define ARMTIMER_FRCTR_RDR_OFS (0)
#define ARMTIMER_FRCTR_RDR_MASK ((uint32_t) 0xFFFFFFFF)
/* BSC Bits (I2C single master only operation) */
/* EMMC Bits (External Mass Media Controller) */
/* PCM/I2S Audio Bits */
/* SPI/BSC Slave Bits */
/* USB Bits */

/* ARM Control Logic Module Registers */
/* 0x40000000: Control Register (RW) */
#define ARMCTL_CTL_CORETIMER_CLK_SRC_OFS (8)
#define ARMCTL_CTL_CORETIMER_CLK_SRC ((uint32_t) 0x00000100)
#define ARMCTL_CTL_CORETIMER_INCREMENTS_OFS (9)
#define ARMCTL_CTL_CORETIMER_INCREMENTS ((uint32_t) 0x00000200)
/* 0x40000008: Core Timer pre-scaler (RW) */
#define ARMCTL_CORETIMER_PRESCALER_OFS (0)
#define ARMCTL_CORETIMER_PRESCALER_MASK ((uint32_t) 0xFFFFFFFF)
/* 0x4000001C: Core Timer read: LS 32 bits */
/* Write: LS-32 holding register */
#define ARMCTL_CORETIMER_BITS_LO_OFS (0)
#define ARMCTL_CORETIMER_BITS_LO_MASK ((uint32_t) 0xFFFFFFFF)
/* 0x40000020: Core Timer read: Stored MS 32 bits register */
/* Write: MS 32 bits */
#define ARMCTL_CORETIMER_BITS_HI_OFS (0)
#define ARMCTL_CORETIMER_BITS_HI_MASK ((uint32_t) 0xFFFFFFFF)
/* 0x4000000C: GPU interrupt routing */
#define ARMCTL_GPU_IRQ_ROUTING_OFS (0)
#define ARMCTL_GPU_IRQ_ROUTING_MASK ((uint32_t) 0x00000003)
#define ARMCTL_GPU_IRQ_ROUTING0 ((uint32_t) 0x00000001)
#define ARMCTL_GPU_IRQ_ROUTING1 ((uint32_t) 0x00000002)
#define ARMCTL_GPU_IRQ_ROUTING_0 ((uint32_t) 0x00000000)
#define ARMCTL_GPU_IRQ_ROUTING_1 ((uint32_t) 0x00000001)
#define ARMCTL_GPU_IRQ_ROUTING_2 ((uint32_t) 0x00000002)
#define ARMCTL_GPU_IRQ_ROUTING_3 ((uint32_t) 0x00000003)
#define ARMCTL_GPU_IRQ_ROUTING__CORE0_IRQ_INPUT ((uint32_t) 0x00000000)
#define ARMCTL_GPU_IRQ_ROUTING__CORE1_IRQ_INPUT ((uint32_t) 0x00000001)
#define ARMCTL_GPU_IRQ_ROUTING__CORE2_IRQ_INPUT ((uint32_t) 0x00000002)
#define ARMCTL_GPU_IRQ_ROUTING__CORE3_IRQ_INPUT ((uint32_t) 0x00000003)
#define ARMCTL_GPU_FIQ_ROUTING_OFS (2)
#define ARMCTL_GPU_FIQ_ROUTING_MASK ((uint32_t) 0x0000000C)
#define ARMCTL_GPU_FIQ_ROUTING0 ((uint32_t) 0x00000004)
#define ARMCTL_GPU_FIQ_ROUTING1 ((uint32_t) 0x00000008)
#define ARMCTL_GPU_FIQ_ROUTING_0 ((uint32_t) 0x00000000)
#define ARMCTL_GPU_FIQ_ROUTING_1 ((uint32_t) 0x00000004)
#define ARMCTL_GPU_FIQ_ROUTING_2 ((uint32_t) 0x00000008)
#define ARMCTL_GPU_FIQ_ROUTING_3 ((uint32_t) 0x0000000C)
#define ARMCTL_GPU_FIQ_ROUTING__CORE0_FIQ_INPUT ((uint32_t) 0x00000000)
#define ARMCTL_GPU_FIQ_ROUTING__CORE1_FIQ_INPUT ((uint32_t) 0x00000004)
#define ARMCTL_GPU_FIQ_ROUTING__CORE2_FIQ_INPUT ((uint32_t) 0x00000008)
#define ARMCTL_GPU_FIQ_ROUTING__CORE3_FIQ_INPUT ((uint32_t) 0x0000000C)
/* 0x40000010 Performance monitor interrupt routing write-set */
/* Register */
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ0CTL_OFS (0)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ0CTL ((uint32_t) 0x00000001)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ1CTL_OFS (1)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ1CTL ((uint32_t) 0x00000002)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ2CTL_OFS (2)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ2CTL ((uint32_t) 0x00000004)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ3CTL_OFS (3)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ3CTL ((uint32_t) 0x00000008)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ0CTL_OFS (4)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ0CTL ((uint32_t) 0x00000010)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ1CTL_OFS (5)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ1CTL ((uint32_t) 0x00000020)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ2CTL_OFS (6)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ2CTL ((uint32_t) 0x00000040)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ3CTL_OFS (7)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ3CTL ((uint32_t) 0x00000080)
/* 0x40000014 Performance monitor interrupt routing write-clear */
/* Register */
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ0CTL_OFS (0)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ0CTL ((uint32_t) 0x00000001)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ1CTL_OFS (1)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ1CTL ((uint32_t) 0x00000002)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ2CTL_OFS (2)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ2CTL ((uint32_t) 0x00000004)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ3CTL_OFS (3)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ3CTL ((uint32_t) 0x00000008)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ0CTL_OFS (4)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ0CTL ((uint32_t) 0x00000010)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ1CTL_OFS (5)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ1CTL ((uint32_t) 0x00000020)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ2CTL_OFS (6)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ2CTL ((uint32_t) 0x00000040)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ3CTL_OFS (7)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ3CTL ((uint32_t) 0x00000080)
/* 0x40000040 Core0 Timers interrupt control */
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPS_IRQ_OFS (0)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPS_IRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPNS_IRQ_OFS (1)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPNS_IRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_THP_IRQ_OFS (2)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_THP_IRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TV_IRQ_OFS (3)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TV_IRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPS_FIQ_OFS (4)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPS_FIQ ((uint32_t) 0x00000010)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPNS_FIQ_OFS (5)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPNS_FIQ ((uint32_t) 0x00000020)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_THP_FIQ_OFS (6)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_THP_FIQ ((uint32_t) 0x00000040)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TV_FIQ_OFS (7)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TV_FIQ ((uint32_t) 0x00000080)
/* 0x40000044 Core1 Timers interrupt control */
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPS_IRQ_OFS (0)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPS_IRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPNS_IRQ_OFS (1)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPNS_IRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_THP_IRQ_OFS (2)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_THP_IRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TV_IRQ_OFS (3)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TV_IRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPS_FIQ_OFS (4)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPS_FIQ ((uint32_t) 0x00000010)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPNS_FIQ_OFS (5)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPNS_FIQ ((uint32_t) 0x00000020)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_THP_FIQ_OFS (6)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_THP_FIQ ((uint32_t) 0x00000040)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TV_FIQ_OFS (7)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TV_FIQ ((uint32_t) 0x00000080)
/* 0x40000048 Core2 Timers interrupt control */
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPS_IRQ_OFS (0)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPS_IRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPNS_IRQ_OFS (1)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPNS_IRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_THP_IRQ_OFS (2)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_THP_IRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TV_IRQ_OFS (3)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TV_IRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPS_FIQ_OFS (4)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPS_FIQ ((uint32_t) 0x00000010)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPNS_FIQ_OFS (5)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPNS_FIQ ((uint32_t) 0x00000020)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_THP_FIQ_OFS (6)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_THP_FIQ ((uint32_t) 0x00000040)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TV_FIQ_OFS (7)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TV_FIQ ((uint32_t) 0x00000080)
/* 0x4000004C Core3 Timers interrupt control */
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPS_IRQ_OFS (0)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPS_IRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPNS_IRQ_OFS (1)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPNS_IRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_THP_IRQ_OFS (2)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_THP_IRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TV_IRQ_OFS (3)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TV_IRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPS_FIQ_OFS (4)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPS_FIQ ((uint32_t) 0x00000010)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPNS_FIQ_OFS (5)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPNS_FIQ ((uint32_t) 0x00000020)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_THP_FIQ_OFS (6)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_THP_FIQ ((uint32_t) 0x00000040)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TV_FIQ_OFS (7)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TV_FIQ ((uint32_t) 0x00000080)
/* 0x40000050 Core0 Mailboxes interrupt control */
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB0IRQ_OFS (0)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB0IRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB1IRQ_OFS (1)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB1IRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB2IRQ_OFS (2)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB2IRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB3IRQ_OFS (3)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB3IRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB0FIQ_OFS (4)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB0FIQ ((uint32_t) 0x00000010)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB1FIQ_OFS (5)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB1FIQ ((uint32_t) 0x00000020)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB2FIQ_OFS (6)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB2FIQ ((uint32_t) 0x00000040)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB3FIQ_OFS (7)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB3FIQ ((uint32_t) 0x00000080)
/* 0x40000054 Core1 Mailboxes interrupt control */
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB0IRQ_OFS (0)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB0IRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB1IRQ_OFS (1)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB1IRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB2IRQ_OFS (2)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB2IRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB3IRQ_OFS (3)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB3IRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB0FIQ_OFS (4)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB0FIQ ((uint32_t) 0x00000010)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB1FIQ_OFS (5)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB1FIQ ((uint32_t) 0x00000020)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB2FIQ_OFS (6)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB2FIQ ((uint32_t) 0x00000040)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB3FIQ_OFS (7)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB3FIQ ((uint32_t) 0x00000080)
/* 0x40000058 Core2 Mailboxes interrupt control */
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB0IRQ_OFS (0)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB0IRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB1IRQ_OFS (1)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB1IRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB2IRQ_OFS (2)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB2IRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB3IRQ_OFS (3)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB3IRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB0FIQ_OFS (4)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB0FIQ ((uint32_t) 0x00000010)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB1FIQ_OFS (5)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB1FIQ ((uint32_t) 0x00000020)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB2FIQ_OFS (6)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB2FIQ ((uint32_t) 0x00000040)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB3FIQ_OFS (7)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB3FIQ ((uint32_t) 0x00000080)
/* 0x4000005C Core3 Mailboxes interrupt control */
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB0IRQ_OFS (0)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB0IRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB1IRQ_OFS (1)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB1IRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB2IRQ_OFS (2)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB2IRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB3IRQ_OFS (3)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB3IRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB0FIQ_OFS (4)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB0FIQ ((uint32_t) 0x00000010)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB1FIQ_OFS (5)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB1FIQ ((uint32_t) 0x00000020)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB2FIQ_OFS (6)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB2FIQ ((uint32_t) 0x00000040)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB3FIQ_OFS (7)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB3FIQ ((uint32_t) 0x00000080)
/* 0x40000080 Core0 Mailbox 0 write-to-set (W) */
/* 0x40000084 Core0 Mailbox 1 write-to-set (W) */
/* 0x40000088 Core0 Mailbox 2 write-to-set (W) */
/* 0x4000008C Core0 Mailbox 3 write-to-set (W) */
/* 0x40000090 Core1 Mailbox 0 write-to-set (W) */
/* 0x40000094 Core1 Mailbox 1 write-to-set (W) */
/* 0x40000098 Core1 Mailbox 2 write-to-set (W) */
/* 0x4000009C Core1 Mailbox 3 write-to-set (W) */
/* 0x400000A0 Core2 Mailbox 0 write-to-set (W) */
/* 0x400000A4 Core2 Mailbox 1 write-to-set (W) */
/* 0x400000A8 Core2 Mailbox 2 write-to-set (W) */
/* 0x400000AC Core2 Mailbox 3 write-to-set (W) */
/* 0x400000B0 Core3 Mailbox 0 write-to-set (W) */
/* 0x400000B4 Core3 Mailbox 1 write-to-set (W) */
/* 0x400000B8 Core3 Mailbox 2 write-to-set (W) */
/* 0x400000BC Core3 Mailbox 3 write-to-set (W) */
/* 0x400000C0 Core0 Mailbox 0 write-to-clear (RW) */
/* 0x400000C4 Core0 Mailbox 1 write-to-clear (RW) */
/* 0x400000C8 Core0 Mailbox 2 write-to-clear (RW) */
/* 0x400000CC Core0 Mailbox 3 write-to-clear (RW) */
/* 0x400000D0 Core1 Mailbox 0 write-to-clear (RW) */
/* 0x400000D4 Core1 Mailbox 1 write-to-clear (RW) */
/* 0x400000D8 Core1 Mailbox 2 write-to-clear (RW) */
/* 0x400000DC Core1 Mailbox 3 write-to-clear (RW) */
/* 0x400000E0 Core2 Mailbox 0 write-to-clear (RW) */
/* 0x400000E4 Core2 Mailbox 1 write-to-clear (RW) */
/* 0x400000E8 Core2 Mailbox 2 write-to-clear (RW) */
/* 0x400000EC Core2 Mailbox 3 write-to-clear (RW) */
/* 0x400000F0 Core3 Mailbox 0 write-to-clear (RW) */
/* 0x400000F4 Core3 Mailbox 1 write-to-clear (RW) */
/* 0x400000F8 Core3 Mailbox 2 write-to-clear (RW) */
/* 0x400000FC Core3 Mailbox 3 write-to-clear (RW) */
/* 0x4000002C AXI outstanding counters */
#define ARMCTL_AXI_OUTSTANDING_COUNTERS_READS_OFS (0)
#define ARMCTL_AXI_OUTSTANDING_COUNTERS_READS_MASK ((uint32_t) 0x000003FF)
#define ARMCTL_AXI_OUTSTANDING_COUNTERS_WRITES_OFS (16)
#define ARMCTL_AXI_OUTSTANDING_COUNTERS_WRITES_MASK ((uint32_t) 0x03FF0000)
/* 0x40000030 AXI outstanding interrupt */
#define ARMCTL_AXI_OUTSTANDING_IRQ_TIMEOUT_OFS (0)
#define ARMCTL_AXI_OUTSTANDING_IRQ_TIMEOUT_MASK ((uint32_t) 0x000FFFFF)
#define ARMCTL_AXI_OUTSTANDING_IRQ_ENABLE_OFS (20)
#define ARMCTL_AXI_OUTSTANDING_IRQ_ENABLE ((uint32_t) 0x00100000)
/* 0x40000060 Core0 interrupt source */
#define ARMCTL_CORE0_IRQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE0_IRQ_SRC_TPSIRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE0_IRQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE0_IRQ_SRC_TPNSIRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE0_IRQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE0_IRQ_SRC_THPIRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE0_IRQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE0_IRQ_SRC_TVIRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX0 ((uint32_t) 0x00000010)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX1 ((uint32_t) 0x00000020)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX2 ((uint32_t) 0x00000040)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX3 ((uint32_t) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE0_IRQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE0_IRQ_SRC_GPU ((uint32_t) 0x00000100)
#define ARMCTL_CORE0_IRQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE0_IRQ_SRC_PMU ((uint32_t) 0x00000200)
/* AXI IRQ for core 0 only, all others are 0 */
#define ARMCTL_CORE0_IRQ_SRC_AXI_OFS (10)
#define ARMCTL_CORE0_IRQ_SRC_AXI ((uint32_t) 0x00000400)
#define ARMCTL_CORE0_IRQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE0_IRQ_SRC_LOCALTIMER ((uint32_t) 0x00000800)
#define ARMCTL_CORE0_IRQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE0_IRQ_SRC_PERIPH ((uint32_t) 0x00001000)

/* 0x40000064 Core1 interrupt source */
#define ARMCTL_CORE1_IRQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE1_IRQ_SRC_TPSIRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE1_IRQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE1_IRQ_SRC_TPNSIRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE1_IRQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE1_IRQ_SRC_THPIRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE1_IRQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE1_IRQ_SRC_TVIRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX0 ((uint32_t) 0x00000010)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX1 ((uint32_t) 0x00000020)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX2 ((uint32_t) 0x00000040)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX3 ((uint32_t) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE1_IRQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE1_IRQ_SRC_GPU ((uint32_t) 0x00000100)
#define ARMCTL_CORE1_IRQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE1_IRQ_SRC_PMU ((uint32_t) 0x00000200)
/* AXI IRQ for core 0 only, all others are 0 */
#define ARMCTL_CORE1_IRQ_SRC_AXI_OFS (10)
#define ARMCTL_CORE1_IRQ_SRC_AXI ((uint32_t) 0x00000400)
#define ARMCTL_CORE1_IRQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE1_IRQ_SRC_LOCALTIMER ((uint32_t) 0x00000800)
#define ARMCTL_CORE1_IRQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE1_IRQ_SRC_PERIPH ((uint32_t) 0x00001000)

/* 0x40000068 Core2 interrupt source */
#define ARMCTL_CORE2_IRQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE2_IRQ_SRC_TPSIRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE2_IRQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE2_IRQ_SRC_TPNSIRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE2_IRQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE2_IRQ_SRC_THPIRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE2_IRQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE2_IRQ_SRC_TVIRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX0 ((uint32_t) 0x00000010)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX1 ((uint32_t) 0x00000020)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX2 ((uint32_t) 0x00000040)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX3 ((uint32_t) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE2_IRQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE2_IRQ_SRC_GPU ((uint32_t) 0x00000100)
#define ARMCTL_CORE2_IRQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE2_IRQ_SRC_PMU ((uint32_t) 0x00000200)
/* AXI IRQ for core 0 only, all others are 0 */
#define ARMCTL_CORE2_IRQ_SRC_AXI_OFS (10)
#define ARMCTL_CORE2_IRQ_SRC_AXI ((uint32_t) 0x00000400)
#define ARMCTL_CORE2_IRQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE2_IRQ_SRC_LOCALTIMER ((uint32_t) 0x00000800)
#define ARMCTL_CORE2_IRQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE2_IRQ_SRC_PERIPH ((uint32_t) 0x00001000)

/* 0x4000006C Core3 interrupt source */
#define ARMCTL_CORE3_IRQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE3_IRQ_SRC_TPSIRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE3_IRQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE3_IRQ_SRC_TPNSIRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE3_IRQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE3_IRQ_SRC_THPIRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE3_IRQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE3_IRQ_SRC_TVIRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX0 ((uint32_t) 0x00000010)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX1 ((uint32_t) 0x00000020)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX2 ((uint32_t) 0x00000040)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX3 ((uint32_t) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE3_IRQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE3_IRQ_SRC_GPU ((uint32_t) 0x00000100)
#define ARMCTL_CORE3_IRQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE3_IRQ_SRC_PMU ((uint32_t) 0x00000200)
/* AXI IRQ for core 0 only, all others are 0 */
#define ARMCTL_CORE3_IRQ_SRC_AXI_OFS (10)
#define ARMCTL_CORE3_IRQ_SRC_AXI ((uint32_t) 0x00000400)
#define ARMCTL_CORE3_IRQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE3_IRQ_SRC_LOCALTIMER ((uint32_t) 0x00000800)
#define ARMCTL_CORE3_IRQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE3_IRQ_SRC_PERIPH ((uint32_t) 0x00001000)

/* 0x40000070 Core0 fast interrupt source */
#define ARMCTL_CORE0_FIQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE0_FIQ_SRC_TPSIRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE0_FIQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE0_FIQ_SRC_TPNSIRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE0_FIQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE0_FIQ_SRC_THPIRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE0_FIQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE0_FIQ_SRC_TVIRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX0 ((uint32_t) 0x00000010)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX1 ((uint32_t) 0x00000020)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX2 ((uint32_t) 0x00000040)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX3 ((uint32_t) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE0_FIQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE0_FIQ_SRC_GPU ((uint32_t) 0x00000100)
#define ARMCTL_CORE0_FIQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE0_FIQ_SRC_PMU ((uint32_t) 0x00000200)
#define ARMCTL_CORE0_FIQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE0_FIQ_SRC_LOCALTIMER ((uint32_t) 0x00000800)
#define ARMCTL_CORE0_FIQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE0_FIQ_SRC_PERIPH ((uint32_t) 0x00001000)

/* 0x40000074 Core1 fast interrupt source */
#define ARMCTL_CORE1_FIQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE1_FIQ_SRC_TPSIRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE1_FIQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE1_FIQ_SRC_TPNSIRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE1_FIQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE1_FIQ_SRC_THPIRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE1_FIQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE1_FIQ_SRC_TVIRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX0 ((uint32_t) 0x00000010)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX1 ((uint32_t) 0x00000020)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX2 ((uint32_t) 0x00000040)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX3 ((uint32_t) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE1_FIQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE1_FIQ_SRC_GPU ((uint32_t) 0x00000100)
#define ARMCTL_CORE1_FIQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE1_FIQ_SRC_PMU ((uint32_t) 0x00000200)
#define ARMCTL_CORE1_FIQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE1_FIQ_SRC_LOCALTIMER ((uint32_t) 0x00000800)
#define ARMCTL_CORE1_FIQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE1_FIQ_SRC_PERIPH ((uint32_t) 0x00001000)

/* 0x40000078 Core2 fast interrupt source */
#define ARMCTL_CORE2_FIQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE2_FIQ_SRC_TPSIRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE2_FIQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE2_FIQ_SRC_TPNSIRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE2_FIQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE2_FIQ_SRC_THPIRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE2_FIQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE2_FIQ_SRC_TVIRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX0 ((uint32_t) 0x00000010)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX1 ((uint32_t) 0x00000020)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX2 ((uint32_t) 0x00000040)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX3 ((uint32_t) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE2_FIQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE2_FIQ_SRC_GPU ((uint32_t) 0x00000100)
#define ARMCTL_CORE2_FIQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE2_FIQ_SRC_PMU ((uint32_t) 0x00000200)
#define ARMCTL_CORE2_FIQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE2_FIQ_SRC_LOCALTIMER ((uint32_t) 0x00000800)
#define ARMCTL_CORE2_FIQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE2_FIQ_SRC_PERIPH ((uint32_t) 0x00001000)

/* 0x4000007C Core3 fast interrupt source */
#define ARMCTL_CORE3_FIQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE3_FIQ_SRC_TPSIRQ ((uint32_t) 0x00000001)
#define ARMCTL_CORE3_FIQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE3_FIQ_SRC_TPNSIRQ ((uint32_t) 0x00000002)
#define ARMCTL_CORE3_FIQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE3_FIQ_SRC_THPIRQ ((uint32_t) 0x00000004)
#define ARMCTL_CORE3_FIQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE3_FIQ_SRC_TVIRQ ((uint32_t) 0x00000008)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX0 ((uint32_t) 0x00000010)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX1 ((uint32_t) 0x00000020)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX2 ((uint32_t) 0x00000040)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX3 ((uint32_t) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE3_FIQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE3_FIQ_SRC_GPU ((uint32_t) 0x00000100)
#define ARMCTL_CORE3_FIQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE3_FIQ_SRC_PMU ((uint32_t) 0x00000200)
#define ARMCTL_CORE3_FIQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE3_FIQ_SRC_LOCALTIMER ((uint32_t) 0x00000800)
#define ARMCTL_CORE3_FIQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE3_FIQ_SRC_PERIPH ((uint32_t) 0x00001000)

/* 0x40000034 Local Timer Control & Status */
#define ARMCTL_LTCSR_RELOADVAL_OFS (0)
#define ARMCTL_LTCSR_RELOADVAL_MASK ((uint32_t) 0x0FFFFFFF)
#define ARMCTL_LTCSR_TIMER_ENABLE_OFS (28)
#define ARMCTL_LTCSR_TIMER_ENABLE ((uint32_t) 0x10000000)
#define ARMCTL_LTCSR_IRQ_ENABLE_OFS (29)
#define ARMCTL_LTCSR_IRQ_ENABLE ((uint32_t) 0x20000000)
#define ARMCTL_LTCSR_IRQ_FLAG_OFS (31)
#define ARMCTL_LTCSR_IRQ_FLAG ((uint32_t) 0x80000000)
/* 0x40000038 Local Timer IRQ Clear & Reload (W) */
#define ARMCTL_LTWFR_RELOAD_OFS (30)
#define ARMCTL_LTWFR_RELOAD ((uint32_t) 0x40000000)
#define ARMCTL_LTWFR_IRQ_FLAG_CLR_OFS (31)
#define ARMCTL_LTWFR_IRQ_FLAG_CLR ((uint32_t) 0x80000000)
/* 0x40000024 Local Interrupt Routing */
/* Local Timer is the only local interrupt source present */
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_OFS (0)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_MASK ((uint32_t) 0x00000007)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST0 ((uint32_t) 0x00000001)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST1 ((uint32_t) 0x00000002)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST2 ((uint32_t) 0x00000004)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_0 ((uint32_t) 0x00000000)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_1 ((uint32_t) 0x00000001)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_2 ((uint32_t) 0x00000002)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_3 ((uint32_t) 0x00000003)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_4 ((uint32_t) 0x00000004)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_5 ((uint32_t) 0x00000005)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_6 ((uint32_t) 0x00000006)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_7 ((uint32_t) 0x00000007)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE0IRQ ((uint32_t) 0x00000000)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE1IRQ ((uint32_t) 0x00000001)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE2IRQ ((uint32_t) 0x00000002)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE3IRQ ((uint32_t) 0x00000003)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE0FIQ ((uint32_t) 0x00000004)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE1FIQ ((uint32_t) 0x00000005)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE2FIQ ((uint32_t) 0x00000006)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE3FIQ ((uint32_t) 0x00000007)

#endif /* RPI_H_ */
