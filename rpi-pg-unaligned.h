#ifndef RPI_H_
#define RPI_H_

/* Raspberry Pi 3 Model B Rev 1.2 hardware header */
/* for device driver peripheral access (4kbyte page unaligned) */

/* Use standard integer types with explicit width */
/*#include "stdint.h"*/

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
/*
typedef enum IRQn
{*/
   /* Cortex-M4 Processor Exceptions Numbers */
   /*
   NonMaskableInt_IRQn = -14,
   HardFault_IRQn = -13,
   MemoryManagement = -12,
   BusFault = -11,
   UsageFault = -10,
   SVCall_IRQn = -5,
   DebugMonitor_IRQn = -4,
   PendSV_IRQn = -2,
   SysTick_IRQn = -1,*/
   /*Peripheral Exceptions Numbers */
/*
   GPU_IRQ0 = 0,
   GPU_IRQ1 = 1
} IRQn_Type;*/

/* Definition of standard bits */
/*
#define BIT0 (uint16_t) (0x0001)
#define BIT1 (uint16_t) (0x0002)
#define BIT2 (uint16_t) (0x0004)
#define BIT3 (uint16_t) (0x0008)
#define BIT4 (uint16_t) (0x0010)
#define BIT5 (uint16_t) (0x0020)
#define BIT6 (uint16_t) (0x0040)
#define BIT7 (uint16_t) (0x0080)
#define BIT(x) ((uint16_t)1 << (x))
*/

/* Device and peripheral memory map */
#define FLASH_BASE ((u32) 0x00000000)
#define PERIPH_BASE ((u32) 0x3F000000)
#define LOCAL_PERIPH_BASE ((u32) 0x40000000)

/* Auxiliaries: UART1 & SPI1, SPI2 */
#define AUX_BASE ((u32) PERIPH_BASE + 0x00215000)

/* Broadcom Serial Controller I2C single master only operation */
/* BSC2 Master is dedicated to the HDMI interface and should not */
/* be accessed by other programs */
#define BSC0_MASTER_BASE ((u32) PERIPH_BASE + 0x00205000)
#define BSC1_MASTER_BASE ((u32) PERIPH_BASE + 0x00804000)
#define BSC2_MASTER_BASE ((u32) PERIPH_BASE + 0x00805000)

/* DMA Controller comprised of 16 DMA Channels */
/* Adjacent DMA Channels are offset by 0x100 */
/* DMA Channel 15 is physically removed from other DMA Channels */
#define DMA0_DMA14_BASE ((u32) PERIPH_BASE + 0x00007000)
#define DMA15_BASE ((u32) PERIPH_BASE + 0x00E05000)

/* External Mass Meda Controller */
/* Embedded Multimedia(TM) and SD(TM) card interface by Arasan(TM) */
#define EMMC_BASE ((u32) PERIPH_BASE + 0x00300000)

/* GPIO (General Purpose I/O) supports 54 GPIO lines */
#define GPIO_BASE ((u32) PERIPH_BASE + 0x00200000)

/* Base address for the ARM interrupt register */
#define IRQ_BASE ((u32) PERIPH_BASE + 0x0000B200)

/* PCM Audio Interface */
#define PCM_BASE ((u32) PERIPH_BASE + 0x00203000)

/* Pulse Width Modulator (PWM) */
/*#define PWM_BASE ((u32) PERIPH_BASE + 0x0020C000)*/
/* Address in memory not specified in BCM2835 reference manual */

/* Auxiliaries: UART1 & SPI1, SPI2 */
#define AUX_BASE ((u32) PERIPH_BASE + 0x00215000)
/* SPI (Serial Peripheral Interface) */
#define SPI_BASE ((u32) PERIPH_BASE + 0x00204000)

/* BSC/SPI slave */
#define BSC_SPI_SLAVE_BASE ((u32) PERIPH_BASE + 0x00214000)

/* System Timer */
#define SYSTIMER_BASE ((u32) PERIPH_BASE + 0x00003000)

/* PL011 UART */
/* #define PL011_USRT_BASE ((u32) PERIPH_BASE + 0x00201000) */

/* Timer (ARM side) */
/*Page aligned*/
/*#define ARM_TIMER_BASE ((u32) PERIPH_BASE + 0x0000B000)*/
/*Page unaligned*/
#define ARM_TIMER_BASE ((u32) PERIPH_BASE + 0x0000B400)

/* USB */
#define USB_BASE ((u32) PERIPH_BASE + 0x00980080)

/* GPIO Clocks Clock Manager */
#define CM_BASE ((u32) PERIPH_BASE + 0x00101000)

/* GPIO Pads drive control */
#define GPIO_PADS_CTL_BASE ((u32) PERIPH_BASE + 0x00100000)

/* Peripheral register definitions */
/* Device specific Peripheral register structures */

typedef struct {
   /* GPIO Function Select 0 */
   __IO u32 GPFSEL0;
   /* GPIO Function Select 1 */
   __IO u32 GPFSEL1;
   /* GPIO Function Select 2 */
   __IO u32 GPFSEL2;
   /* GPIO Function Select 3 */
   __IO u32 GPFSEL3;
   /* GPIO Function Select 4 */
   __IO u32 GPFSEL4;
   /* GPIO Function Select 5 */
   __IO u32 GPFSEL5;
   u32 RESERVED0;
   /* GPIO Pin Output Set 0 */
   __O u32 GPSET0;
   /* GPIO Pin Output Set 1 */
   __O u32 GPSET1;
   u32 RESERVED1;
   /* GPIO Pin Output Clear 0 */
   __O u32 GPCLR0;
   /* GPIO Pin Output Clear 1 */
   __O u32 GPCLR1;
   u32 RESERVED2;
   /* GPIO Pin Level 0 */
   __I u32 GPLEV0;
   /* GPIO Pin Level 1 */
   __I u32 GPLEV1;
   u32 RESERVED3;
   /* GPIO Pin Event Detect Status 0 */
   __IO u32 GPEDS0;
   /* GPIO Pin Event Detect Status 1 */
   __IO u32 GPEDS1;
   u32 RESERVED4;
   /* GPIO Pin Rising Edge Detect Enable 0 */
   __IO u32 GPREN0;
   /* GPIO Pin Rising Edge Detect Enable 1 */
   __IO u32 GPREN1;
   u32 RESERVED5;
   /* GPIO Pin Falling Edge Detect Enable 0 */
   __IO u32 GPFEN0;
   /* GPIO Pin Falling Edge Detect Enable 1 */
   __IO u32 GPFEN1;
   u32 RESERVED6;
   /* GPIO Pin High Detect Enable 0 */
   __IO u32 GPHEN0;
   /* GPIO Pin High Detect Enable 1 */
   __IO u32 GPHEN1;
   u32 RESERVED7;
   /* GPIO Pin Low Detect Enable 0 */
   __IO u32 GPLEN0;
   /* GPIO Pin Low Detect Enable 1 */
   __IO u32 GPLEN1;
   u32 RESERVED8;
   /* GPIO Pin Async. Rising Edge Detect 0 */
   __IO u32 GPAREN0;
   /* GPIO Pin Async. Rising Edge Detect 1 */
   __IO u32 GPAREN1;
   u32 RESERVED9;
   /* GPIO Pin Async. Falling Edge Detect 0 */
   __IO u32 GPAFEN0;
   /* GPIO Pin Async. Falling Edge Detect 1 */
   __IO u32 GPAFEN1;
   u32 RESERVED10;
   /* GPIO Pin Pull-up/down Enable */
   __IO u32 GPPUD;
   /* GPIO Pin Pull-up/down Enable Clock 0 */
   __IO u32 GPPUDCLK0;
   /* GPIO Pin Pull-up/down Enable Clock 1 */
   __IO u32 GPPUDCLK1;
   u32 RESERVED11; 
   u32 RESERVED12; 
   u32 RESERVED13; 
   u32 RESERVED14; 
   __IO u32 TEST;
} GPIO_Type;

typedef struct {
   /* 0x2C bytes for */
   /* Page aligned boundary to start at PERIPH_BASE + 0x00100000 */
   __IO u32 RESERVED[11];
   __IO u32 PADS0_27;
   __IO u32 PADS28_45;
   __IO u32 PADS46_53;   
} GPIO_Pads_Control_Type;

typedef struct {
   /* 0x70 bytes for */
   /* Page aligned boundary to start at PERIPH_BASE + 0x00101000) */
   __IO u32 RESERVED[28];
   /* General Purpose Clock Control 0*/
   __IO u32 GPCTL0;
   /* General Purpose Clock Divisor 0*/
   __IO u32 GPDIV0;
   /* General Purpose Clock Control 1*/
   __IO u32 GPCTL1;
   /* General Purpose Clock Divisor 1*/
   __IO u32 GPDIV1;
   /* General Purpose Clock Control 2*/
   __IO u32 GPCTL2;
   /* General Purpose Clock Divisor 2*/
   __IO u32 GPDIV2;
} CM_Type; 

typedef struct {
   /* Control and Status register */
   __IO u32 CS;
   /* Conrol Block Address register */
   __IO u32 CONBLK_AD;
   /* Control Block Word 0 (Transfer Information) */
   __I u32 TI;
   /* Control Block Word 1 (Source Address) */
   __I u32 SOURCE_AD;
   /* Control Block Word 2 (Destination Address) */
   __I u32 DEST_AD;
   /* Control Block Word 3 (Transfer Length) */
   __I u32 TXFR_LEN;
   /* Control Block Word 4 (2D Stride) */
   __I u32 STRIDE;
   /* Control Block Word 5 (Next Control Block Address) */
   __I u32 NEXTCONBK;
   /* Debug */
   __IO u32 DEBUG;
} DMA_Channel_Type;

typedef struct {
   /* Interrupt status of each DMA channel */
   __IO u32 INT_STATUS;
   u32 RESERVED0;
   u32 RESERVED1;
   u32 RESERVED2;
   /* Global enable bits for each DMA channel */
   __IO u32 ENABLE; 
} DMA_Global_Type;

typedef struct {
   /* Transfer Information */
   u32 TI;
   /* Source Address */
   u32 SOURCE_AD;
   /* Destination Address */
   u32 DEST_AD;
   /* Transfer Length */
   u32 TXFR_LEN;
   /* 2D Mode Stride */
   u32 STRIDE;
   /* Next Control Block Address */
   u32 NEXTCONBK;
   u32 RESERVED0;
   u32 RESERVED1;
} DMA_Control_Block_Type;

typedef struct {
   /* Interrupt Basic Pending */
   __I u32 IRQBP;
   /* Interrupt Pending 1 */
   __I u32 IRQP1;
   /* Interrupt Pending 2 */
   __I u32 IRQP2;
   /* FIQ control */
   __IO u32 FIQCNTL;
   /* Enable Interrupts 1 */
   __IO u32 IRQEN1;
   /* Enable Interrupts 2 */
   __IO u32 IRQEN2;
   /* Enable Basic Interrupts */
   __IO u32 BASIC_IRQEN;
   /* Disable Interrupts 1 */
   __IO u32 DISABLE_IRQ1;
   /* Disable Interrupts 2 */
   __IO u32 DISABLE_IRQ2;
   /* Disable Basic Interrupts */
   __IO u32 DISABLE_BASIC_IRQ;
} Interrupt_Controller_Type;

typedef struct {
   /* PWM Control */
   __IO u32 CTL;
   /* PWM Status */
   __IO u32 STA;
   /* PWM DMA Configuration */
   __IO u32 DMAC;
   /* PWM Channel 1 Range */
   __IO u32 RNG1;
   /* PWM Channel 1 Data */
   __IO u32 DAT1;
   /* PWM FIFO Input */
   __IO u32 FIF1;
   /* PWM Channel 2 Range */
   __IO u32 RNG2;
   /* PWM Channel 2 Data */
   __IO u32 DAT2;
} PWM_Type;

typedef struct {
   /* Main control and status bits for the SPI */
   __IO u32 CS;
   /* FIFO TX/RX buffer */ 
   __IO u32 FIFO;
   /* SPI clock rate */
   __IO u32 CLK;
   /* SPI data length rate */
   u32 DLEN;
   /* LoSSI output hold delay */
   u32 LTOH;
   /* Generation of DREQ and Panic signals */
   /* to an external DMA */
   u32 DC;
} SPI_Type;

typedef struct {

} DIO_Type;

typedef struct {
   /* Auxiliary Interrupt status */
   __I u32 AUX_IRQ;
   /* Auxiliary enables */
   __IO u32 AUX_ENABLES;
   /* Mini UART I/O Data */
   __IO u32 AUX_MU_IO_REG;
   /* Mini UART Interrupt Enable */
   __IO u32 AUX_MU_IER_REG;
   /* Mini UART Interrupt Identify */
   __IO u32 AUX_MU_IIR_REG;
   /* Mini UART Line Control */
   __IO u32 AUX_MU_LCR_REG;
   /* Mini UART Modem Control */
   __IO u32 AUX_MU_MCR_REG;
   /* Mini UART Line Status */
   __I u32 AUX_MU_LSR_REG;
   /* Mini UART Modem Status */
   __I u32 AUX_MU_MSR_REG;
   /* Mini UART Scratch */
   __IO u32 AUX_MU_SCRATCH;
   /* Mini UART Extra Control */
   __IO u32 AUX_MU_CNTL_REG;
   /* Mini UART Extra Status */
   __I u32 AUX_MU_STAT_REG;
   /* Mini UART Baudrate */
   __IO u32 AUX_MU_BAUD_REG;
   /* SPI 1 Control Register 0 */
   __IO u32 AUX_SPI0_CNTL0_REG;
   /* SPI 1 Control Register 1 */
   __IO u32 AUX_SPI0_CNTL1_REG;
   /* SPI 1 Status */
   __I u32 AUX_SPI0_STAT_REG;
   /* SPI 1 Peek */
   __I u32 AUX_SPI0_PEEK_REG;
   /* SPI 1 Data */
   /* These four addresses all write to the same FIFO */
   /* Writing to any of these registers causes the SPI CS_n pins */
   /* To be de-asserted at the end of the access */
   __IO u32 AUX_SPI0_IO_REG;
   u32 RESERVED0;
   u32 RESERVED1;
   u32 RESERVED2;
   /* Extended CS port of the SPI interfaces */
   /* These four addresses all write to the same FIFO */
   /* Writing to these addresses causes the SPI CS_n pins to */
   /* remain asserted at the end of the access */
   __IO u32 AUX_SPI0_TXHOLD_REG;
   u32 RESERVED3;
   u32 RESERVED4;
   u32 RESERVED5;
   /* SPI 2 Control register 0 */
   __IO u32 AUX_SPI1_CNTL0_REG;
   /* SPI 2 Control register 1 */
   __IO u32 AUX_SPI1_CNTL1_REG;
   /* SPI 2 Status */
   __I u32 AUX_SPI1_STAT_REG;
   /* SPI 2 Peek */
   __I u32 AUX_SPI1_PEEK_REG;
   /* SPI 2 Data */
   /* These four addresses all write to the same FIFO */
   /* Writing to any of these registers causes the SPI CS_n pins */
   /* To be de-asserted at the end of the access */
   __IO u32 AUX_SPI1_IO_REG;
   u32 RESERVED6;
   u32 RESERVED7;
   u32 RESERVED8;
   /* Extended CS port of the SPI interfaces */
   /* These four addresses all write to the same FIFO */
   /* Writing to these addresses causes the SPI CS_n pins to */
   /* remain asserted at the end of the access */
   __IO u32 AUX_SPI1_TXHOLD_REG;
   u32 RESERVED9;
   u32 RESERVED10;
   u32 RESERVED11;
} AUX_Type;

typedef struct {
   /* System Timer Control/Status */
   __IO u32 CS;
   /* System Timer Counter Lower 32 bits */
   __I u32 CLO;
   /* System Timer Counter Higher 32 bits */
   __I u32 CHI;
   /* System Timer Compare 0 */
   __IO u32 C0;
   /* System Timer Compare 1 */
   __IO u32 C1;
   /* System Timer Compare 2 */
   __IO u32 C2;
   /* System Timer Compare 3 */
   __IO u32 C3;
} SysTimer_Type;

typedef struct {
   /* 0x400 bytes for */
   /* Page-aligned boundary to start at PERIPH_BASE + 0x0000B000 */
   /*__IO u32 RESERVED0[256];*/
   /* Timer Load Register */
   __IO u32 LD;
   /* Timer Value Register (R) */
   __I u32 VAL;
   /* Timer control Register */
   __IO u32 CTL;
   /* Timer IRQ clear register (W) */
   __O u32 IRQ_CLR_ACK;
   /* Timer Raw IRQ register (R) */
   __I u32 IRQ_RAW;
   /* Timer Masked IRQ register (R) */
   __I u32 IRQ_MASKED;
   /* Timer Reload register */
   __IO u32 RELOAD;
   /* Timer Pre-Divider register */
   /* Not present in SP804 */
   __IO u32 PD;
   /* Free-running counter */
   /* Not present in SP804 */
   __I u32 FRC;
} ARMTimer_Type;

/* Provide ARM timer, IRQs, mailboxes */
typedef struct {
   /* 0x40000000 Control Register */
   __IO u32 CTL;
   /* 0x40000004 unused */
   __IO u32 RESERVED0;
   /* 0x40000008 core timer prescaler */
   __IO u32 CORETIMER_PRESCALER;
   /* 0x4000000C GPU interrupts routing */
   __IO u32 GPU_IRQ_ROUTING;
   /* 0x40000010 Performance monitor interrupts routing write-set */
   __O u32 PMU_IRQ_ROUTING_SET;
   /* 0x40000014 Performance monitor interrupts routing write-clear */
   __O u32 PMU_IRQ_ROUTING_CLR;
   /* 0x40000018 unused */
   __IO u32 RESERVED1;
   /* 0x4000001C Core timer access LS 32 bits */
   __IO u32 CORETIMER_BITS_LO;
   /* 0x40000020 Core timer access MS 32 bits */
   __IO u32 CORETIMER_BITS_HI;
   /* 0x40000024 Local Interrupt 0 [1-7] routing */
   __IO u32 LOCAL_IRQ_ROUTING;
   /* 0x40000028 Local Interrupts 8-15 routing (unused) */
   __IO u32 RESERVED2;
   /* 0x4000002C AXI outstanding counters */
   __IO u32 AXI_OUTSTANDING_COUNTERS;
   /* 0x40000030 AXI outstanding IRQ */
   __IO u32 AXI_OUTSTANDING_IRQ;
   /* 0x40000034 Local Timer control & status */
   __IO u32 LTCSR;
   /* 0x40000038 Local Timer write flags (IRQ Clear and reload) (W) */
   __O u32 LTWFR;
   /* 0x4000003C unused */
   __IO u32 RESERVED3;
   /* 0x40000040 Core0 Timers Interrupt control */
   __IO u32 CORE0_TIMERS_IRQ_CTL;
   /* 0x40000044 Core1 Timers Interrupt control */
   __IO u32 CORE1_TIMERS_IRQ_CTL;
   /* 0x40000048 Core2 Timers Interrupt control */
   __IO u32 CORE2_TIMERS_IRQ_CTL;
   /* 0x4000004C Core3 Timers Interrupt control */
   __IO u32 CORE3_TIMERS_IRQ_CTL;
   /* 0x40000050 Core0 Mailboxes Interrupt control */
   __IO u32 CORE0_MAILBOXES_IRQ_CTL;
   /* 0x40000054 Core1 Mailboxes Interrupt control */
   __IO u32 CORE1_MAILBOXES_IRQ_CTL;
   /* 0x40000058 Core2 Mailboxes Interrupt control */
   __IO u32 CORE2_MAILBOXES_IRQ_CTL;
   /* 0x4000005C Core3 Mailboxes Interrupt control */
   __IO u32 CORE3_MAILBOXES_IRQ_CTL;
   /* 0x40000060 Core0 IRQ Source */
   __IO u32 CORE0_IRQ_SRC;
   /* 0x40000064 Core1 IRQ Source */
   __IO u32 CORE1_IRQ_SRC;
   /* 0x40000068 Core2 IRQ Source */
   __IO u32 CORE2_IRQ_SRC;
   /* 0x4000006C Core3 IRQ Source */
   __IO u32 CORE3_IRQ_SRC;
   /* 0x40000070 Core0 FIQ Source */
   __IO u32 CORE0_FIQ_SRC;
   /* 0x40000074 Core1 FIQ Source */
   __IO u32 CORE1_FIQ_SRC;
   /* 0x40000078 Core2 FIQ Source */
   __IO u32 CORE2_FIQ_SRC;
   /* 0x4000007C Core3 FIQ Source */
   __IO u32 CORE3_FIQ_SRC;
   /* 0x40000080 Core0 Mailbox 0 write-set (W) */
   __O u32 CORE0_MAILBOX_0_WRITE_SET;
   /* 0x40000084 Core0 Mailbox 1 write-set (W) */
   __O u32 CORE0_MAILBOX_1_WRITE_SET;
   /* 0x40000088 Core0 Mailbox 2 write-set (W) */
   __O u32 CORE0_MAILBOX_2_WRITE_SET;
   /* 0x4000008C Core0 Mailbox 3 write-set (W) */
   __O u32 CORE0_MAILBOX_3_WRITE_SET;
   /* 0x40000090 Core1 Mailbox 0 write-set (W) */
   __O u32 CORE1_MAILBOX_0_WRITE_SET;
   /* 0x40000094 Core1 Mailbox 1 write-set (W) */
   __O u32 CORE1_MAILBOX_1_WRITE_SET;
   /* 0x40000098 Core1 Mailbox 2 write-set (W) */
   __O u32 CORE1_MAILBOX_2_WRITE_SET;
   /* 0x4000009C Core1 Mailbox 3 write-set (W) */
   __O u32 CORE1_MAILBOX_3_WRITE_SET;
   /* 0x400000A0 Core2 Mailbox 0 write-set (W) */
   __O u32 CORE2_MAILBOX_0_WRITE_SET;
   /* 0x400000A4 Core2 Mailbox 1 write-set (W) */
   __O u32 CORE2_MAILBOX_1_WRITE_SET;
   /* 0x400000A8 Core2 Mailbox 2 write-set (W) */
   __O u32 CORE2_MAILBOX_2_WRITE_SET;
   /* 0x400000AC Core2 Mailbox 3 write-set (W) */
   __O u32 CORE2_MAILBOX_3_WRITE_SET;
   /* 0x400000B0 Core3 Mailbox 0 write-set (W) */
   __O u32 CORE3_MAILBOX_0_WRITE_SET;
   /* 0x400000B4 Core3 Mailbox 1 write-set (W) */
   __O u32 CORE3_MAILBOX_1_WRITE_SET;
   /* 0x400000B8 Core3 Mailbox 2 write-set (W) */
   __O u32 CORE3_MAILBOX_2_WRITE_SET;
   /* 0x400000BC Core3 Mailbox 3 write-set (W) */
   __O u32 CORE3_MAILBOX_3_WRITE_SET;
   /* 0x400000C0 Core0 Mailbox 0 read & write-high-to-clear */
   __IO u32 CORE0_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000C4 Core0 Mailbox 1 read & write-high-to-clear */
   __IO u32 CORE0_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000C8 Core0 Mailbox 2 read & write-high-to-clear */
   __IO u32 CORE0_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000CC Core0 Mailbox 3 read & write-high-to-clear */
   __IO u32 CORE0_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000D0 Core1 Mailbox 0 read & write-high-to-clear */
   __IO u32 CORE1_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000D4 Core1 Mailbox 1 read & write-high-to-clear */
   __IO u32 CORE1_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000D8 Core1 Mailbox 2 read & write-high-to-clear */
   __IO u32 CORE1_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000DC Core1 Mailbox 3 read & write-high-to-clear */
   __IO u32 CORE1_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000E0 Core2 Mailbox 0 read & write-high-to-clear */
   __IO u32 CORE2_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000E4 Core2 Mailbox 1 read & write-high-to-clear */
   __IO u32 CORE2_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000E8 Core2 Mailbox 2 read & write-high-to-clear */
   __IO u32 CORE2_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000EC Core2 Mailbox 3 read & write-high-to-clear */
   __IO u32 CORE2_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000F0 Core3 Mailbox 0 read & write-high-to-clear */
   __IO u32 CORE3_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000F4 Core3 Mailbox 1 read & write-high-to-clear */
   __IO u32 CORE3_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000F8 Core3 Mailbox 2 read & write-high-to-clear */
   __IO u32 CORE3_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR; 
   /* 0x400000FC Core3 Mailbox 3 read & write-high-to-clear */
   __IO u32 CORE3_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR; 
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

/*#define P1 ((GPIO_Type *) GPIO_BASE)*/
#define SYS_TMR ((SysTimer_Type *) SYS_TMR_BASE)
/*#define ARM_TMR ((ARMTimer_Type *) ARM_TMR_BASE) */
#define AUX ((AUX_Type *) AUX_BASE)
#define SPI ((SPI_Type *) SPI_BASE)
#define PWM ((PWM_Type* ) PWM_BASE)
#define IRQ_CTLS ((Interrupt_Controller_Type *) IRQ_BASE)
/*#define CM ((CM_Type *) CM_BASE)*/
/*#define PADS ((GPIO_Pads_Control_Type *) GPIO_PADS_CTL_BASE)*/
/*#define ARMCTL ((ARM_Control_Logic_Module_Type *) LOCAL_PERIPH_BASE) */

/* Peripheral register control bits */
/* R/W - read and write bit */
/* W1C - write 1 to clear the bit */
/* RO - read only bit */
/* W1SC - write 1, will self-clear */

/* DMA Bits */

/* CS - DMA Control and Status Register (RW) */
/* Activate the DMA bit (R/W) */
#define DMA_CS_ACTIVE_OFS (0)
#define DMA_CS_ACTIVE ((u32) 0x00000001)
/* DMA End Flag bit (W1C) */
#define DMA_CS_END_OFS (1)
#define DMA_CS_END ((u32) 0x00000002)
/* DMA Interrupt Status bit (W1C) */
#define DMA_CS_INT_OFS (2)
#define DMA_CS_INT ((u32) 0x00000004)
/* DMA DREQ (Data Request) state bit (RO) */
#define DMA_CS_DREQ_OFS (3)
#define DMA_CS_DREQ ((u32) 0x00000008)
/* DMA Paused State bit (RO) */
#define DMA_CS_PAUSED_OFS (4)
#define DMA_CS_PAUSED ((u32) 0x00000010)
/* DMA Paused by DREQ State bit (RO) */
#define DMA_CS_DREQ_STOPS_DMA_OFS (5)
#define DMA_CS_DREQ_STOPS_DMA ((u32) 0x00000020)
/* Waiting for Outstanding Writes bit (RO) */
#define DMA_CS_WAITING_FOR_OUTSTANDING_WRITES_OFS (6)
#define DMA_CS_WAITING_FOR_OUTSTANDING_WRITES ((u32) 0x00000040)
/* DMA Error (RO) bit */
#define DMA_CS_ERROR_OFS (8)
#define DMA_CS_ERROR ((u32) 0x00000100)
/* AXI Priority Level bits (RW) */
#define DMA_CS_PRIORITY_OFS (16)
#define DMA_CS_PRIORITY_MASK ((u32) 0x000F0000)
#define DMA_CS_PRIORITY0 ((u32) 0x00010000)
#define DMA_CS_PRIORITY1 ((u32) 0x00020000)
#define DMA_CS_PRIORITY2 ((u32) 0x00040000)
#define DMA_CS_PRIORITY3 ((u32) 0x00080000)
/* AXI Panic Priority Level bits (RW) */
#define DMA_CS_PANIC_PRIORITY_OFS (20)
#define DMA_CS_PANIC_PRIORITY_MASK ((u32) 0x00F00000)
#define DMA_CS_PANIC_PRIORITY0 ((u32) 0x00100000)
#define DMA_CS_PANIC_PRIORITY1 ((u32) 0x00200000)
#define DMA_CS_PANIC_PRIORITY2 ((u32) 0x00400000)
#define DMA_CS_PANIC_PRIORITY3 ((u32) 0x00800000)
/* Wait for outstanding writes bit (RW) */
#define DMA_CS_WAIT_FOR_OUTSTANDING_WRITES_OFS (28)
#define DMA_CS_WAIT_FOR_OUTSTANDING_WRITES ((u32) 0x10000000)
/* Disable debug pause signal bit (RW) */
#define DMA_CS_DISDEBUG_OFS (29)
#define DMA_CS_DISDEBUG ((u32) 0x20000000)
/* Abort DMA bit (W1SC) */
#define DMA_CS_ABORT_OFS (30)
#define DMA_CS_ABORT ((u32) 0x40000000)
/* DMA Channel Reset bit (W1SC) */
#define DMA_CS_RESET_OFS (31)
#define DMA_CS_RESET ((u32) 0x80000000)

/* TI - DMA Transfer Information register (RW) */
/* DMA0 - DMA6 */
/* Interrupt Enable bit (RW) */
#define DMA0_DMA6_TI_INTEN_OFS (0)
#define DMA0_DMA6_TI_INTEN ((u32) 0x00000001)
/* 2D Mode bit (RW) */
#define DMA0_DMA6_TI_TDMODE_OFS (1)
#define DMA0_DMA6_TI_TDMODE ((u32) 0x00000002)
/* Wait for a Write Response bit (RW) */
#define DMA0_DMA6_TI_WAIT_RESP_OFS (3)
#define DMA0_DMA6_TI_WAIT_RESP ((u32) 0x00000008)
/* Destination Address Increment bit (RW) */
#define DMA0_DMA6_TI_DEST_INC_OFS (4)
#define DMA0_DMA6_TI_DEST_INC ((u32) 0x00000010)
/* Destination Transfer Width bit (RW) */
#define DMA0_DMA6_TI_DEST_WIDTH_OFS (5)
#define DMA0_DMA6_TI_DEST_WIDTH ((u32) 0x00000020)
/* Control Destination Writes with DREQ bit (RW) */
#define DMA0_DMA6_TI_DEST_DREQ_OFS (6)
#define DMA0_DMA6_TI_DEST_DREQ ((u32) 0x00000040)
/* Ignore Writes bit (RW) */
#define DMA0_DMA6_TI_DEST_IGNORE_OFS (7)
#define DMA0_DMA6_TI_DEST_IGNORE ((u32) 0x00000080)
/* Source Address Increment bit (RW) */
#define DMA0_DMA6_TI_SRC_INC_OFS (8)
#define DMA0_DMA6_TI_SRC_INC ((u32) 0x00000100)
/* Source Transfer Width bit (RW) */
#define DMA0_DMA6_TI_SRC_WIDTH_OFS (9)
#define DMA0_DMA6_TI_SRC_WIDTH ((u32) 0x00000200)
/* Controle Source Reads with DREQ bit (RW) */
#define DMA0_DMA6_TI_SRC_DREQ_OFS (10)
#define DMA0_DMA6_TI_SRC_DREQ ((u32) 0x00000400)
/* Ignore Reads bit (RW) */
#define DMA0_DMA6_TI_SRC_IGNORE_OFS (11)
#define DMA0_DMA6_TI_SRC_IGNORE ((u32) 0x0000080)
/* Burst Transfer Length bits (RW) */
#define DMA0_DMA6_TI_BURST_LENGTH_OFS (12)
#define DMA0_DMA6_TI_BURST_LENGTH_MASK ((u32) 0x0000F000)
#define DMA0_DMA6_TI_BURST_LENGTH0 ((u32) 0x0000100)
#define DMA0_DMA6_TI_BURST_LENGTH1 ((u32) 0x0000200)
#define DMA0_DMA6_TI_BURST_LENGTH2 ((u32) 0x0000400)
#define DMA0_DMA6_TI_BURST_LENGTH3 ((u32) 0x0000800)
/* Peripheral Mapping bits (RW) */
#define DMA0_DMA6_TI_PERMAP_OFS (16)
#define DMA0_DMA6_TI_PERMAP_MASK ((u32) 0x001F0000)
#define DMA0_DMA6_TI_PERMAP0 ((u32) 0x00010000)
#define DMA0_DMA6_TI_PERMAP1 ((u32) 0x00020000)
#define DMA0_DMA6_TI_PERMAP2 ((u32) 0x00040000)
#define DMA0_DMA6_TI_PERMAP3 ((u32) 0x00080000)
#define DMA0_DMA6_TI_PERMAP4 ((u32) 0x00100000)
/* Add Wait Cycles bits (RW) */
#define DMA0_DMA6_TI_WAITS_OFS (21)
#define DMA0_DMA6_TI_WAITS_MASK ((u32) 0x03E00000)
#define DMA0_DMA6_TI_WAITS0 ((u32) 0x00200000)
#define DMA0_DMA6_TI_WAITS1 ((u32) 0x00400000)
#define DMA0_DMA6_TI_WAITS2 ((u32) 0x00800000)
#define DMA0_DMA6_TI_WAITS3 ((u32) 0x01000000)
#define DMA0_DMA6_TI_WAITS4 ((u32) 0x02000000)
/* No Wide Bursts bit (RW) */
#define DMA0_DMA6_TI_NO_WIDE_BURSTS_OFS (26)
#define DMA0_DMA6_TI_NO_WIDE_BURSTS ((u32) 0x04000000)
/* DMA7 - DMA 14 */
/* Interrupt Enable bit (RW) */
#define DMA7_DMA14_TI_INTEN_OFS (0)
#define DMA7_DMA14_TI_INTEN ((u32) 0x00000001)
/* Wait for a Write Response bit (RW) */
#define DMA7_DMA14_TI_WAIT_RESP_OFS (3)
#define DMA7_DMA14_TI_WAIT_RESP ((u32) 0x00000008)
/* Destination Address Increment bit (RW) */
#define DMA7_DMA14_TI_DEST_INC_OFS (4)
#define DMA7_DMA14_TI_DEST_INC ((u32) 0x00000010)
/* Destination Transfer Width bit (RW) */
#define DMA7_DMA14_TI_DEST_WIDTH_OFS (5)
#define DMA7_DMA14_TI_DEST_WIDTH ((u32) 0x00000020)
/* Control Destination Writes with DREQ bit (RW) */
#define DMA7_DMA14_TI_DEST_DREQ_OFS (6)
#define DMA7_DMA14_TI_DEST_DREQ ((u32) 0x00000040)
/* Ignore Writes bit (RW) - no description */
#define DMA7_DMA14_TI_DEST_IGNORE_OFS (7)
#define DMA7_DMA14_TI_DEST_IGNORE ((u32) 0x00000080)
/* Source Address Increment bit (RW) */
#define DMA7_DMA14_TI_SRC_INC_OFS (8)
#define DMA7_DMA14_TI_SRC_INC ((u32) 0x00000100)
/* Source Transfer Width bit (RW) */
#define DMA7_DMA14_TI_SRC_WIDTH_OFS (9)
#define DMA7_DMA14_TI_SRC_WIDTH ((u32) 0x00000200)
/* Controle Source Reads with DREQ bit (RW) */
#define DMA7_DMA14_TI_SRC_DREQ_OFS (10)
#define DMA7_DMA14_TI_SRC_DREQ ((u32) 0x00000400)
/* Ignore Reads bit (RW) - no description */
#define DMA7_DMA14_TI_SRC_IGNORE_OFS (11)
#define DMA7_DMA14_TI_SRC_IGNORE ((u32) 0x0000080)
/* Burst Transfer Length bits (RW) */
#define DMA7_DMA14_TI_BURST_LENGTH_OFS (12)
#define DMA7_DMA14_TI_BURST_LENGTH_MASK ((u32) 0x0000F000)
#define DMA7_DMA14_TI_BURST_LENGTH0 ((u32) 0x0000100)
#define DMA7_DMA14_TI_BURST_LENGTH1 ((u32) 0x0000200)
#define DMA7_DMA14_TI_BURST_LENGTH2 ((u32) 0x0000400)
#define DMA7_DMA14_TI_BURST_LENGTH3 ((u32) 0x0000800)
/* Peripheral Mapping bits (RW) */
#define DMA7_DMA14_TI_PERMAP_OFS (16)
#define DMA7_DMA14_TI_PERMAP_MASK ((u32) 0x001F0000)
#define DMA7_DMA14_TI_PERMAP0 ((u32) 0x00010000)
#define DMA7_DMA14_TI_PERMAP1 ((u32) 0x00020000)
#define DMA7_DMA14_TI_PERMAP2 ((u32) 0x00040000)
#define DMA7_DMA14_TI_PERMAP3 ((u32) 0x00080000)
#define DMA7_DMA14_TI_PERMAP4 ((u32) 0x00100000)
/* Add Wait Cycles bits (RW) */
#define DMA7_DMA14_TI_WAITS_OFS (21)
#define DMA7_DMA14_TI_WAITS_MASK ((u32) 0x03E00000)
#define DMA7_DMA14_TI_WAITS0 ((u32) 0x00200000)
#define DMA7_DMA14_TI_WAITS1 ((u32) 0x00400000)
#define DMA7_DMA14_TI_WAITS2 ((u32) 0x00800000)
#define DMA7_DMA14_TI_WAITS3 ((u32) 0x01000000)
#define DMA7_DMA14_TI_WAITS4 ((u32) 0x02000000)

/* TXFR_LEN - DMA Transfer Length Register (RW) */
/* DMA0 - DMA6 */
/* Transfer Length in bytes bits (RW) */
#define DMA0_DMA6_TXFR_LEN_XLENGTH_OFS (0)
#define DMA0_DMA6_TXFR_LEN_XLENGTH_MASK ((u32) 0x0000FFFF)
/* YLENGTH bits (RW) */
#define DMA0_DMA6_TXFR_LEN_YLENGTH_OFS (16)
#define DMA0_DMA6_TXFR_LEN_YLENGTH_MASK ((u32) 0x3FFF0000)
/* DMA7 - DMA14 */
/* Transfer Length in bytes bits (RW) */
#define DMA7_DMA14_TXFR_LEN_XLENGTH_OFS (0)
#define DMA7_DMA14_TXFR_LEN_XLENGTH_MASK ((u32) 0x0000FFFF)

/* STRIDE - DMA 2D Stride Register (RW) */
/* DMA0 - DMA6 */
/* Source Stride (2D Mode) bits (RW) */
#define DMA0_DMA6_STRIDE_S_STRIDE_OFS (0)
#define DMA0_DMA6_STRIDE_S_STRIDE_MASK ((u32) 0x0000FFFF)
/* Destination Stride (2D Mode) bits (RW) */
#define DMA0_DMA6_STRIDE_D_STRIDE_OFS (16)
#define DMA0_DMA6_STRIDE_D_STRIDE_MASK ((u32) 0xFFFF0000)


/* DEBUG - DMA Debug Register (RW) */
/* DMA0 - DMA6 */
/* Read Last Not Set Error bit (RW) */
#define DMA0_DMA6_DEBUG_READ_LAST_NOT_SET_ERROR_OFS (0)
#define DMA0_DMA6_DEBUG_READ_LAST_NOT_SET_ERROR ((u32) 0x00000001)
/* FIFO Error (RW) */
#define DMA0_DMA6_DEBUG_FIFO_ERROR_OFS (1)
#define DMA0_DMA6_DEBUG_FIFO_ERROR ((u32) 0x00000002)
/* Slave Read Response Error (RW) */
#define DMA0_DMA6_DEBUG_READ_ERROR_OFS (2)
#define DMA0_DMA6_DEBUG_READ_ERROR ((u32) 0x00000004)
/* DMA Outstanding Writes Counter bits (RO) */
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES_OFS (4)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES_MASK ((u32) 0x000000F0)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES0 ((u32) 0x00000010)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES1 ((u32) 0x00000020)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES2 ((u32) 0x00000040)
#define DMA0_DMA6_DEBUG_OUTSTANDING_WRITES3 ((u32) 0x00000080)
/* DMA ID bits (RO) */
#define DMA0_DMA6_DEBUG_DMA_ID_OFS (8)
#define DMA0_DMA6_DEBUG_DMA_ID_MASK ((u32) 0x0000FF00)
#define DMA0_DMA6_DEBUG_DMA_ID0 ((u32) 0x00000100)
#define DMA0_DMA6_DEBUG_DMA_ID1 ((u32) 0x00000200)
#define DMA0_DMA6_DEBUG_DMA_ID2 ((u32) 0x00000400)
#define DMA0_DMA6_DEBUG_DMA_ID3 ((u32) 0x00000800)
#define DMA0_DMA6_DEBUG_DMA_ID4 ((u32) 0x00001000)
#define DMA0_DMA6_DEBUG_DMA_ID5 ((u32) 0x00002000)
#define DMA0_DMA6_DEBUG_DMA_ID6 ((u32) 0x00004000)
#define DMA0_DMA6_DEBUG_DMA_ID7 ((u32) 0x00008000)
/* DMA State Machine State bits (RO) */
#define DMA0_DMA6_DEBUG_DMA_STATE_OFS (16)
#define DMA0_DMA6_DEBUG_DMA_STATE_MASK ((u32) 0x01FF0000)
#define DMA0_DMA6_DEBUG_DMA_STATE0 ((u32) 0x00010000)
#define DMA0_DMA6_DEBUG_DMA_STATE1 ((u32) 0x00020000)
#define DMA0_DMA6_DEBUG_DMA_STATE2 ((u32) 0x00040000)
#define DMA0_DMA6_DEBUG_DMA_STATE3 ((u32) 0x00080000)
#define DMA0_DMA6_DEBUG_DMA_STATE4 ((u32) 0x00100000)
#define DMA0_DMA6_DEBUG_DMA_STATE5 ((u32) 0x00200000)
#define DMA0_DMA6_DEBUG_DMA_STATE6 ((u32) 0x00400000)
#define DMA0_DMA6_DEBUG_DMA_STATE7 ((u32) 0x00800000)
#define DMA0_DMA6_DEBUG_DMA_STATE8 ((u32) 0x01000000)
/* DMA Version bits (RO) */
#define DMA0_DMA6_DEBUG_VERSION_OFS (25)
#define DMA0_DMA6_DEBUG_VERSION_MASK ((u32) 0x0E000000)
#define DMA0_DMA6_DEBUG_VERSION0 ((u32) 0x02000000)
#define DMA0_DMA6_DEBUG_VERSION1 ((u32) 0x04000000)
#define DMA0_DMA6_DEBUG_VERSION2 ((u32) 0x08000000)
/* DMA Lite bit (RO) */
#define DMA0_DMA6_DEBUG_LITE_OFS (28)
#define DMA0_DMA6_DEBUG_LITE ((u32) 0x10000000)
/* DMA7 - DMA14 */
/* Read Last Not Set Error bit (RW) */
#define DMA7_DMA14_DEBUG_READ_LAST_NOT_SET_ERROR_OFS (0)
#define DMA7_DMA14_DEBUG_READ_LAST_NOT_SET_ERROR ((u32) 0x00000001)
/* FIFO Error (RW) */
#define DMA7_DMA14_DEBUG_FIFO_ERROR_OFS (1)
#define DMA7_DMA14_DEBUG_FIFO_ERROR ((u32) 0x00000002)
/* Slave Read Response Error (RW) */
#define DMA7_DMA14_DEBUG_READ_ERROR_OFS (2)
#define DMA7_DMA14_DEBUG_READ_ERROR ((u32) 0x00000004)
/* DMA Outstanding Writes Counter bits (RO) */
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES_OFS (4)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES_MASK ((u32) 0x000000F0)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES0 ((u32) 0x00000010)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES1 ((u32) 0x00000020)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES2 ((u32) 0x00000040)
#define DMA7_DMA14_DEBUG_OUTSTANDING_WRITES3 ((u32) 0x00000080)
/* DMA ID bits (RO) */
#define DMA7_DMA14_DEBUG_DMA_ID_OFS (8)
#define DMA7_DMA14_DEBUG_DMA_ID_MASK ((u32) 0x0000FF00)
#define DMA7_DMA14_DEBUG_DMA_ID0 ((u32) 0x00000100)
#define DMA7_DMA14_DEBUG_DMA_ID1 ((u32) 0x00000200)
#define DMA7_DMA14_DEBUG_DMA_ID2 ((u32) 0x00000400)
#define DMA7_DMA14_DEBUG_DMA_ID3 ((u32) 0x00000800)
#define DMA7_DMA14_DEBUG_DMA_ID4 ((u32) 0x00001000)
#define DMA7_DMA14_DEBUG_DMA_ID5 ((u32) 0x00002000)
#define DMA7_DMA14_DEBUG_DMA_ID6 ((u32) 0x00004000)
#define DMA7_DMA14_DEBUG_DMA_ID7 ((u32) 0x00008000)
/* DMA State Machine State bits (RO) */
#define DMA7_DMA14_DEBUG_DMA_STATE_OFS (16)
#define DMA7_DMA14_DEBUG_DMA_STATE_MASK ((u32) 0x01FF0000)
#define DMA7_DMA14_DEBUG_DMA_STATE0 ((u32) 0x00010000)
#define DMA7_DMA14_DEBUG_DMA_STATE1 ((u32) 0x00020000)
#define DMA7_DMA14_DEBUG_DMA_STATE2 ((u32) 0x00040000)
#define DMA7_DMA14_DEBUG_DMA_STATE3 ((u32) 0x00080000)
#define DMA7_DMA14_DEBUG_DMA_STATE4 ((u32) 0x00100000)
#define DMA7_DMA14_DEBUG_DMA_STATE5 ((u32) 0x00200000)
#define DMA7_DMA14_DEBUG_DMA_STATE6 ((u32) 0x00400000)
#define DMA7_DMA14_DEBUG_DMA_STATE7 ((u32) 0x00800000)
#define DMA7_DMA14_DEBUG_DMA_STATE8 ((u32) 0x01000000)
/* DMA Version bits (RO) */
#define DMA7_DMA14_DEBUG_VERSION_OFS (25)
#define DMA7_DMA14_DEBUG_VERSION_MASK ((u32) 0x0E000000)
#define DMA7_DMA14_DEBUG_VERSION0 ((u32) 0x02000000)
#define DMA7_DMA14_DEBUG_VERSION1 ((u32) 0x04000000)
#define DMA7_DMA14_DEBUG_VERSION2 ((u32) 0x08000000)
/* DMA Lite bit (RO) */
#define DMA7_DMA14_DEBUG_LITE_OFS (28)
#define DMA7_DMA14_DEBUG_LITE ((u32) 0x10000000)

/* S_ADDR - DMA Source Address register (RW) */
/* DMA Source Address bits (RW) */
#define DMA_S_ADDR_OFS (0)
#define DMA_S_ADDR_MASK ((u32) 0xFFFFFFFF)

/* D_ADDR - DMA Destination Address register (RW) */
/* DMA Destination Address bits (RW) */
#define DMA_D_ADDR_OFS (0)
#define DMA_D_ADDR_MASK ((u32) 0xFFFFFFFF)

/* CONBLK - DMA Control Block Address register (RW) */
/* Control Block Address bits (RW) */
#define DMA_CONBLK_SCB_ADDR_OFS (0)
#define DMA_CONBLK_SCB_ADDR_MASK ((u32) 0xFFFFFFFF)

/* NEXTCONBK - Next Control Block Address Register (RW) */
/* Address of next CB for chained DMA operations (RW) */
#define DMA_NEXTCONBK_ADDR_OFS (0)
#define DMA_NEXTCONBK_ADDR_MASK ((u32) 0xFFFFFFFF)

/* DMA Global Register bits */
/* INT_STATUS - Interrupt Status of each DMA engine (RW) */
#define DMA_INT_STATUS_INT0_OFS (0)
#define DMA_INT_STATUS_INT0 ((u32) 0x00000001)
#define DMA_INT_STATUS_INT1_OFS (1)
#define DMA_INT_STATUS_INT1 ((u32) 0x00000002)
#define DMA_INT_STATUS_INT2_OFS (2)
#define DMA_INT_STATUS_INT2 ((u32) 0x00000004)
#define DMA_INT_STATUS_INT3_OFS (3)
#define DMA_INT_STATUS_INT3 ((u32) 0x00000008)
#define DMA_INT_STATUS_INT4_OFS (4)
#define DMA_INT_STATUS_INT4 ((u32) 0x00000010)
#define DMA_INT_STATUS_INT5_OFS (5)
#define DMA_INT_STATUS_INT5 ((u32) 0x00000020)
#define DMA_INT_STATUS_INT6_OFS (6)
#define DMA_INT_STATUS_INT6 ((u32) 0x00000040)
#define DMA_INT_STATUS_INT7_OFS (7)
#define DMA_INT_STATUS_INT7 ((u32) 0x00000080)
#define DMA_INT_STATUS_INT8_OFS (8)
#define DMA_INT_STATUS_INT8 ((u32) 0x00000100)
#define DMA_INT_STATUS_INT9_OFS (9)
#define DMA_INT_STATUS_INT9 ((u32) 0x00000200)
#define DMA_INT_STATUS_INT10_OFS (10)
#define DMA_INT_STATUS_INT10 ((u32) 0x00000400)
#define DMA_INT_STATUS_INT11_OFS (11)
#define DMA_INT_STATUS_INT11 ((u32) 0x00000800)
#define DMA_INT_STATUS_INT12_OFS (12)
#define DMA_INT_STATUS_INT12 ((u32) 0x00001000)
#define DMA_INT_STATUS_INT13_OFS (13)
#define DMA_INT_STATUS_INT13 ((u32) 0x00002000)
#define DMA_INT_STATUS_INT14_OFS (14)
#define DMA_INT_STATUS_INT14 ((u32) 0x00004000)

/* Global DMA Enable Register (RW) */
#define DMA_ENABLE_EN0_OFS (0)
#define DMA_ENABLE_EN0 ((u32) 0x00000001)
#define DMA_ENABLE_EN1_OFS (1)
#define DMA_ENABLE_EN1 ((u32) 0x00000002)
#define DMA_ENABLE_EN2_OFS (2)
#define DMA_ENABLE_EN2 ((u32) 0x00000004)
#define DMA_ENABLE_EN3_OFS (3)
#define DMA_ENABLE_EN3 ((u32) 0x00000008)
#define DMA_ENABLE_EN4_OFS (4)
#define DMA_ENABLE_EN4 ((u32) 0x00000010)
#define DMA_ENABLE_EN5_OFS (5)
#define DMA_ENABLE_EN5 ((u32) 0x00000020)
#define DMA_ENABLE_EN6_OFS (6)
#define DMA_ENABLE_EN6 ((u32) 0x00000040)
#define DMA_ENABLE_EN7_OFS (7)
#define DMA_ENABLE_EN7 ((u32) 0x00000080)
#define DMA_ENABLE_EN8_OFS (8)
#define DMA_ENABLE_EN8 ((u32) 0x00000100)
#define DMA_ENABLE_EN9_OFS (9)
#define DMA_ENABLE_EN9 ((u32) 0x00000200)
#define DMA_ENABLE_EN10_OFS (10)
#define DMA_ENABLE_EN10 ((u32) 0x00000400)
#define DMA_ENABLE_EN11_OFS (11)
#define DMA_ENABLE_EN11 ((u32) 0x00000800)
#define DMA_ENABLE_EN12_OFS (12)
#define DMA_ENABLE_EN12 ((u32) 0x00001000)
#define DMA_ENABLE_EN13_OFS (13)
#define DMA_ENABLE_EN13 ((u32) 0x00002000)
#define DMA_ENABLE_EN14_OFS (14)
#define DMA_ENABLE_EN14 ((u32) 0x00004000)

/* PWM Bits */

/* GPIO Bits */
/* GPIO Function Select 0 Register (RW) */
/* FSEL0 - Function Select 0 bits (RW) */
#define GPIO_GPFSEL0_FSEL0_OFS (0)
#define GPIO_GPFSEL0_FSEL0_MASK ((u32) 0x00000007)
#define GPIO_GPFSEL0_FSEL00 ((u32) 0x00000001)
#define GPIO_GPFSEL0_FSEL01 ((u32) 0x00000002)
#define GPIO_GPFSEL0_FSEL02 ((u32) 0x00000004)
#define GPIO_GPFSEL0_FSEL0_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL0_1 ((u32) 0x00000001)
#define GPIO_GPFSEL0_FSEL0_2 ((u32) 0x00000002)
#define GPIO_GPFSEL0_FSEL0_3 ((u32) 0x00000003)
#define GPIO_GPFSEL0_FSEL0_4 ((u32) 0x00000004)
#define GPIO_GPFSEL0_FSEL0_5 ((u32) 0x00000005)
#define GPIO_GPFSEL0_FSEL0_6 ((u32) 0x00000006)
#define GPIO_GPFSEL0_FSEL0_7 ((u32) 0x00000007)
#define GPIO_GPFSEL0_FSEL0__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL0__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL0_FSEL0__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL0_FSEL0__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL0_FSEL0__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL0_FSEL0__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL0_FSEL0__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL0_FSEL0__ALT3 ((u32) 0x00000007)
/* FSEL1 - Function Select 1 bits (RW) */
#define GPIO_GPFSEL0_FSEL1_OFS (3)
#define GPIO_GPFSEL0_FSEL1_MASK ((u32) 0x00000038)
#define GPIO_GPFSEL0_FSEL10 ((u32) 0x00000040)
#define GPIO_GPFSEL0_FSEL11 ((u32) 0x00000080)
#define GPIO_GPFSEL0_FSEL12 ((u32) 0x00000100)
#define GPIO_GPFSEL0_FSEL1_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL1_1 ((u32) 0x00000040)
#define GPIO_GPFSEL0_FSEL1_2 ((u32) 0x00000080)
#define GPIO_GPFSEL0_FSEL1_3 ((u32) 0x000000C0)
#define GPIO_GPFSEL0_FSEL1_4 ((u32) 0x00000100)
#define GPIO_GPFSEL0_FSEL1_5 ((u32) 0x00000140)
#define GPIO_GPFSEL0_FSEL1_6 ((u32) 0x00000180)
#define GPIO_GPFSEL0_FSEL1_7 ((u32) 0x000001C0)
#define GPIO_GPFSEL0_FSEL1__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL1__OUTPUT ((u32) 0x00000040)
#define GPIO_GPFSEL0_FSEL1__ALT5 ((u32) 0x00000080)
#define GPIO_GPFSEL0_FSEL1__ALT4 ((u32) 0x000000C0)
#define GPIO_GPFSEL0_FSEL1__ALT0 ((u32) 0x00000100)
#define GPIO_GPFSEL0_FSEL1__ALT1 ((u32) 0x00000140)
#define GPIO_GPFSEL0_FSEL1__ALT2 ((u32) 0x00000180)
#define GPIO_GPFSEL0_FSEL1__ALT3 ((u32) 0x000001C0)
/* FSEL2 - Function Select 2 bits (RW) */
#define GPIO_GPFSEL0_FSEL2_OFS (6)
#define GPIO_GPFSEL0_FSEL2_MASK ((u32) 0x000001C0)
#define GPIO_GPFSEL0_FSEL20 ((u32) 0x00000001)
#define GPIO_GPFSEL0_FSEL21 ((u32) 0x00000002)
#define GPIO_GPFSEL0_FSEL22 ((u32) 0x00000004)
#define GPIO_GPFSEL0_FSEL2_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL2_1 ((u32) 0x00000001)
#define GPIO_GPFSEL0_FSEL2_2 ((u32) 0x00000002)
#define GPIO_GPFSEL0_FSEL2_3 ((u32) 0x00000003)
#define GPIO_GPFSEL0_FSEL2_4 ((u32) 0x00000004)
#define GPIO_GPFSEL0_FSEL2_5 ((u32) 0x00000005)
#define GPIO_GPFSEL0_FSEL2_6 ((u32) 0x00000006)
#define GPIO_GPFSEL0_FSEL2_7 ((u32) 0x00000007)
#define GPIO_GPFSEL0_FSEL2__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL2__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL0_FSEL2__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL0_FSEL2__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL0_FSEL2__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL0_FSEL2__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL0_FSEL2__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL0_FSEL2__ALT3 ((u32) 0x00000007)
/* FSEL3 - Function Select 3 bits (RW) */
#define GPIO_GPFSEL0_FSEL3_OFS (9)
#define GPIO_GPFSEL0_FSEL3_MASK ((u32) 0x00000E00)
#define GPIO_GPFSEL0_FSEL30 ((u32) 0x00000200)
#define GPIO_GPFSEL0_FSEL31 ((u32) 0x00000400)
#define GPIO_GPFSEL0_FSEL32 ((u32) 0x00000800)
#define GPIO_GPFSEL0_FSEL3_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL3_1 ((u32) 0x00000200)
#define GPIO_GPFSEL0_FSEL3_2 ((u32) 0x00000400)
#define GPIO_GPFSEL0_FSEL3_3 ((u32) 0x00000600)
#define GPIO_GPFSEL0_FSEL3_4 ((u32) 0x00000800)
#define GPIO_GPFSEL0_FSEL3_5 ((u32) 0x00000A00)
#define GPIO_GPFSEL0_FSEL3_6 ((u32) 0x00000C00)
#define GPIO_GPFSEL0_FSEL3_7 ((u32) 0x00000E00)
#define GPIO_GPFSEL0_FSEL3__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL3__OUTPUT ((u32) 0x0000200)
#define GPIO_GPFSEL0_FSEL3__ALT5 ((u32) 0x00000400)
#define GPIO_GPFSEL0_FSEL3__ALT4 ((u32) 0x00000600)
#define GPIO_GPFSEL0_FSEL3__ALT0 ((u32) 0x00000800)
#define GPIO_GPFSEL0_FSEL3__ALT1 ((u32) 0x00000A00)
#define GPIO_GPFSEL0_FSEL3__ALT2 ((u32) 0x00000C00)
#define GPIO_GPFSEL0_FSEL3__ALT3 ((u32) 0x00000E00)
/* FSEL4 - Function Select 4 bits (RW) */
#define GPIO_GPFSEL0_FSEL4_OFS (12)
#define GPIO_GPFSEL0_FSEL4_MASK ((u32) 0x00007000)
#define GPIO_GPFSEL0_FSEL40 ((u32) 0x00001000)
#define GPIO_GPFSEL0_FSEL41 ((u32) 0x00002000)
#define GPIO_GPFSEL0_FSEL42 ((u32) 0x00004000)
#define GPIO_GPFSEL0_FSEL4_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL4_1 ((u32) 0x00001000)
#define GPIO_GPFSEL0_FSEL4_2 ((u32) 0x00002000)
#define GPIO_GPFSEL0_FSEL4_3 ((u32) 0x00003000)
#define GPIO_GPFSEL0_FSEL4_4 ((u32) 0x00004000)
#define GPIO_GPFSEL0_FSEL4_5 ((u32) 0x00005000)
#define GPIO_GPFSEL0_FSEL4_6 ((u32) 0x00006000)
#define GPIO_GPFSEL0_FSEL4_7 ((u32) 0x00007000)
#define GPIO_GPFSEL0_FSEL4__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL4__OUTPUT ((u32) 0x00001000)
#define GPIO_GPFSEL0_FSEL4__ALT5 ((u32) 0x00002000)
#define GPIO_GPFSEL0_FSEL4__ALT4 ((u32) 0x00003000)
#define GPIO_GPFSEL0_FSEL4__ALT0 ((u32) 0x00004000)
#define GPIO_GPFSEL0_FSEL4__ALT1 ((u32) 0x00005000)
#define GPIO_GPFSEL0_FSEL4__ALT2 ((u32) 0x00006000)
#define GPIO_GPFSEL0_FSEL4__ALT3 ((u32) 0x00007000)
/* FSEL5 - Function Select 5 bits (RW) */
#define GPIO_GPFSEL0_FSEL5_OFS (15)
#define GPIO_GPFSEL0_FSEL5_MASK ((u32) 0x00038000)
#define GPIO_GPFSEL0_FSEL50 ((u32) 0x00008000)
#define GPIO_GPFSEL0_FSEL51 ((u32) 0x00010000)
#define GPIO_GPFSEL0_FSEL52 ((u32) 0x00020000)
#define GPIO_GPFSEL0_FSEL5_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL5_1 ((u32) 0x00080000)
#define GPIO_GPFSEL0_FSEL5_2 ((u32) 0x00100000)
#define GPIO_GPFSEL0_FSEL5_3 ((u32) 0x00180000)
#define GPIO_GPFSEL0_FSEL5_4 ((u32) 0x00200000)
#define GPIO_GPFSEL0_FSEL5_5 ((u32) 0x00280000)
#define GPIO_GPFSEL0_FSEL5_6 ((u32) 0x00300000)
#define GPIO_GPFSEL0_FSEL5_7 ((u32) 0x00380000)
#define GPIO_GPFSEL0_FSEL5__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL5__OUTPUT ((u32) 0x00080000)
#define GPIO_GPFSEL0_FSEL5__ALT5 ((u32) 0x00100000)
#define GPIO_GPFSEL0_FSEL5__ALT4 ((u32) 0x00180000)
#define GPIO_GPFSEL0_FSEL5__ALT0 ((u32) 0x00200000)
#define GPIO_GPFSEL0_FSEL5__ALT1 ((u32) 0x00280000)
#define GPIO_GPFSEL0_FSEL5__ALT2 ((u32) 0x00300000)
#define GPIO_GPFSEL0_FSEL5__ALT3 ((u32) 0x00380000)
/* FSEL6 - Function Select 6 bits (RW) */
#define GPIO_GPFSEL0_FSEL6_OFS (18)
#define GPIO_GPFSEL0_FSEL6_MASK ((u32) 0x001C0000)
#define GPIO_GPFSEL0_FSEL60 ((u32) 0x00040000)
#define GPIO_GPFSEL0_FSEL61 ((u32) 0x00080000)
#define GPIO_GPFSEL0_FSEL62 ((u32) 0x00100000)
#define GPIO_GPFSEL0_FSEL6_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL6_1 ((u32) 0x00040000)
#define GPIO_GPFSEL0_FSEL6_2 ((u32) 0x00080000)
#define GPIO_GPFSEL0_FSEL6_3 ((u32) 0x000C0000)
#define GPIO_GPFSEL0_FSEL6_4 ((u32) 0x00100000)
#define GPIO_GPFSEL0_FSEL6_5 ((u32) 0x00140000)
#define GPIO_GPFSEL0_FSEL6_6 ((u32) 0x00180000)
#define GPIO_GPFSEL0_FSEL6_7 ((u32) 0x001C0000)
#define GPIO_GPFSEL0_FSEL6__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL6__OUTPUT ((u32) 0x00040000)
#define GPIO_GPFSEL0_FSEL6__ALT5 ((u32) 0x00080000)
#define GPIO_GPFSEL0_FSEL6__ALT4 ((u32) 0x000C0000)
#define GPIO_GPFSEL0_FSEL6__ALT0 ((u32) 0x00100000)
#define GPIO_GPFSEL0_FSEL6__ALT1 ((u32) 0x00140000)
#define GPIO_GPFSEL0_FSEL6__ALT2 ((u32) 0x00180000)
#define GPIO_GPFSEL0_FSEL6__ALT3 ((u32) 0x001C0000)
/* FSEL7 - Function Select 7 bits (RW) */
#define GPIO_GPFSEL0_FSEL7_OFS (21)
#define GPIO_GPFSEL0_FSEL7_MASK ((u32) 0x00E00000)
#define GPIO_GPFSEL0_FSEL70 ((u32) 0x00200000)
#define GPIO_GPFSEL0_FSEL71 ((u32) 0x00400000)
#define GPIO_GPFSEL0_FSEL72 ((u32) 0x00800000)
#define GPIO_GPFSEL0_FSEL7_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL7_1 ((u32) 0x00200000)
#define GPIO_GPFSEL0_FSEL7_2 ((u32) 0x00400000)
#define GPIO_GPFSEL0_FSEL7_3 ((u32) 0x00600000)
#define GPIO_GPFSEL0_FSEL7_4 ((u32) 0x00800000)
#define GPIO_GPFSEL0_FSEL7_5 ((u32) 0x00A00000)
#define GPIO_GPFSEL0_FSEL7_6 ((u32) 0x00C00000)
#define GPIO_GPFSEL0_FSEL7_7 ((u32) 0x00E00000)
#define GPIO_GPFSEL0_FSEL7__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL7__OUTPUT ((u32) 0x00200000)
#define GPIO_GPFSEL0_FSEL7__ALT5 ((u32) 0x00400000)
#define GPIO_GPFSEL0_FSEL7__ALT4 ((u32) 0x00600000)
#define GPIO_GPFSEL0_FSEL7__ALT0 ((u32) 0x00800000)
#define GPIO_GPFSEL0_FSEL7__ALT1 ((u32) 0x00A00000)
#define GPIO_GPFSEL0_FSEL7__ALT2 ((u32) 0x00C00000)
#define GPIO_GPFSEL0_FSEL7__ALT3 ((u32) 0x00E00000)
/* FSEL8 - Function Select 8 bits (RW) */
#define GPIO_GPFSEL0_FSEL8_OFS (24)
#define GPIO_GPFSEL0_FSEL8_MASK ((u32) 0x07000000)
#define GPIO_GPFSEL0_FSEL80 ((u32) 0x01000000)
#define GPIO_GPFSEL0_FSEL81 ((u32) 0x02000000)
#define GPIO_GPFSEL0_FSEL82 ((u32) 0x04000000)
#define GPIO_GPFSEL0_FSEL8_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL8_1 ((u32) 0x01000000)
#define GPIO_GPFSEL0_FSEL8_2 ((u32) 0x02000000)
#define GPIO_GPFSEL0_FSEL8_3 ((u32) 0x03000000)
#define GPIO_GPFSEL0_FSEL8_4 ((u32) 0x04000000)
#define GPIO_GPFSEL0_FSEL8_5 ((u32) 0x05000000)
#define GPIO_GPFSEL0_FSEL8_6 ((u32) 0x06000000)
#define GPIO_GPFSEL0_FSEL8_7 ((u32) 0x07000000)
#define GPIO_GPFSEL0_FSEL8__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL8__OUTPUT ((u32) 0x01000000)
#define GPIO_GPFSEL0_FSEL8__ALT5 ((u32) 0x02000000)
#define GPIO_GPFSEL0_FSEL8__ALT4 ((u32) 0x03000000)
#define GPIO_GPFSEL0_FSEL8__ALT0 ((u32) 0x04000000)
#define GPIO_GPFSEL0_FSEL8__ALT1 ((u32) 0x05000000)
#define GPIO_GPFSEL0_FSEL8__ALT2 ((u32) 0x06000000)
#define GPIO_GPFSEL0_FSEL8__ALT3 ((u32) 0x07000000)
/* FSEL9 - Function Select 9 bits (RW) */
#define GPIO_GPFSEL0_FSEL9_OFS (27)
#define GPIO_GPFSEL0_FSEL9_MASK ((u32) 0x38000000)
#define GPIO_GPFSEL0_FSEL90 ((u32) 0x08000000)
#define GPIO_GPFSEL0_FSEL91 ((u32) 0x10000000)
#define GPIO_GPFSEL0_FSEL92 ((u32) 0x20000000)
#define GPIO_GPFSEL0_FSEL9_0 ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL9_1 ((u32) 0x08000000)
#define GPIO_GPFSEL0_FSEL9_2 ((u32) 0x10000000)
#define GPIO_GPFSEL0_FSEL9_3 ((u32) 0x18000000)
#define GPIO_GPFSEL0_FSEL9_4 ((u32) 0x20000000)
#define GPIO_GPFSEL0_FSEL9_5 ((u32) 0x28000000)
#define GPIO_GPFSEL0_FSEL9_6 ((u32) 0x30000000)
#define GPIO_GPFSEL0_FSEL9_7 ((u32) 0x38000000)
#define GPIO_GPFSEL0_FSEL9__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL0_FSEL9__OUTPUT ((u32) 0x08000000)
#define GPIO_GPFSEL0_FSEL9__ALT5 ((u32) 0x10000000)
#define GPIO_GPFSEL0_FSEL9__ALT4 ((u32) 0x18000000)
#define GPIO_GPFSEL0_FSEL9__ALT0 ((u32) 0x20000000)
#define GPIO_GPFSEL0_FSEL9__ALT1 ((u32) 0x28000000)
#define GPIO_GPFSEL0_FSEL9__ALT2 ((u32) 0x30000000)
#define GPIO_GPFSEL0_FSEL9__ALT3 ((u32) 0x30000000)

/* GPIO Function Select 1 Register (RW) */
/* FSEL10 - Function Select 10 bits (RW) */
#define GPIO_GPFSEL1_FSEL10_OFS (0)
#define GPIO_GPFSEL1_FSEL10_MASK ((u32) 0x00000007)
#define GPIO_GPFSEL1_FSEL100 ((u32) 0x00000001)
#define GPIO_GPFSEL1_FSEL101 ((u32) 0x00000002)
#define GPIO_GPFSEL1_FSEL102 ((u32) 0x00000004)
#define GPIO_GPFSEL1_FSEL10_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL10_1 ((u32) 0x00000001)
#define GPIO_GPFSEL1_FSEL10_2 ((u32) 0x00000002)
#define GPIO_GPFSEL1_FSEL10_3 ((u32) 0x00000003)
#define GPIO_GPFSEL1_FSEL10_4 ((u32) 0x00000004)
#define GPIO_GPFSEL1_FSEL10_5 ((u32) 0x00000005)
#define GPIO_GPFSEL1_FSEL10_6 ((u32) 0x00000006)
#define GPIO_GPFSEL1_FSEL10_7 ((u32) 0x00000007)
#define GPIO_GPFSEL1_FSEL10__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL10__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL1_FSEL10__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL1_FSEL10__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL1_FSEL10__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL1_FSEL10__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL1_FSEL10__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL1_FSEL10__ALT3 ((u32) 0x00000007)
/* FSEL11 - Function Select 11 bits (RW) */
#define GPIO_GPFSEL1_FSEL11_OFS (3)
#define GPIO_GPFSEL1_FSEL11_MASK ((u32) 0x00000038)
#define GPIO_GPFSEL1_FSEL110 ((u32) 0x00000040)
#define GPIO_GPFSEL1_FSEL111 ((u32) 0x00000080)
#define GPIO_GPFSEL1_FSEL112 ((u32) 0x00000100)
#define GPIO_GPFSEL1_FSEL11_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL11_1 ((u32) 0x00000040)
#define GPIO_GPFSEL1_FSEL11_2 ((u32) 0x00000080)
#define GPIO_GPFSEL1_FSEL11_3 ((u32) 0x000000C0)
#define GPIO_GPFSEL1_FSEL11_4 ((u32) 0x00000100)
#define GPIO_GPFSEL1_FSEL11_5 ((u32) 0x00000140)
#define GPIO_GPFSEL1_FSEL11_6 ((u32) 0x00000180)
#define GPIO_GPFSEL1_FSEL11_7 ((u32) 0x000001C0)
#define GPIO_GPFSEL1_FSEL11__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL11__OUTPUT ((u32) 0x00000040)
#define GPIO_GPFSEL1_FSEL11__ALT5 ((u32) 0x00000080)
#define GPIO_GPFSEL1_FSEL11__ALT4 ((u32) 0x000000C0)
#define GPIO_GPFSEL1_FSEL11__ALT0 ((u32) 0x00000100)
#define GPIO_GPFSEL1_FSEL11__ALT1 ((u32) 0x00000140)
#define GPIO_GPFSEL1_FSEL11__ALT2 ((u32) 0x00000180)
#define GPIO_GPFSEL1_FSEL11__ALT3 ((u32) 0x000001C0)
/* FSEL12 - Function Select 12 bits (RW) */
#define GPIO_GPFSEL1_FSEL12_OFS (6)
#define GPIO_GPFSEL1_FSEL12_MASK ((u32) 0x000001C0)
#define GPIO_GPFSEL1_FSEL120 ((u32) 0x00000001)
#define GPIO_GPFSEL1_FSEL121 ((u32) 0x00000002)
#define GPIO_GPFSEL1_FSEL122 ((u32) 0x00000004)
#define GPIO_GPFSEL1_FSEL12_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL12_1 ((u32) 0x00000001)
#define GPIO_GPFSEL1_FSEL12_2 ((u32) 0x00000002)
#define GPIO_GPFSEL1_FSEL12_3 ((u32) 0x00000003)
#define GPIO_GPFSEL1_FSEL12_4 ((u32) 0x00000004)
#define GPIO_GPFSEL1_FSEL12_5 ((u32) 0x00000005)
#define GPIO_GPFSEL1_FSEL12_6 ((u32) 0x00000006)
#define GPIO_GPFSEL1_FSEL12_7 ((u32) 0x00000007)
#define GPIO_GPFSEL1_FSEL12__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL12__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL1_FSEL12__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL1_FSEL12__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL1_FSEL12__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL1_FSEL12__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL1_FSEL12__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL1_FSEL12__ALT3 ((u32) 0x00000007)
/* FSEL13 - Function Select 13 bits (RW) */
#define GPIO_GPFSEL1_FSEL13_OFS (9)
#define GPIO_GPFSEL1_FSEL13_MASK ((u32) 0x00000E00)
#define GPIO_GPFSEL1_FSEL130 ((u32) 0x00000200)
#define GPIO_GPFSEL1_FSEL131 ((u32) 0x00000400)
#define GPIO_GPFSEL1_FSEL132 ((u32) 0x00000800)
#define GPIO_GPFSEL1_FSEL13_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL13_1 ((u32) 0x00000200)
#define GPIO_GPFSEL1_FSEL13_2 ((u32) 0x00000400)
#define GPIO_GPFSEL1_FSEL13_3 ((u32) 0x00000600)
#define GPIO_GPFSEL1_FSEL13_4 ((u32) 0x00000800)
#define GPIO_GPFSEL1_FSEL13_5 ((u32) 0x00000A00)
#define GPIO_GPFSEL1_FSEL13_6 ((u32) 0x00000C00)
#define GPIO_GPFSEL1_FSEL13_7 ((u32) 0x00000E00)
#define GPIO_GPFSEL1_FSEL13__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL13__OUTPUT ((u32) 0x0000200)
#define GPIO_GPFSEL1_FSEL13__ALT5 ((u32) 0x00000400)
#define GPIO_GPFSEL1_FSEL13__ALT4 ((u32) 0x00000600)
#define GPIO_GPFSEL1_FSEL13__ALT0 ((u32) 0x00000800)
#define GPIO_GPFSEL1_FSEL13__ALT1 ((u32) 0x00000A00)
#define GPIO_GPFSEL1_FSEL13__ALT2 ((u32) 0x00000C00)
#define GPIO_GPFSEL1_FSEL13__ALT3 ((u32) 0x00000E00)
/* FSEL14 - Function Select 14 bits (RW) */
#define GPIO_GPFSEL1_FSEL14_OFS (12)
#define GPIO_GPFSEL1_FSEL14_MASK ((u32) 0x00007000)
#define GPIO_GPFSEL1_FSEL140 ((u32) 0x00001000)
#define GPIO_GPFSEL1_FSEL141 ((u32) 0x00002000)
#define GPIO_GPFSEL1_FSEL142 ((u32) 0x00004000)
#define GPIO_GPFSEL1_FSEL14_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL14_1 ((u32) 0x00001000)
#define GPIO_GPFSEL1_FSEL14_2 ((u32) 0x00002000)
#define GPIO_GPFSEL1_FSEL14_3 ((u32) 0x00003000)
#define GPIO_GPFSEL1_FSEL14_4 ((u32) 0x00004000)
#define GPIO_GPFSEL1_FSEL14_5 ((u32) 0x00005000)
#define GPIO_GPFSEL1_FSEL14_6 ((u32) 0x00006000)
#define GPIO_GPFSEL1_FSEL14_7 ((u32) 0x00007000)
#define GPIO_GPFSEL1_FSEL14__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL14__OUTPUT ((u32) 0x00001000)
#define GPIO_GPFSEL1_FSEL14__ALT5 ((u32) 0x00002000)
#define GPIO_GPFSEL1_FSEL14__ALT4 ((u32) 0x00003000)
#define GPIO_GPFSEL1_FSEL14__ALT0 ((u32) 0x00004000)
#define GPIO_GPFSEL1_FSEL14__ALT1 ((u32) 0x00005000)
#define GPIO_GPFSEL1_FSEL14__ALT2 ((u32) 0x00006000)
#define GPIO_GPFSEL1_FSEL14__ALT3 ((u32) 0x00007000)
/* FSEL15 - Function Select 15 bits (RW) */
#define GPIO_GPFSEL1_FSEL15_OFS (15)
#define GPIO_GPFSEL1_FSEL15_MASK ((u32) 0x00038000)
#define GPIO_GPFSEL1_FSEL150 ((u32) 0x00008000)
#define GPIO_GPFSEL1_FSEL151 ((u32) 0x00010000)
#define GPIO_GPFSEL1_FSEL152 ((u32) 0x00020000)
#define GPIO_GPFSEL1_FSEL15_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL15_1 ((u32) 0x00080000)
#define GPIO_GPFSEL1_FSEL15_2 ((u32) 0x00100000)
#define GPIO_GPFSEL1_FSEL15_3 ((u32) 0x00180000)
#define GPIO_GPFSEL1_FSEL15_4 ((u32) 0x00200000)
#define GPIO_GPFSEL1_FSEL15_5 ((u32) 0x00280000)
#define GPIO_GPFSEL1_FSEL15_6 ((u32) 0x00300000)
#define GPIO_GPFSEL1_FSEL15_7 ((u32) 0x00380000)
#define GPIO_GPFSEL1_FSEL15__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL15__OUTPUT ((u32) 0x00080000)
#define GPIO_GPFSEL1_FSEL15__ALT5 ((u32) 0x00100000)
#define GPIO_GPFSEL1_FSEL15__ALT4 ((u32) 0x00180000)
#define GPIO_GPFSEL1_FSEL15__ALT0 ((u32) 0x00200000)
#define GPIO_GPFSEL1_FSEL15__ALT1 ((u32) 0x00280000)
#define GPIO_GPFSEL1_FSEL15__ALT2 ((u32) 0x00300000)
#define GPIO_GPFSEL1_FSEL15__ALT3 ((u32) 0x00380000)
/* FSEL16 - Function Select 16 bits (RW) */
#define GPIO_GPFSEL1_FSEL16_OFS (18)
#define GPIO_GPFSEL1_FSEL16_MASK ((u32) 0x001C0000)
#define GPIO_GPFSEL1_FSEL160 ((u32) 0x00040000)
#define GPIO_GPFSEL1_FSEL161 ((u32) 0x00080000)
#define GPIO_GPFSEL1_FSEL162 ((u32) 0x00100000)
#define GPIO_GPFSEL1_FSEL16_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL16_1 ((u32) 0x00040000)
#define GPIO_GPFSEL1_FSEL16_2 ((u32) 0x00080000)
#define GPIO_GPFSEL1_FSEL16_3 ((u32) 0x000C0000)
#define GPIO_GPFSEL1_FSEL16_4 ((u32) 0x00100000)
#define GPIO_GPFSEL1_FSEL16_5 ((u32) 0x00140000)
#define GPIO_GPFSEL1_FSEL16_6 ((u32) 0x00180000)
#define GPIO_GPFSEL1_FSEL16_7 ((u32) 0x001C0000)
#define GPIO_GPFSEL1_FSEL16__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL16__OUTPUT ((u32) 0x00040000)
#define GPIO_GPFSEL1_FSEL16__ALT5 ((u32) 0x00080000)
#define GPIO_GPFSEL1_FSEL16__ALT4 ((u32) 0x000C0000)
#define GPIO_GPFSEL1_FSEL16__ALT0 ((u32) 0x00100000)
#define GPIO_GPFSEL1_FSEL16__ALT1 ((u32) 0x00140000)
#define GPIO_GPFSEL1_FSEL16__ALT2 ((u32) 0x00180000)
#define GPIO_GPFSEL1_FSEL16__ALT3 ((u32) 0x001C0000)
/* FSEL17 - Function Select 17 bits (RW) */
#define GPIO_GPFSEL1_FSEL17_OFS (21)
#define GPIO_GPFSEL1_FSEL17_MASK ((u32) 0x00E00000)
#define GPIO_GPFSEL1_FSEL170 ((u32) 0x00200000)
#define GPIO_GPFSEL1_FSEL171 ((u32) 0x00400000)
#define GPIO_GPFSEL1_FSEL172 ((u32) 0x00800000)
#define GPIO_GPFSEL1_FSEL17_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL17_1 ((u32) 0x00200000)
#define GPIO_GPFSEL1_FSEL17_2 ((u32) 0x00400000)
#define GPIO_GPFSEL1_FSEL17_3 ((u32) 0x00600000)
#define GPIO_GPFSEL1_FSEL17_4 ((u32) 0x00800000)
#define GPIO_GPFSEL1_FSEL17_5 ((u32) 0x00A00000)
#define GPIO_GPFSEL1_FSEL17_6 ((u32) 0x00C00000)
#define GPIO_GPFSEL1_FSEL17_7 ((u32) 0x00E00000)
#define GPIO_GPFSEL1_FSEL17__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL17__OUTPUT ((u32) 0x00200000)
#define GPIO_GPFSEL1_FSEL17__ALT5 ((u32) 0x00400000)
#define GPIO_GPFSEL1_FSEL17__ALT4 ((u32) 0x00600000)
#define GPIO_GPFSEL1_FSEL17__ALT0 ((u32) 0x00800000)
#define GPIO_GPFSEL1_FSEL17__ALT1 ((u32) 0x00A00000)
#define GPIO_GPFSEL1_FSEL17__ALT2 ((u32) 0x00C00000)
#define GPIO_GPFSEL1_FSEL17__ALT3 ((u32) 0x00E00000)
/* FSEL18 - Function Select 18 bits (RW) */
#define GPIO_GPFSEL1_FSEL18_OFS (24)
#define GPIO_GPFSEL1_FSEL18_MASK ((u32) 0x07000000)
#define GPIO_GPFSEL1_FSEL180 ((u32) 0x01000000)
#define GPIO_GPFSEL1_FSEL181 ((u32) 0x02000000)
#define GPIO_GPFSEL1_FSEL182 ((u32) 0x04000000)
#define GPIO_GPFSEL1_FSEL18_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL18_1 ((u32) 0x01000000)
#define GPIO_GPFSEL1_FSEL18_2 ((u32) 0x02000000)
#define GPIO_GPFSEL1_FSEL18_3 ((u32) 0x03000000)
#define GPIO_GPFSEL1_FSEL18_4 ((u32) 0x04000000)
#define GPIO_GPFSEL1_FSEL18_5 ((u32) 0x05000000)
#define GPIO_GPFSEL1_FSEL18_6 ((u32) 0x06000000)
#define GPIO_GPFSEL1_FSEL18_7 ((u32) 0x07000000)
#define GPIO_GPFSEL1_FSEL18__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL18__OUTPUT ((u32) 0x01000000)
#define GPIO_GPFSEL1_FSEL18__ALT5 ((u32) 0x02000000)
#define GPIO_GPFSEL1_FSEL18__ALT4 ((u32) 0x03000000)
#define GPIO_GPFSEL1_FSEL18__ALT0 ((u32) 0x04000000)
#define GPIO_GPFSEL1_FSEL18__ALT1 ((u32) 0x05000000)
#define GPIO_GPFSEL1_FSEL18__ALT2 ((u32) 0x06000000)
#define GPIO_GPFSEL1_FSEL18__ALT3 ((u32) 0x07000000)
/* FSEL19 - Function Select 19 bits (RW) */
#define GPIO_GPFSEL1_FSEL19_OFS (27)
#define GPIO_GPFSEL1_FSEL19_MASK ((u32) 0x38000000)
#define GPIO_GPFSEL1_FSEL190 ((u32) 0x08000000)
#define GPIO_GPFSEL1_FSEL191 ((u32) 0x10000000)
#define GPIO_GPFSEL1_FSEL192 ((u32) 0x20000000)
#define GPIO_GPFSEL1_FSEL19_0 ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL19_1 ((u32) 0x08000000)
#define GPIO_GPFSEL1_FSEL19_2 ((u32) 0x10000000)
#define GPIO_GPFSEL1_FSEL19_3 ((u32) 0x18000000)
#define GPIO_GPFSEL1_FSEL19_4 ((u32) 0x20000000)
#define GPIO_GPFSEL1_FSEL19_5 ((u32) 0x28000000)
#define GPIO_GPFSEL1_FSEL19_6 ((u32) 0x30000000)
#define GPIO_GPFSEL1_FSEL19_7 ((u32) 0x38000000)
#define GPIO_GPFSEL1_FSEL19__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL1_FSEL19__OUTPUT ((u32) 0x08000000)
#define GPIO_GPFSEL1_FSEL19__ALT5 ((u32) 0x10000000)
#define GPIO_GPFSEL1_FSEL19__ALT4 ((u32) 0x18000000)
#define GPIO_GPFSEL1_FSEL19__ALT0 ((u32) 0x20000000)
#define GPIO_GPFSEL1_FSEL19__ALT1 ((u32) 0x28000000)
#define GPIO_GPFSEL1_FSEL19__ALT2 ((u32) 0x30000000)
#define GPIO_GPFSEL1_FSEL19__ALT3 ((u32) 0x30000000)
/* GPIO Function Select 2 Register (RW) */
/* FSEL20 - Function Select 20 bits (RW) */
#define GPIO_GPFSEL2_FSEL20_OFS (0)
#define GPIO_GPFSEL2_FSEL20_MASK ((u32) 0x00000007)
#define GPIO_GPFSEL2_FSEL200 ((u32) 0x00000001)
#define GPIO_GPFSEL2_FSEL201 ((u32) 0x00000002)
#define GPIO_GPFSEL2_FSEL202 ((u32) 0x00000004)
#define GPIO_GPFSEL2_FSEL20_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL20_1 ((u32) 0x00000001)
#define GPIO_GPFSEL2_FSEL20_2 ((u32) 0x00000002)
#define GPIO_GPFSEL2_FSEL20_3 ((u32) 0x00000003)
#define GPIO_GPFSEL2_FSEL20_4 ((u32) 0x00000004)
#define GPIO_GPFSEL2_FSEL20_5 ((u32) 0x00000005)
#define GPIO_GPFSEL2_FSEL20_6 ((u32) 0x00000006)
#define GPIO_GPFSEL2_FSEL20_7 ((u32) 0x00000007)
#define GPIO_GPFSEL2_FSEL20__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL20__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL2_FSEL20__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL2_FSEL20__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL2_FSEL20__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL2_FSEL20__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL2_FSEL20__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL2_FSEL20__ALT3 ((u32) 0x00000007)
/* FSEL21 - Function Select 21 bits (RW) */
#define GPIO_GPFSEL2_FSEL21_OFS (3)
#define GPIO_GPFSEL2_FSEL21_MASK ((u32) 0x00000038)
#define GPIO_GPFSEL2_FSEL210 ((u32) 0x00000040)
#define GPIO_GPFSEL2_FSEL211 ((u32) 0x00000080)
#define GPIO_GPFSEL2_FSEL212 ((u32) 0x00000100)
#define GPIO_GPFSEL2_FSEL21_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL21_1 ((u32) 0x00000040)
#define GPIO_GPFSEL2_FSEL21_2 ((u32) 0x00000080)
#define GPIO_GPFSEL2_FSEL21_3 ((u32) 0x000000C0)
#define GPIO_GPFSEL2_FSEL21_4 ((u32) 0x00000100)
#define GPIO_GPFSEL2_FSEL21_5 ((u32) 0x00000140)
#define GPIO_GPFSEL2_FSEL21_6 ((u32) 0x00000180)
#define GPIO_GPFSEL2_FSEL21_7 ((u32) 0x000001C0)
#define GPIO_GPFSEL2_FSEL21__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL21__OUTPUT ((u32) 0x00000040)
#define GPIO_GPFSEL2_FSEL21__ALT5 ((u32) 0x00000080)
#define GPIO_GPFSEL2_FSEL21__ALT4 ((u32) 0x000000C0)
#define GPIO_GPFSEL2_FSEL21__ALT0 ((u32) 0x00000100)
#define GPIO_GPFSEL2_FSEL21__ALT1 ((u32) 0x00000140)
#define GPIO_GPFSEL2_FSEL21__ALT2 ((u32) 0x00000180)
#define GPIO_GPFSEL2_FSEL21__ALT3 ((u32) 0x000001C0)
/* FSEL22 - Function Select 22 bits (RW) */
#define GPIO_GPFSEL2_FSEL22_OFS (6)
#define GPIO_GPFSEL2_FSEL22_MASK ((u32) 0x000001C0)
#define GPIO_GPFSEL2_FSEL220 ((u32) 0x00000001)
#define GPIO_GPFSEL2_FSEL221 ((u32) 0x00000002)
#define GPIO_GPFSEL2_FSEL222 ((u32) 0x00000004)
#define GPIO_GPFSEL2_FSEL22_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL22_1 ((u32) 0x00000001)
#define GPIO_GPFSEL2_FSEL22_2 ((u32) 0x00000002)
#define GPIO_GPFSEL2_FSEL22_3 ((u32) 0x00000003)
#define GPIO_GPFSEL2_FSEL22_4 ((u32) 0x00000004)
#define GPIO_GPFSEL2_FSEL22_5 ((u32) 0x00000005)
#define GPIO_GPFSEL2_FSEL22_6 ((u32) 0x00000006)
#define GPIO_GPFSEL2_FSEL22_7 ((u32) 0x00000007)
#define GPIO_GPFSEL2_FSEL22__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL22__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL2_FSEL22__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL2_FSEL22__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL2_FSEL22__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL2_FSEL22__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL2_FSEL22__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL2_FSEL22__ALT3 ((u32) 0x00000007)
/* FSEL23 - Function Select 23 bits (RW) */
#define GPIO_GPFSEL2_FSEL23_OFS (9)
#define GPIO_GPFSEL2_FSEL23_MASK ((u32) 0x00000E00)
#define GPIO_GPFSEL2_FSEL230 ((u32) 0x00000200)
#define GPIO_GPFSEL2_FSEL231 ((u32) 0x00000400)
#define GPIO_GPFSEL2_FSEL232 ((u32) 0x00000800)
#define GPIO_GPFSEL2_FSEL23_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL23_1 ((u32) 0x00000200)
#define GPIO_GPFSEL2_FSEL23_2 ((u32) 0x00000400)
#define GPIO_GPFSEL2_FSEL23_3 ((u32) 0x00000600)
#define GPIO_GPFSEL2_FSEL23_4 ((u32) 0x00000800)
#define GPIO_GPFSEL2_FSEL23_5 ((u32) 0x00000A00)
#define GPIO_GPFSEL2_FSEL23_6 ((u32) 0x00000C00)
#define GPIO_GPFSEL2_FSEL23_7 ((u32) 0x00000E00)
#define GPIO_GPFSEL2_FSEL23__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL23__OUTPUT ((u32) 0x0000200)
#define GPIO_GPFSEL2_FSEL23__ALT5 ((u32) 0x00000400)
#define GPIO_GPFSEL2_FSEL23__ALT4 ((u32) 0x00000600)
#define GPIO_GPFSEL2_FSEL23__ALT0 ((u32) 0x00000800)
#define GPIO_GPFSEL2_FSEL23__ALT1 ((u32) 0x00000A00)
#define GPIO_GPFSEL2_FSEL23__ALT2 ((u32) 0x00000C00)
#define GPIO_GPFSEL2_FSEL23__ALT3 ((u32) 0x00000E00)
/* FSEL24 - Function Select 24 bits (RW) */
#define GPIO_GPFSEL2_FSEL24_OFS (12)
#define GPIO_GPFSEL2_FSEL24_MASK ((u32) 0x00007000)
#define GPIO_GPFSEL2_FSEL240 ((u32) 0x00001000)
#define GPIO_GPFSEL2_FSEL241 ((u32) 0x00002000)
#define GPIO_GPFSEL2_FSEL242 ((u32) 0x00004000)
#define GPIO_GPFSEL2_FSEL24_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL24_1 ((u32) 0x00001000)
#define GPIO_GPFSEL2_FSEL24_2 ((u32) 0x00002000)
#define GPIO_GPFSEL2_FSEL24_3 ((u32) 0x00003000)
#define GPIO_GPFSEL2_FSEL24_4 ((u32) 0x00004000)
#define GPIO_GPFSEL2_FSEL24_5 ((u32) 0x00005000)
#define GPIO_GPFSEL2_FSEL24_6 ((u32) 0x00006000)
#define GPIO_GPFSEL2_FSEL24_7 ((u32) 0x00007000)
#define GPIO_GPFSEL2_FSEL24__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL24__OUTPUT ((u32) 0x00001000)
#define GPIO_GPFSEL2_FSEL24__ALT5 ((u32) 0x00002000)
#define GPIO_GPFSEL2_FSEL24__ALT4 ((u32) 0x00003000)
#define GPIO_GPFSEL2_FSEL24__ALT0 ((u32) 0x00004000)
#define GPIO_GPFSEL2_FSEL24__ALT1 ((u32) 0x00005000)
#define GPIO_GPFSEL2_FSEL24__ALT2 ((u32) 0x00006000)
#define GPIO_GPFSEL2_FSEL24__ALT3 ((u32) 0x00007000)
/* FSEL25 - Function Select 25 bits (RW) */
#define GPIO_GPFSEL2_FSEL25_OFS (15)
#define GPIO_GPFSEL2_FSEL25_MASK ((u32) 0x00038000)
#define GPIO_GPFSEL2_FSEL250 ((u32) 0x00008000)
#define GPIO_GPFSEL2_FSEL251 ((u32) 0x00010000)
#define GPIO_GPFSEL2_FSEL252 ((u32) 0x00020000)
#define GPIO_GPFSEL2_FSEL25_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL25_1 ((u32) 0x00080000)
#define GPIO_GPFSEL2_FSEL25_2 ((u32) 0x00100000)
#define GPIO_GPFSEL2_FSEL25_3 ((u32) 0x00180000)
#define GPIO_GPFSEL2_FSEL25_4 ((u32) 0x00200000)
#define GPIO_GPFSEL2_FSEL25_5 ((u32) 0x00280000)
#define GPIO_GPFSEL2_FSEL25_6 ((u32) 0x00300000)
#define GPIO_GPFSEL2_FSEL25_7 ((u32) 0x00380000)
#define GPIO_GPFSEL2_FSEL25__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL25__OUTPUT ((u32) 0x00080000)
#define GPIO_GPFSEL2_FSEL25__ALT5 ((u32) 0x00100000)
#define GPIO_GPFSEL2_FSEL25__ALT4 ((u32) 0x00180000)
#define GPIO_GPFSEL2_FSEL25__ALT0 ((u32) 0x00200000)
#define GPIO_GPFSEL2_FSEL25__ALT1 ((u32) 0x00280000)
#define GPIO_GPFSEL2_FSEL25__ALT2 ((u32) 0x00300000)
#define GPIO_GPFSEL2_FSEL25__ALT3 ((u32) 0x00380000)
/* FSEL26 - Function Select 26 bits (RW) */
#define GPIO_GPFSEL2_FSEL26_OFS (18)
#define GPIO_GPFSEL2_FSEL26_MASK ((u32) 0x001C0000)
#define GPIO_GPFSEL2_FSEL260 ((u32) 0x00040000)
#define GPIO_GPFSEL2_FSEL261 ((u32) 0x00080000)
#define GPIO_GPFSEL2_FSEL262 ((u32) 0x00100000)
#define GPIO_GPFSEL2_FSEL26_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL26_1 ((u32) 0x00040000)
#define GPIO_GPFSEL2_FSEL26_2 ((u32) 0x00080000)
#define GPIO_GPFSEL2_FSEL26_3 ((u32) 0x000C0000)
#define GPIO_GPFSEL2_FSEL26_4 ((u32) 0x00100000)
#define GPIO_GPFSEL2_FSEL26_5 ((u32) 0x00140000)
#define GPIO_GPFSEL2_FSEL26_6 ((u32) 0x00180000)
#define GPIO_GPFSEL2_FSEL26_7 ((u32) 0x001C0000)
#define GPIO_GPFSEL2_FSEL26__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL26__OUTPUT ((u32) 0x00040000)
#define GPIO_GPFSEL2_FSEL26__ALT5 ((u32) 0x00080000)
#define GPIO_GPFSEL2_FSEL26__ALT4 ((u32) 0x000C0000)
#define GPIO_GPFSEL2_FSEL26__ALT0 ((u32) 0x00100000)
#define GPIO_GPFSEL2_FSEL26__ALT1 ((u32) 0x00140000)
#define GPIO_GPFSEL2_FSEL26__ALT2 ((u32) 0x00180000)
#define GPIO_GPFSEL2_FSEL26__ALT3 ((u32) 0x001C0000)
/* FSEL27 - Function Select 27 bits (RW) */
#define GPIO_GPFSEL2_FSEL27_OFS (21)
#define GPIO_GPFSEL2_FSEL27_MASK ((u32) 0x00E00000)
#define GPIO_GPFSEL2_FSEL270 ((u32) 0x00200000)
#define GPIO_GPFSEL2_FSEL271 ((u32) 0x00400000)
#define GPIO_GPFSEL2_FSEL272 ((u32) 0x00800000)
#define GPIO_GPFSEL2_FSEL27_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL27_1 ((u32) 0x00200000)
#define GPIO_GPFSEL2_FSEL27_2 ((u32) 0x00400000)
#define GPIO_GPFSEL2_FSEL27_3 ((u32) 0x00600000)
#define GPIO_GPFSEL2_FSEL27_4 ((u32) 0x00800000)
#define GPIO_GPFSEL2_FSEL27_5 ((u32) 0x00A00000)
#define GPIO_GPFSEL2_FSEL27_6 ((u32) 0x00C00000)
#define GPIO_GPFSEL2_FSEL27_7 ((u32) 0x00E00000)
#define GPIO_GPFSEL2_FSEL27__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL27__OUTPUT ((u32) 0x00200000)
#define GPIO_GPFSEL2_FSEL27__ALT5 ((u32) 0x00400000)
#define GPIO_GPFSEL2_FSEL27__ALT4 ((u32) 0x00600000)
#define GPIO_GPFSEL2_FSEL27__ALT0 ((u32) 0x00800000)
#define GPIO_GPFSEL2_FSEL27__ALT1 ((u32) 0x00A00000)
#define GPIO_GPFSEL2_FSEL27__ALT2 ((u32) 0x00C00000)
#define GPIO_GPFSEL2_FSEL27__ALT3 ((u32) 0x00E00000)
/* FSEL28 - Function Select 28 bits (RW) */
#define GPIO_GPFSEL2_FSEL28_OFS (24)
#define GPIO_GPFSEL2_FSEL28_MASK ((u32) 0x07000000)
#define GPIO_GPFSEL2_FSEL280 ((u32) 0x01000000)
#define GPIO_GPFSEL2_FSEL281 ((u32) 0x02000000)
#define GPIO_GPFSEL2_FSEL282 ((u32) 0x04000000)
#define GPIO_GPFSEL2_FSEL28_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL28_1 ((u32) 0x01000000)
#define GPIO_GPFSEL2_FSEL28_2 ((u32) 0x02000000)
#define GPIO_GPFSEL2_FSEL28_3 ((u32) 0x03000000)
#define GPIO_GPFSEL2_FSEL28_4 ((u32) 0x04000000)
#define GPIO_GPFSEL2_FSEL28_5 ((u32) 0x05000000)
#define GPIO_GPFSEL2_FSEL28_6 ((u32) 0x06000000)
#define GPIO_GPFSEL2_FSEL28_7 ((u32) 0x07000000)
#define GPIO_GPFSEL2_FSEL28__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL28__OUTPUT ((u32) 0x01000000)
#define GPIO_GPFSEL2_FSEL28__ALT5 ((u32) 0x02000000)
#define GPIO_GPFSEL2_FSEL28__ALT4 ((u32) 0x03000000)
#define GPIO_GPFSEL2_FSEL28__ALT0 ((u32) 0x04000000)
#define GPIO_GPFSEL2_FSEL28__ALT1 ((u32) 0x05000000)
#define GPIO_GPFSEL2_FSEL28__ALT2 ((u32) 0x06000000)
#define GPIO_GPFSEL2_FSEL28__ALT3 ((u32) 0x07000000)
/* FSEL29 - Function Select 29 bits (RW) */
#define GPIO_GPFSEL2_FSEL29_OFS (27)
#define GPIO_GPFSEL2_FSEL29_MASK ((u32) 0x38000000)
#define GPIO_GPFSEL2_FSEL290 ((u32) 0x08000000)
#define GPIO_GPFSEL2_FSEL291 ((u32) 0x10000000)
#define GPIO_GPFSEL2_FSEL292 ((u32) 0x20000000)
#define GPIO_GPFSEL2_FSEL29_0 ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL29_1 ((u32) 0x08000000)
#define GPIO_GPFSEL2_FSEL29_2 ((u32) 0x10000000)
#define GPIO_GPFSEL2_FSEL29_3 ((u32) 0x18000000)
#define GPIO_GPFSEL2_FSEL29_4 ((u32) 0x20000000)
#define GPIO_GPFSEL2_FSEL29_5 ((u32) 0x28000000)
#define GPIO_GPFSEL2_FSEL29_6 ((u32) 0x30000000)
#define GPIO_GPFSEL2_FSEL29_7 ((u32) 0x38000000)
#define GPIO_GPFSEL2_FSEL29__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL2_FSEL29__OUTPUT ((u32) 0x08000000)
#define GPIO_GPFSEL2_FSEL29__ALT5 ((u32) 0x10000000)
#define GPIO_GPFSEL2_FSEL29__ALT4 ((u32) 0x18000000)
#define GPIO_GPFSEL2_FSEL29__ALT0 ((u32) 0x20000000)
#define GPIO_GPFSEL2_FSEL29__ALT1 ((u32) 0x28000000)
#define GPIO_GPFSEL2_FSEL29__ALT2 ((u32) 0x30000000)
#define GPIO_GPFSEL2_FSEL29__ALT3 ((u32) 0x30000000)
/* GPIO Function Select 3 Register (RW) */
/* FSEL30 - Function Select 30 bits (RW) */
#define GPIO_GPFSEL3_FSEL30_OFS (0)
#define GPIO_GPFSEL3_FSEL30_MASK ((u32) 0x00000007)
#define GPIO_GPFSEL3_FSEL300 ((u32) 0x00000001)
#define GPIO_GPFSEL3_FSEL301 ((u32) 0x00000002)
#define GPIO_GPFSEL3_FSEL302 ((u32) 0x00000004)
#define GPIO_GPFSEL3_FSEL30_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL30_1 ((u32) 0x00000001)
#define GPIO_GPFSEL3_FSEL30_2 ((u32) 0x00000002)
#define GPIO_GPFSEL3_FSEL30_3 ((u32) 0x00000003)
#define GPIO_GPFSEL3_FSEL30_4 ((u32) 0x00000004)
#define GPIO_GPFSEL3_FSEL30_5 ((u32) 0x00000005)
#define GPIO_GPFSEL3_FSEL30_6 ((u32) 0x00000006)
#define GPIO_GPFSEL3_FSEL30_7 ((u32) 0x00000007)
#define GPIO_GPFSEL3_FSEL30__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL30__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL3_FSEL30__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL3_FSEL30__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL3_FSEL30__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL3_FSEL30__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL3_FSEL30__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL3_FSEL30__ALT3 ((u32) 0x00000007)
/* FSEL31 - Function Select 31 bits (RW) */
#define GPIO_GPFSEL3_FSEL31_OFS (3)
#define GPIO_GPFSEL3_FSEL31_MASK ((u32) 0x00000038)
#define GPIO_GPFSEL3_FSEL310 ((u32) 0x00000040)
#define GPIO_GPFSEL3_FSEL311 ((u32) 0x00000080)
#define GPIO_GPFSEL3_FSEL312 ((u32) 0x00000100)
#define GPIO_GPFSEL3_FSEL31_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL31_1 ((u32) 0x00000040)
#define GPIO_GPFSEL3_FSEL31_2 ((u32) 0x00000080)
#define GPIO_GPFSEL3_FSEL31_3 ((u32) 0x000000C0)
#define GPIO_GPFSEL3_FSEL31_4 ((u32) 0x00000100)
#define GPIO_GPFSEL3_FSEL31_5 ((u32) 0x00000140)
#define GPIO_GPFSEL3_FSEL31_6 ((u32) 0x00000180)
#define GPIO_GPFSEL3_FSEL31_7 ((u32) 0x000001C0)
#define GPIO_GPFSEL3_FSEL31__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL31__OUTPUT ((u32) 0x00000040)
#define GPIO_GPFSEL3_FSEL31__ALT5 ((u32) 0x00000080)
#define GPIO_GPFSEL3_FSEL31__ALT4 ((u32) 0x000000C0)
#define GPIO_GPFSEL3_FSEL31__ALT0 ((u32) 0x00000100)
#define GPIO_GPFSEL3_FSEL31__ALT1 ((u32) 0x00000140)
#define GPIO_GPFSEL3_FSEL31__ALT2 ((u32) 0x00000180)
#define GPIO_GPFSEL3_FSEL31__ALT3 ((u32) 0x000001C0)
/* FSEL32 - Function Select 32 bits (RW) */
#define GPIO_GPFSEL3_FSEL32_OFS (6)
#define GPIO_GPFSEL3_FSEL32_MASK ((u32) 0x000001C0)
#define GPIO_GPFSEL3_FSEL320 ((u32) 0x00000001)
#define GPIO_GPFSEL3_FSEL321 ((u32) 0x00000002)
#define GPIO_GPFSEL3_FSEL322 ((u32) 0x00000004)
#define GPIO_GPFSEL3_FSEL32_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL32_1 ((u32) 0x00000001)
#define GPIO_GPFSEL3_FSEL32_2 ((u32) 0x00000002)
#define GPIO_GPFSEL3_FSEL32_3 ((u32) 0x00000003)
#define GPIO_GPFSEL3_FSEL32_4 ((u32) 0x00000004)
#define GPIO_GPFSEL3_FSEL32_5 ((u32) 0x00000005)
#define GPIO_GPFSEL3_FSEL32_6 ((u32) 0x00000006)
#define GPIO_GPFSEL3_FSEL32_7 ((u32) 0x00000007)
#define GPIO_GPFSEL3_FSEL32__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL32__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL3_FSEL32__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL3_FSEL32__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL3_FSEL32__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL3_FSEL32__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL3_FSEL32__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL3_FSEL32__ALT3 ((u32) 0x00000007)
/* FSEL33 - Function Select 33 bits (RW) */
#define GPIO_GPFSEL3_FSEL33_OFS (9)
#define GPIO_GPFSEL3_FSEL33_MASK ((u32) 0x00000E00)
#define GPIO_GPFSEL3_FSEL330 ((u32) 0x00000200)
#define GPIO_GPFSEL3_FSEL331 ((u32) 0x00000400)
#define GPIO_GPFSEL3_FSEL332 ((u32) 0x00000800)
#define GPIO_GPFSEL3_FSEL33_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL33_1 ((u32) 0x00000200)
#define GPIO_GPFSEL3_FSEL33_2 ((u32) 0x00000400)
#define GPIO_GPFSEL3_FSEL33_3 ((u32) 0x00000600)
#define GPIO_GPFSEL3_FSEL33_4 ((u32) 0x00000800)
#define GPIO_GPFSEL3_FSEL33_5 ((u32) 0x00000A00)
#define GPIO_GPFSEL3_FSEL33_6 ((u32) 0x00000C00)
#define GPIO_GPFSEL3_FSEL33_7 ((u32) 0x00000E00)
#define GPIO_GPFSEL3_FSEL33__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL33__OUTPUT ((u32) 0x0000200)
#define GPIO_GPFSEL3_FSEL33__ALT5 ((u32) 0x00000400)
#define GPIO_GPFSEL3_FSEL33__ALT4 ((u32) 0x00000600)
#define GPIO_GPFSEL3_FSEL33__ALT0 ((u32) 0x00000800)
#define GPIO_GPFSEL3_FSEL33__ALT1 ((u32) 0x00000A00)
#define GPIO_GPFSEL3_FSEL33__ALT2 ((u32) 0x00000C00)
#define GPIO_GPFSEL3_FSEL33__ALT3 ((u32) 0x00000E00)
/* FSEL34 - Function Select 34 bits (RW) */
#define GPIO_GPFSEL3_FSEL34_OFS (12)
#define GPIO_GPFSEL3_FSEL34_MASK ((u32) 0x00007000)
#define GPIO_GPFSEL3_FSEL340 ((u32) 0x00001000)
#define GPIO_GPFSEL3_FSEL341 ((u32) 0x00002000)
#define GPIO_GPFSEL3_FSEL342 ((u32) 0x00004000)
#define GPIO_GPFSEL3_FSEL34_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL34_1 ((u32) 0x00001000)
#define GPIO_GPFSEL3_FSEL34_2 ((u32) 0x00002000)
#define GPIO_GPFSEL3_FSEL34_3 ((u32) 0x00003000)
#define GPIO_GPFSEL3_FSEL34_4 ((u32) 0x00004000)
#define GPIO_GPFSEL3_FSEL34_5 ((u32) 0x00005000)
#define GPIO_GPFSEL3_FSEL34_6 ((u32) 0x00006000)
#define GPIO_GPFSEL3_FSEL34_7 ((u32) 0x00007000)
#define GPIO_GPFSEL3_FSEL34__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL34__OUTPUT ((u32) 0x00001000)
#define GPIO_GPFSEL3_FSEL34__ALT5 ((u32) 0x00002000)
#define GPIO_GPFSEL3_FSEL34__ALT4 ((u32) 0x00003000)
#define GPIO_GPFSEL3_FSEL34__ALT0 ((u32) 0x00004000)
#define GPIO_GPFSEL3_FSEL34__ALT1 ((u32) 0x00005000)
#define GPIO_GPFSEL3_FSEL34__ALT2 ((u32) 0x00006000)
#define GPIO_GPFSEL3_FSEL34__ALT3 ((u32) 0x00007000)
/* FSEL35 - Function Select 35 bits (RW) */
#define GPIO_GPFSEL3_FSEL35_OFS (15)
#define GPIO_GPFSEL3_FSEL35_MASK ((u32) 0x00038000)
#define GPIO_GPFSEL3_FSEL350 ((u32) 0x00008000)
#define GPIO_GPFSEL3_FSEL351 ((u32) 0x00010000)
#define GPIO_GPFSEL3_FSEL352 ((u32) 0x00020000)
#define GPIO_GPFSEL3_FSEL35_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL35_1 ((u32) 0x00080000)
#define GPIO_GPFSEL3_FSEL35_2 ((u32) 0x00100000)
#define GPIO_GPFSEL3_FSEL35_3 ((u32) 0x00180000)
#define GPIO_GPFSEL3_FSEL35_4 ((u32) 0x00200000)
#define GPIO_GPFSEL3_FSEL35_5 ((u32) 0x00280000)
#define GPIO_GPFSEL3_FSEL35_6 ((u32) 0x00300000)
#define GPIO_GPFSEL3_FSEL35_7 ((u32) 0x00380000)
#define GPIO_GPFSEL3_FSEL35__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL35__OUTPUT ((u32) 0x00080000)
#define GPIO_GPFSEL3_FSEL35__ALT5 ((u32) 0x00100000)
#define GPIO_GPFSEL3_FSEL35__ALT4 ((u32) 0x00180000)
#define GPIO_GPFSEL3_FSEL35__ALT0 ((u32) 0x00200000)
#define GPIO_GPFSEL3_FSEL35__ALT1 ((u32) 0x00280000)
#define GPIO_GPFSEL3_FSEL35__ALT2 ((u32) 0x00300000)
#define GPIO_GPFSEL3_FSEL35__ALT3 ((u32) 0x00380000)
/* FSEL36 - Function Select 36 bits (RW) */
#define GPIO_GPFSEL3_FSEL36_OFS (18)
#define GPIO_GPFSEL3_FSEL36_MASK ((u32) 0x001C0000)
#define GPIO_GPFSEL3_FSEL360 ((u32) 0x00040000)
#define GPIO_GPFSEL3_FSEL361 ((u32) 0x00080000)
#define GPIO_GPFSEL3_FSEL362 ((u32) 0x00100000)
#define GPIO_GPFSEL3_FSEL36_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL36_1 ((u32) 0x00040000)
#define GPIO_GPFSEL3_FSEL36_2 ((u32) 0x00080000)
#define GPIO_GPFSEL3_FSEL36_3 ((u32) 0x000C0000)
#define GPIO_GPFSEL3_FSEL36_4 ((u32) 0x00100000)
#define GPIO_GPFSEL3_FSEL36_5 ((u32) 0x00140000)
#define GPIO_GPFSEL3_FSEL36_6 ((u32) 0x00180000)
#define GPIO_GPFSEL3_FSEL36_7 ((u32) 0x001C0000)
#define GPIO_GPFSEL3_FSEL36__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL36__OUTPUT ((u32) 0x00040000)
#define GPIO_GPFSEL3_FSEL36__ALT5 ((u32) 0x00080000)
#define GPIO_GPFSEL3_FSEL36__ALT4 ((u32) 0x000C0000)
#define GPIO_GPFSEL3_FSEL36__ALT0 ((u32) 0x00100000)
#define GPIO_GPFSEL3_FSEL36__ALT1 ((u32) 0x00140000)
#define GPIO_GPFSEL3_FSEL36__ALT2 ((u32) 0x00180000)
#define GPIO_GPFSEL3_FSEL36__ALT3 ((u32) 0x001C0000)
/* FSEL37 - Function Select 37 bits (RW) */
#define GPIO_GPFSEL3_FSEL37_OFS (21)
#define GPIO_GPFSEL3_FSEL37_MASK ((u32) 0x00E00000)
#define GPIO_GPFSEL3_FSEL370 ((u32) 0x00200000)
#define GPIO_GPFSEL3_FSEL371 ((u32) 0x00400000)
#define GPIO_GPFSEL3_FSEL372 ((u32) 0x00800000)
#define GPIO_GPFSEL3_FSEL37_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL37_1 ((u32) 0x00200000)
#define GPIO_GPFSEL3_FSEL37_2 ((u32) 0x00400000)
#define GPIO_GPFSEL3_FSEL37_3 ((u32) 0x00600000)
#define GPIO_GPFSEL3_FSEL37_4 ((u32) 0x00800000)
#define GPIO_GPFSEL3_FSEL37_5 ((u32) 0x00A00000)
#define GPIO_GPFSEL3_FSEL37_6 ((u32) 0x00C00000)
#define GPIO_GPFSEL3_FSEL37_7 ((u32) 0x00E00000)
#define GPIO_GPFSEL3_FSEL37__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL37__OUTPUT ((u32) 0x00200000)
#define GPIO_GPFSEL3_FSEL37__ALT5 ((u32) 0x00400000)
#define GPIO_GPFSEL3_FSEL37__ALT4 ((u32) 0x00600000)
#define GPIO_GPFSEL3_FSEL37__ALT0 ((u32) 0x00800000)
#define GPIO_GPFSEL3_FSEL37__ALT1 ((u32) 0x00A00000)
#define GPIO_GPFSEL3_FSEL37__ALT2 ((u32) 0x00C00000)
#define GPIO_GPFSEL3_FSEL37__ALT3 ((u32) 0x00E00000)
/* FSEL38 - Function Select 38 bits (RW) */
#define GPIO_GPFSEL3_FSEL38_OFS (24)
#define GPIO_GPFSEL3_FSEL38_MASK ((u32) 0x07000000)
#define GPIO_GPFSEL3_FSEL380 ((u32) 0x01000000)
#define GPIO_GPFSEL3_FSEL381 ((u32) 0x02000000)
#define GPIO_GPFSEL3_FSEL382 ((u32) 0x04000000)
#define GPIO_GPFSEL3_FSEL38_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL38_1 ((u32) 0x01000000)
#define GPIO_GPFSEL3_FSEL38_2 ((u32) 0x02000000)
#define GPIO_GPFSEL3_FSEL38_3 ((u32) 0x03000000)
#define GPIO_GPFSEL3_FSEL38_4 ((u32) 0x04000000)
#define GPIO_GPFSEL3_FSEL38_5 ((u32) 0x05000000)
#define GPIO_GPFSEL3_FSEL38_6 ((u32) 0x06000000)
#define GPIO_GPFSEL3_FSEL38_7 ((u32) 0x07000000)
#define GPIO_GPFSEL3_FSEL38__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL38__OUTPUT ((u32) 0x01000000)
#define GPIO_GPFSEL3_FSEL38__ALT5 ((u32) 0x02000000)
#define GPIO_GPFSEL3_FSEL38__ALT4 ((u32) 0x03000000)
#define GPIO_GPFSEL3_FSEL38__ALT0 ((u32) 0x04000000)
#define GPIO_GPFSEL3_FSEL38__ALT1 ((u32) 0x05000000)
#define GPIO_GPFSEL3_FSEL38__ALT2 ((u32) 0x06000000)
#define GPIO_GPFSEL3_FSEL38__ALT3 ((u32) 0x07000000)
/* FSEL39 - Function Select 39 bits (RW) */
#define GPIO_GPFSEL3_FSEL39_OFS (27)
#define GPIO_GPFSEL3_FSEL39_MASK ((u32) 0x38000000)
#define GPIO_GPFSEL3_FSEL390 ((u32) 0x08000000)
#define GPIO_GPFSEL3_FSEL391 ((u32) 0x10000000)
#define GPIO_GPFSEL3_FSEL392 ((u32) 0x20000000)
#define GPIO_GPFSEL3_FSEL39_0 ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL39_1 ((u32) 0x08000000)
#define GPIO_GPFSEL3_FSEL39_2 ((u32) 0x10000000)
#define GPIO_GPFSEL3_FSEL39_3 ((u32) 0x18000000)
#define GPIO_GPFSEL3_FSEL39_4 ((u32) 0x20000000)
#define GPIO_GPFSEL3_FSEL39_5 ((u32) 0x28000000)
#define GPIO_GPFSEL3_FSEL39_6 ((u32) 0x30000000)
#define GPIO_GPFSEL3_FSEL39_7 ((u32) 0x38000000)
#define GPIO_GPFSEL3_FSEL39__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL3_FSEL39__OUTPUT ((u32) 0x08000000)
#define GPIO_GPFSEL3_FSEL39__ALT5 ((u32) 0x10000000)
#define GPIO_GPFSEL3_FSEL39__ALT4 ((u32) 0x18000000)
#define GPIO_GPFSEL3_FSEL39__ALT0 ((u32) 0x20000000)
#define GPIO_GPFSEL3_FSEL39__ALT1 ((u32) 0x28000000)
#define GPIO_GPFSEL3_FSEL39__ALT2 ((u32) 0x30000000)
#define GPIO_GPFSEL3_FSEL39__ALT3 ((u32) 0x30000000)
/* GPIO Function Select 4 Register (RW) */
/* FSEL40 - Function Select 40 bits (RW) */
#define GPIO_GPFSEL4_FSEL40_OFS (0)
#define GPIO_GPFSEL4_FSEL40_MASK ((u32) 0x00000007)
#define GPIO_GPFSEL4_FSEL400 ((u32) 0x00000001)
#define GPIO_GPFSEL4_FSEL401 ((u32) 0x00000002)
#define GPIO_GPFSEL4_FSEL402 ((u32) 0x00000004)
#define GPIO_GPFSEL4_FSEL40_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL40_1 ((u32) 0x00000001)
#define GPIO_GPFSEL4_FSEL40_2 ((u32) 0x00000002)
#define GPIO_GPFSEL4_FSEL40_3 ((u32) 0x00000003)
#define GPIO_GPFSEL4_FSEL40_4 ((u32) 0x00000004)
#define GPIO_GPFSEL4_FSEL40_5 ((u32) 0x00000005)
#define GPIO_GPFSEL4_FSEL40_6 ((u32) 0x00000006)
#define GPIO_GPFSEL4_FSEL40_7 ((u32) 0x00000007)
#define GPIO_GPFSEL4_FSEL40__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL40__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL4_FSEL40__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL4_FSEL40__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL4_FSEL40__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL4_FSEL40__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL4_FSEL40__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL4_FSEL40__ALT3 ((u32) 0x00000007)
/* FSEL41 - Function Select 41 bits (RW) */
#define GPIO_GPFSEL4_FSEL41_OFS (3)
#define GPIO_GPFSEL4_FSEL41_MASK ((u32) 0x00000038)
#define GPIO_GPFSEL4_FSEL410 ((u32) 0x00000040)
#define GPIO_GPFSEL4_FSEL411 ((u32) 0x00000080)
#define GPIO_GPFSEL4_FSEL412 ((u32) 0x00000100)
#define GPIO_GPFSEL4_FSEL41_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL41_1 ((u32) 0x00000040)
#define GPIO_GPFSEL4_FSEL41_2 ((u32) 0x00000080)
#define GPIO_GPFSEL4_FSEL41_3 ((u32) 0x000000C0)
#define GPIO_GPFSEL4_FSEL41_4 ((u32) 0x00000100)
#define GPIO_GPFSEL4_FSEL41_5 ((u32) 0x00000140)
#define GPIO_GPFSEL4_FSEL41_6 ((u32) 0x00000180)
#define GPIO_GPFSEL4_FSEL41_7 ((u32) 0x000001C0)
#define GPIO_GPFSEL4_FSEL41__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL41__OUTPUT ((u32) 0x00000040)
#define GPIO_GPFSEL4_FSEL41__ALT5 ((u32) 0x00000080)
#define GPIO_GPFSEL4_FSEL41__ALT4 ((u32) 0x000000C0)
#define GPIO_GPFSEL4_FSEL41__ALT0 ((u32) 0x00000100)
#define GPIO_GPFSEL4_FSEL41__ALT1 ((u32) 0x00000140)
#define GPIO_GPFSEL4_FSEL41__ALT2 ((u32) 0x00000180)
#define GPIO_GPFSEL4_FSEL41__ALT3 ((u32) 0x000001C0)
/* FSEL42 - Function Select 42 bits (RW) */
#define GPIO_GPFSEL4_FSEL42_OFS (6)
#define GPIO_GPFSEL4_FSEL42_MASK ((u32) 0x000001C0)
#define GPIO_GPFSEL4_FSEL420 ((u32) 0x00000001)
#define GPIO_GPFSEL4_FSEL421 ((u32) 0x00000002)
#define GPIO_GPFSEL4_FSEL422 ((u32) 0x00000004)
#define GPIO_GPFSEL4_FSEL42_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL42_1 ((u32) 0x00000001)
#define GPIO_GPFSEL4_FSEL42_2 ((u32) 0x00000002)
#define GPIO_GPFSEL4_FSEL42_3 ((u32) 0x00000003)
#define GPIO_GPFSEL4_FSEL42_4 ((u32) 0x00000004)
#define GPIO_GPFSEL4_FSEL42_5 ((u32) 0x00000005)
#define GPIO_GPFSEL4_FSEL42_6 ((u32) 0x00000006)
#define GPIO_GPFSEL4_FSEL42_7 ((u32) 0x00000007)
#define GPIO_GPFSEL4_FSEL42__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL42__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL4_FSEL42__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL4_FSEL42__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL4_FSEL42__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL4_FSEL42__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL4_FSEL42__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL4_FSEL42__ALT3 ((u32) 0x00000007)
/* FSEL43 - Function Select 43 bits (RW) */
#define GPIO_GPFSEL4_FSEL43_OFS (9)
#define GPIO_GPFSEL4_FSEL43_MASK ((u32) 0x00000E00)
#define GPIO_GPFSEL4_FSEL430 ((u32) 0x00000200)
#define GPIO_GPFSEL4_FSEL431 ((u32) 0x00000400)
#define GPIO_GPFSEL4_FSEL432 ((u32) 0x00000800)
#define GPIO_GPFSEL4_FSEL43_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL43_1 ((u32) 0x00000200)
#define GPIO_GPFSEL4_FSEL43_2 ((u32) 0x00000400)
#define GPIO_GPFSEL4_FSEL43_3 ((u32) 0x00000600)
#define GPIO_GPFSEL4_FSEL43_4 ((u32) 0x00000800)
#define GPIO_GPFSEL4_FSEL43_5 ((u32) 0x00000A00)
#define GPIO_GPFSEL4_FSEL43_6 ((u32) 0x00000C00)
#define GPIO_GPFSEL4_FSEL43_7 ((u32) 0x00000E00)
#define GPIO_GPFSEL4_FSEL43__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL43__OUTPUT ((u32) 0x0000200)
#define GPIO_GPFSEL4_FSEL43__ALT5 ((u32) 0x00000400)
#define GPIO_GPFSEL4_FSEL43__ALT4 ((u32) 0x00000600)
#define GPIO_GPFSEL4_FSEL43__ALT0 ((u32) 0x00000800)
#define GPIO_GPFSEL4_FSEL43__ALT1 ((u32) 0x00000A00)
#define GPIO_GPFSEL4_FSEL43__ALT2 ((u32) 0x00000C00)
#define GPIO_GPFSEL4_FSEL43__ALT3 ((u32) 0x00000E00)
/* FSEL44 - Function Select 44 bits (RW) */
#define GPIO_GPFSEL4_FSEL44_OFS (12)
#define GPIO_GPFSEL4_FSEL44_MASK ((u32) 0x00007000)
#define GPIO_GPFSEL4_FSEL440 ((u32) 0x00001000)
#define GPIO_GPFSEL4_FSEL441 ((u32) 0x00002000)
#define GPIO_GPFSEL4_FSEL442 ((u32) 0x00004000)
#define GPIO_GPFSEL4_FSEL44_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL44_1 ((u32) 0x00001000)
#define GPIO_GPFSEL4_FSEL44_2 ((u32) 0x00002000)
#define GPIO_GPFSEL4_FSEL44_3 ((u32) 0x00003000)
#define GPIO_GPFSEL4_FSEL44_4 ((u32) 0x00004000)
#define GPIO_GPFSEL4_FSEL44_5 ((u32) 0x00005000)
#define GPIO_GPFSEL4_FSEL44_6 ((u32) 0x00006000)
#define GPIO_GPFSEL4_FSEL44_7 ((u32) 0x00007000)
#define GPIO_GPFSEL4_FSEL44__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL44__OUTPUT ((u32) 0x00001000)
#define GPIO_GPFSEL4_FSEL44__ALT5 ((u32) 0x00002000)
#define GPIO_GPFSEL4_FSEL44__ALT4 ((u32) 0x00003000)
#define GPIO_GPFSEL4_FSEL44__ALT0 ((u32) 0x00004000)
#define GPIO_GPFSEL4_FSEL44__ALT1 ((u32) 0x00005000)
#define GPIO_GPFSEL4_FSEL44__ALT2 ((u32) 0x00006000)
#define GPIO_GPFSEL4_FSEL44__ALT3 ((u32) 0x00007000)
/* FSEL45 - Function Select 45 bits (RW) */
#define GPIO_GPFSEL4_FSEL45_OFS (15)
#define GPIO_GPFSEL4_FSEL45_MASK ((u32) 0x00038000)
#define GPIO_GPFSEL4_FSEL450 ((u32) 0x00008000)
#define GPIO_GPFSEL4_FSEL451 ((u32) 0x00010000)
#define GPIO_GPFSEL4_FSEL452 ((u32) 0x00020000)
#define GPIO_GPFSEL4_FSEL45_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL45_1 ((u32) 0x00080000)
#define GPIO_GPFSEL4_FSEL45_2 ((u32) 0x00100000)
#define GPIO_GPFSEL4_FSEL45_3 ((u32) 0x00180000)
#define GPIO_GPFSEL4_FSEL45_4 ((u32) 0x00200000)
#define GPIO_GPFSEL4_FSEL45_5 ((u32) 0x00280000)
#define GPIO_GPFSEL4_FSEL45_6 ((u32) 0x00300000)
#define GPIO_GPFSEL4_FSEL45_7 ((u32) 0x00380000)
#define GPIO_GPFSEL4_FSEL45__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL45__OUTPUT ((u32) 0x00080000)
#define GPIO_GPFSEL4_FSEL45__ALT5 ((u32) 0x00100000)
#define GPIO_GPFSEL4_FSEL45__ALT4 ((u32) 0x00180000)
#define GPIO_GPFSEL4_FSEL45__ALT0 ((u32) 0x00200000)
#define GPIO_GPFSEL4_FSEL45__ALT1 ((u32) 0x00280000)
#define GPIO_GPFSEL4_FSEL45__ALT2 ((u32) 0x00300000)
#define GPIO_GPFSEL4_FSEL45__ALT3 ((u32) 0x00380000)
/* FSEL46 - Function Select 46 bits (RW) */
#define GPIO_GPFSEL4_FSEL46_OFS (18)
#define GPIO_GPFSEL4_FSEL46_MASK ((u32) 0x001C0000)
#define GPIO_GPFSEL4_FSEL460 ((u32) 0x00040000)
#define GPIO_GPFSEL4_FSEL461 ((u32) 0x00080000)
#define GPIO_GPFSEL4_FSEL462 ((u32) 0x00100000)
#define GPIO_GPFSEL4_FSEL46_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL46_1 ((u32) 0x00040000)
#define GPIO_GPFSEL4_FSEL46_2 ((u32) 0x00080000)
#define GPIO_GPFSEL4_FSEL46_3 ((u32) 0x000C0000)
#define GPIO_GPFSEL4_FSEL46_4 ((u32) 0x00100000)
#define GPIO_GPFSEL4_FSEL46_5 ((u32) 0x00140000)
#define GPIO_GPFSEL4_FSEL46_6 ((u32) 0x00180000)
#define GPIO_GPFSEL4_FSEL46_7 ((u32) 0x001C0000)
#define GPIO_GPFSEL4_FSEL46__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL46__OUTPUT ((u32) 0x00040000)
#define GPIO_GPFSEL4_FSEL46__ALT5 ((u32) 0x00080000)
#define GPIO_GPFSEL4_FSEL46__ALT4 ((u32) 0x000C0000)
#define GPIO_GPFSEL4_FSEL46__ALT0 ((u32) 0x00100000)
#define GPIO_GPFSEL4_FSEL46__ALT1 ((u32) 0x00140000)
#define GPIO_GPFSEL4_FSEL46__ALT2 ((u32) 0x00180000)
#define GPIO_GPFSEL4_FSEL46__ALT3 ((u32) 0x001C0000)
/* FSEL47 - Function Select 47 bits (RW) */
#define GPIO_GPFSEL4_FSEL47_OFS (21)
#define GPIO_GPFSEL4_FSEL47_MASK ((u32) 0x00E00000)
#define GPIO_GPFSEL4_FSEL470 ((u32) 0x00200000)
#define GPIO_GPFSEL4_FSEL471 ((u32) 0x00400000)
#define GPIO_GPFSEL4_FSEL472 ((u32) 0x00800000)
#define GPIO_GPFSEL4_FSEL47_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL47_1 ((u32) 0x00200000)
#define GPIO_GPFSEL4_FSEL47_2 ((u32) 0x00400000)
#define GPIO_GPFSEL4_FSEL47_3 ((u32) 0x00600000)
#define GPIO_GPFSEL4_FSEL47_4 ((u32) 0x00800000)
#define GPIO_GPFSEL4_FSEL47_5 ((u32) 0x00A00000)
#define GPIO_GPFSEL4_FSEL47_6 ((u32) 0x00C00000)
#define GPIO_GPFSEL4_FSEL47_7 ((u32) 0x00E00000)
#define GPIO_GPFSEL4_FSEL47__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL47__OUTPUT ((u32) 0x00200000)
#define GPIO_GPFSEL4_FSEL47__ALT5 ((u32) 0x00400000)
#define GPIO_GPFSEL4_FSEL47__ALT4 ((u32) 0x00600000)
#define GPIO_GPFSEL4_FSEL47__ALT0 ((u32) 0x00800000)
#define GPIO_GPFSEL4_FSEL47__ALT1 ((u32) 0x00A00000)
#define GPIO_GPFSEL4_FSEL47__ALT2 ((u32) 0x00C00000)
#define GPIO_GPFSEL4_FSEL47__ALT3 ((u32) 0x00E00000)
/* FSEL48 - Function Select 48 bits (RW) */
#define GPIO_GPFSEL4_FSEL48_OFS (24)
#define GPIO_GPFSEL4_FSEL48_MASK ((u32) 0x07000000)
#define GPIO_GPFSEL4_FSEL480 ((u32) 0x01000000)
#define GPIO_GPFSEL4_FSEL481 ((u32) 0x02000000)
#define GPIO_GPFSEL4_FSEL482 ((u32) 0x04000000)
#define GPIO_GPFSEL4_FSEL48_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL48_1 ((u32) 0x01000000)
#define GPIO_GPFSEL4_FSEL48_2 ((u32) 0x02000000)
#define GPIO_GPFSEL4_FSEL48_3 ((u32) 0x03000000)
#define GPIO_GPFSEL4_FSEL48_4 ((u32) 0x04000000)
#define GPIO_GPFSEL4_FSEL48_5 ((u32) 0x05000000)
#define GPIO_GPFSEL4_FSEL48_6 ((u32) 0x06000000)
#define GPIO_GPFSEL4_FSEL48_7 ((u32) 0x07000000)
#define GPIO_GPFSEL4_FSEL48__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL48__OUTPUT ((u32) 0x01000000)
#define GPIO_GPFSEL4_FSEL48__ALT5 ((u32) 0x02000000)
#define GPIO_GPFSEL4_FSEL48__ALT4 ((u32) 0x03000000)
#define GPIO_GPFSEL4_FSEL48__ALT0 ((u32) 0x04000000)
#define GPIO_GPFSEL4_FSEL48__ALT1 ((u32) 0x05000000)
#define GPIO_GPFSEL4_FSEL48__ALT2 ((u32) 0x06000000)
#define GPIO_GPFSEL4_FSEL48__ALT3 ((u32) 0x07000000)
/* FSEL49 - Function Select 49 bits (RW) */
#define GPIO_GPFSEL4_FSEL49_OFS (27)
#define GPIO_GPFSEL4_FSEL49_MASK ((u32) 0x38000000)
#define GPIO_GPFSEL4_FSEL490 ((u32) 0x08000000)
#define GPIO_GPFSEL4_FSEL491 ((u32) 0x10000000)
#define GPIO_GPFSEL4_FSEL492 ((u32) 0x20000000)
#define GPIO_GPFSEL4_FSEL49_0 ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL49_1 ((u32) 0x08000000)
#define GPIO_GPFSEL4_FSEL49_2 ((u32) 0x10000000)
#define GPIO_GPFSEL4_FSEL49_3 ((u32) 0x18000000)
#define GPIO_GPFSEL4_FSEL49_4 ((u32) 0x20000000)
#define GPIO_GPFSEL4_FSEL49_5 ((u32) 0x28000000)
#define GPIO_GPFSEL4_FSEL49_6 ((u32) 0x30000000)
#define GPIO_GPFSEL4_FSEL49_7 ((u32) 0x38000000)
#define GPIO_GPFSEL4_FSEL49__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL4_FSEL49__OUTPUT ((u32) 0x08000000)
#define GPIO_GPFSEL4_FSEL49__ALT5 ((u32) 0x10000000)
#define GPIO_GPFSEL4_FSEL49__ALT4 ((u32) 0x18000000)
#define GPIO_GPFSEL4_FSEL49__ALT0 ((u32) 0x20000000)
#define GPIO_GPFSEL4_FSEL49__ALT1 ((u32) 0x28000000)
#define GPIO_GPFSEL4_FSEL49__ALT2 ((u32) 0x30000000)
#define GPIO_GPFSEL4_FSEL49__ALT3 ((u32) 0x30000000)
/* GPIO Function Select 5 Register (RW) */
/* FSEL50 - Function Select 50 bits (RW) */
#define GPIO_GPFSEL5_FSEL50_OFS (0)
#define GPIO_GPFSEL5_FSEL50_MASK ((u32) 0x00000007)
#define GPIO_GPFSEL5_FSEL500 ((u32) 0x00000001)
#define GPIO_GPFSEL5_FSEL501 ((u32) 0x00000002)
#define GPIO_GPFSEL5_FSEL502 ((u32) 0x00000004)
#define GPIO_GPFSEL5_FSEL50_0 ((u32) 0x00000000)
#define GPIO_GPFSEL5_FSEL50_1 ((u32) 0x00000001)
#define GPIO_GPFSEL5_FSEL50_2 ((u32) 0x00000002)
#define GPIO_GPFSEL5_FSEL50_3 ((u32) 0x00000003)
#define GPIO_GPFSEL5_FSEL50_4 ((u32) 0x00000004)
#define GPIO_GPFSEL5_FSEL50_5 ((u32) 0x00000005)
#define GPIO_GPFSEL5_FSEL50_6 ((u32) 0x00000006)
#define GPIO_GPFSEL5_FSEL50_7 ((u32) 0x00000007)
#define GPIO_GPFSEL5_FSEL50__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL5_FSEL50__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL5_FSEL50__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL5_FSEL50__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL5_FSEL50__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL5_FSEL50__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL5_FSEL50__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL5_FSEL50__ALT3 ((u32) 0x00000007)
/* FSEL51 - Function Select 51 bits (RW) */
#define GPIO_GPFSEL5_FSEL51_OFS (3)
#define GPIO_GPFSEL5_FSEL51_MASK ((u32) 0x00000038)
#define GPIO_GPFSEL5_FSEL510 ((u32) 0x00000040)
#define GPIO_GPFSEL5_FSEL511 ((u32) 0x00000080)
#define GPIO_GPFSEL5_FSEL512 ((u32) 0x00000100)
#define GPIO_GPFSEL5_FSEL51_0 ((u32) 0x00000000)
#define GPIO_GPFSEL5_FSEL51_1 ((u32) 0x00000040)
#define GPIO_GPFSEL5_FSEL51_2 ((u32) 0x00000080)
#define GPIO_GPFSEL5_FSEL51_3 ((u32) 0x000000C0)
#define GPIO_GPFSEL5_FSEL51_4 ((u32) 0x00000100)
#define GPIO_GPFSEL5_FSEL51_5 ((u32) 0x00000140)
#define GPIO_GPFSEL5_FSEL51_6 ((u32) 0x00000180)
#define GPIO_GPFSEL5_FSEL51_7 ((u32) 0x000001C0)
#define GPIO_GPFSEL5_FSEL51__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL5_FSEL51__OUTPUT ((u32) 0x00000040)
#define GPIO_GPFSEL5_FSEL51__ALT5 ((u32) 0x00000080)
#define GPIO_GPFSEL5_FSEL51__ALT4 ((u32) 0x000000C0)
#define GPIO_GPFSEL5_FSEL51__ALT0 ((u32) 0x00000100)
#define GPIO_GPFSEL5_FSEL51__ALT1 ((u32) 0x00000140)
#define GPIO_GPFSEL5_FSEL51__ALT2 ((u32) 0x00000180)
#define GPIO_GPFSEL5_FSEL51__ALT3 ((u32) 0x000001C0)
/* FSEL52 - Function Select 52 bits (RW) */
#define GPIO_GPFSEL5_FSEL52_OFS (6)
#define GPIO_GPFSEL5_FSEL52_MASK ((u32) 0x000001C0)
#define GPIO_GPFSEL5_FSEL520 ((u32) 0x00000001)
#define GPIO_GPFSEL5_FSEL521 ((u32) 0x00000002)
#define GPIO_GPFSEL5_FSEL522 ((u32) 0x00000004)
#define GPIO_GPFSEL5_FSEL52_0 ((u32) 0x00000000)
#define GPIO_GPFSEL5_FSEL52_1 ((u32) 0x00000001)
#define GPIO_GPFSEL5_FSEL52_2 ((u32) 0x00000002)
#define GPIO_GPFSEL5_FSEL52_3 ((u32) 0x00000003)
#define GPIO_GPFSEL5_FSEL52_4 ((u32) 0x00000004)
#define GPIO_GPFSEL5_FSEL52_5 ((u32) 0x00000005)
#define GPIO_GPFSEL5_FSEL52_6 ((u32) 0x00000006)
#define GPIO_GPFSEL5_FSEL52_7 ((u32) 0x00000007)
#define GPIO_GPFSEL5_FSEL52__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL5_FSEL52__OUTPUT ((u32) 0x00000001)
#define GPIO_GPFSEL5_FSEL52__ALT5 ((u32) 0x00000002)
#define GPIO_GPFSEL5_FSEL52__ALT4 ((u32) 0x00000003)
#define GPIO_GPFSEL5_FSEL52__ALT0 ((u32) 0x00000004)
#define GPIO_GPFSEL5_FSEL52__ALT1 ((u32) 0x00000005)
#define GPIO_GPFSEL5_FSEL52__ALT2 ((u32) 0x00000006)
#define GPIO_GPFSEL5_FSEL52__ALT3 ((u32) 0x00000007)
/* FSEL53 - Function Select 53 bits (RW) */
#define GPIO_GPFSEL5_FSEL53_OFS (9)
#define GPIO_GPFSEL5_FSEL53_MASK ((u32) 0x00000E00)
#define GPIO_GPFSEL5_FSEL530 ((u32) 0x00000200)
#define GPIO_GPFSEL5_FSEL531 ((u32) 0x00000400)
#define GPIO_GPFSEL5_FSEL532 ((u32) 0x00000800)
#define GPIO_GPFSEL5_FSEL53_0 ((u32) 0x00000000)
#define GPIO_GPFSEL5_FSEL53_1 ((u32) 0x00000200)
#define GPIO_GPFSEL5_FSEL53_2 ((u32) 0x00000400)
#define GPIO_GPFSEL5_FSEL53_3 ((u32) 0x00000600)
#define GPIO_GPFSEL5_FSEL53_4 ((u32) 0x00000800)
#define GPIO_GPFSEL5_FSEL53_5 ((u32) 0x00000A00)
#define GPIO_GPFSEL5_FSEL53_6 ((u32) 0x00000C00)
#define GPIO_GPFSEL5_FSEL53_7 ((u32) 0x00000E00)
#define GPIO_GPFSEL5_FSEL53__INPUT ((u32) 0x00000000)
#define GPIO_GPFSEL5_FSEL53__OUTPUT ((u32) 0x0000200)
#define GPIO_GPFSEL5_FSEL53__ALT5 ((u32) 0x00000400)
#define GPIO_GPFSEL5_FSEL53__ALT4 ((u32) 0x00000600)
#define GPIO_GPFSEL5_FSEL53__ALT0 ((u32) 0x00000800)
#define GPIO_GPFSEL5_FSEL53__ALT1 ((u32) 0x00000A00)
#define GPIO_GPFSEL5_FSEL53__ALT2 ((u32) 0x00000C00)
#define GPIO_GPFSEL5_FSEL53__ALT3 ((u32) 0x00000E00)

/* GPIO Pin Output Set 0 Register (W) */
/* SETn (n=0..31) bits (W) */
#define GPIO_GPSET0_OFS (0)
#define GPIO_GPSET0_MASK ((u32) 0xFFFFFFFF)
/* Set GPIO pin 0 (W) */
#define GPIO_GPSET00 ((u32) 0x00000001)
#define GPIO_GPSET01 ((u32) 0x00000002)
#define GPIO_GPSET02 ((u32) 0x00000004)
#define GPIO_GPSET03 ((u32) 0x00000008)
#define GPIO_GPSET04 ((u32) 0x00000010)
#define GPIO_GPSET05 ((u32) 0x00000020)
#define GPIO_GPSET06 ((u32) 0x00000040)
#define GPIO_GPSET07 ((u32) 0x00000080)
#define GPIO_GPSET08 ((u32) 0x00000100)
#define GPIO_GPSET09 ((u32) 0x00000200)
#define GPIO_GPSET010 ((u32) 0x00000400)
#define GPIO_GPSET011 ((u32) 0x00000800)
#define GPIO_GPSET012 ((u32) 0x00001000)
#define GPIO_GPSET013 ((u32) 0x00002000)
#define GPIO_GPSET014 ((u32) 0x00004000)
#define GPIO_GPSET015 ((u32) 0x00008000)
#define GPIO_GPSET016 ((u32) 0x00010000)
#define GPIO_GPSET017 ((u32) 0x00020000)
#define GPIO_GPSET018 ((u32) 0x00040000)
#define GPIO_GPSET019 ((u32) 0x00080000)
#define GPIO_GPSET020 ((u32) 0x00100000)
#define GPIO_GPSET021 ((u32) 0x00200000)
#define GPIO_GPSET022 ((u32) 0x00400000)
#define GPIO_GPSET023 ((u32) 0x00800000)
#define GPIO_GPSET024 ((u32) 0x01000000)
#define GPIO_GPSET025 ((u32) 0x02000000)
#define GPIO_GPSET026 ((u32) 0x04000000)
#define GPIO_GPSET027 ((u32) 0x08000000)
#define GPIO_GPSET028 ((u32) 0x10000000)
#define GPIO_GPSET029 ((u32) 0x20000000)
#define GPIO_GPSET030 ((u32) 0x40000000)
#define GPIO_GPSET031 ((u32) 0x80000000)

/* GPIO Pin Output Set 1 Register (W) */
/* SETn (n=0..31) bits (W) */
#define GPIO_GPSET1_OFS (0)
#define GPIO_GPSET1_MASK ((u32) 0xFFFFFFFF)
/* Set GPIO pin 0 (W) */
#define GPIO_GPSET132 ((u32) 0x00000001)
#define GPIO_GPSET133 ((u32) 0x00000002)
#define GPIO_GPSET134 ((u32) 0x00000004)
#define GPIO_GPSET134 ((u32) 0x00000008)
#define GPIO_GPSET135 ((u32) 0x00000010)
#define GPIO_GPSET136 ((u32) 0x00000020)
#define GPIO_GPSET137 ((u32) 0x00000040)
#define GPIO_GPSET138 ((u32) 0x00000080)
#define GPIO_GPSET139 ((u32) 0x00000100)
#define GPIO_GPSET140 ((u32) 0x00000200)
#define GPIO_GPSET141 ((u32) 0x00000400)
#define GPIO_GPSET142 ((u32) 0x00000800)
#define GPIO_GPSET143 ((u32) 0x00001000)
#define GPIO_GPSET144 ((u32) 0x00002000)
#define GPIO_GPSET145 ((u32) 0x00004000)
#define GPIO_GPSET146 ((u32) 0x00008000)
#define GPIO_GPSET147 ((u32) 0x00010000)
#define GPIO_GPSET148 ((u32) 0x00020000)
#define GPIO_GPSET149 ((u32) 0x00040000)
#define GPIO_GPSET150 ((u32) 0x00080000)
#define GPIO_GPSET151 ((u32) 0x00100000)
#define GPIO_GPSET152 ((u32) 0x00200000)
#define GPIO_GPSET153 ((u32) 0x00400000)

/* GPIO Pin Output Clear 0 Register (W) */
/* CLRn (n=0..31) bits (W) */
#define GPIO_GPCLR0_OFS (0)
#define GPIO_GPCLR0_MASK ((u32) 0xFFFFFFFF)
/* Clear GPIO pin 0 (W) */
#define GPIO_GPCLR00 ((u32) 0x00000001)
#define GPIO_GPCLR01 ((u32) 0x00000002)
#define GPIO_GPCLR02 ((u32) 0x00000004)
#define GPIO_GPCLR03 ((u32) 0x00000008)
#define GPIO_GPCLR04 ((u32) 0x00000010)
#define GPIO_GPCLR05 ((u32) 0x00000020)
#define GPIO_GPCLR06 ((u32) 0x00000040)
#define GPIO_GPCLR07 ((u32) 0x00000080)
#define GPIO_GPCLR08 ((u32) 0x00000100)
#define GPIO_GPCLR09 ((u32) 0x00000200)
#define GPIO_GPCLR010 ((u32) 0x00000400)
#define GPIO_GPCLR011 ((u32) 0x00000800)
#define GPIO_GPCLR012 ((u32) 0x00001000)
#define GPIO_GPCLR013 ((u32) 0x00002000)
#define GPIO_GPCLR014 ((u32) 0x00004000)
#define GPIO_GPCLR015 ((u32) 0x00008000)
#define GPIO_GPCLR016 ((u32) 0x00010000)
#define GPIO_GPCLR017 ((u32) 0x00020000)
#define GPIO_GPCLR018 ((u32) 0x00040000)
#define GPIO_GPCLR019 ((u32) 0x00080000)
#define GPIO_GPCLR020 ((u32) 0x00100000)
#define GPIO_GPCLR021 ((u32) 0x00200000)
#define GPIO_GPCLR022 ((u32) 0x00400000)
#define GPIO_GPCLR023 ((u32) 0x00800000)
#define GPIO_GPCLR024 ((u32) 0x01000000)
#define GPIO_GPCLR025 ((u32) 0x02000000)
#define GPIO_GPCLR026 ((u32) 0x04000000)
#define GPIO_GPCLR027 ((u32) 0x08000000)
#define GPIO_GPCLR028 ((u32) 0x10000000)
#define GPIO_GPCLR029 ((u32) 0x20000000)
#define GPIO_GPCLR030 ((u32) 0x40000000)
#define GPIO_GPCLR031 ((u32) 0x80000000)

/* GPIO Pin Output Clear 1 Register (W) */
/* CLRn (n=0..31) bits (W) */
#define GPIO_GPCLR1_OFS (0)
#define GPIO_GPCLR1_MASK ((u32) 0xFFFFFFFF)
/* Clear GPIO pin 0 (W) */
#define GPIO_GPCLR132 ((u32) 0x00000001)
#define GPIO_GPCLR133 ((u32) 0x00000002)
#define GPIO_GPCLR134 ((u32) 0x00000004)
#define GPIO_GPCLR134 ((u32) 0x00000008)
#define GPIO_GPCLR135 ((u32) 0x00000010)
#define GPIO_GPCLR136 ((u32) 0x00000020)
#define GPIO_GPCLR137 ((u32) 0x00000040)
#define GPIO_GPCLR138 ((u32) 0x00000080)
#define GPIO_GPCLR139 ((u32) 0x00000100)
#define GPIO_GPCLR140 ((u32) 0x00000200)
#define GPIO_GPCLR141 ((u32) 0x00000400)
#define GPIO_GPCLR142 ((u32) 0x00000800)
#define GPIO_GPCLR143 ((u32) 0x00001000)
#define GPIO_GPCLR144 ((u32) 0x00002000)
#define GPIO_GPCLR145 ((u32) 0x00004000)
#define GPIO_GPCLR146 ((u32) 0x00008000)
#define GPIO_GPCLR147 ((u32) 0x00010000)
#define GPIO_GPCLR148 ((u32) 0x00020000)
#define GPIO_GPCLR149 ((u32) 0x00040000)
#define GPIO_GPCLR150 ((u32) 0x00080000)
#define GPIO_GPCLR151 ((u32) 0x00100000)
#define GPIO_GPCLR152 ((u32) 0x00200000)
#define GPIO_GPCLR153 ((u32) 0x00400000)

/* GPIO Pin Level 0 Register (R) */
/* Read value of the GPIO pin 0..31 */
#define GPIO_GPLEV0_OFS (0)
#define GPIO_GPLEV0_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPLEV00 ((u32) 0x00000001)
#define GPIO_GPLEV01 ((u32) 0x00000002)
#define GPIO_GPLEV02 ((u32) 0x00000004)
#define GPIO_GPLEV03 ((u32) 0x00000008)
#define GPIO_GPLEV04 ((u32) 0x00000010)
#define GPIO_GPLEV05 ((u32) 0x00000020)
#define GPIO_GPLEV06 ((u32) 0x00000040)
#define GPIO_GPLEV07 ((u32) 0x00000080)
#define GPIO_GPLEV08 ((u32) 0x00000100)
#define GPIO_GPLEV09 ((u32) 0x00000200)
#define GPIO_GPLEV010 ((u32) 0x00000400)
#define GPIO_GPLEV011 ((u32) 0x00000800)
#define GPIO_GPLEV012 ((u32) 0x00001000)
#define GPIO_GPLEV013 ((u32) 0x00002000)
#define GPIO_GPLEV014 ((u32) 0x00004000)
#define GPIO_GPLEV015 ((u32) 0x00008000)
#define GPIO_GPLEV016 ((u32) 0x00010000)
#define GPIO_GPLEV017 ((u32) 0x00020000)
#define GPIO_GPLEV018 ((u32) 0x00040000)
#define GPIO_GPLEV019 ((u32) 0x00080000)
#define GPIO_GPLEV020 ((u32) 0x00100000)
#define GPIO_GPLEV021 ((u32) 0x00200000)
#define GPIO_GPLEV022 ((u32) 0x00400000)
#define GPIO_GPLEV023 ((u32) 0x00800000)
#define GPIO_GPLEV024 ((u32) 0x01000000)
#define GPIO_GPLEV025 ((u32) 0x02000000)
#define GPIO_GPLEV026 ((u32) 0x04000000)
#define GPIO_GPLEV027 ((u32) 0x08000000)
#define GPIO_GPLEV028 ((u32) 0x10000000)
#define GPIO_GPLEV029 ((u32) 0x20000000)
#define GPIO_GPLEV030 ((u32) 0x40000000)
#define GPIO_GPLEV031 ((u32) 0x80000000)

/* GPIO Pin Level 1 Register (R) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPLEV1_OFS (0)
#define GPIO_GPLEV1_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPLEV132 ((u32) 0x00000001)
#define GPIO_GPLEV133 ((u32) 0x00000002)
#define GPIO_GPLEV134 ((u32) 0x00000004)
#define GPIO_GPLEV134 ((u32) 0x00000008)
#define GPIO_GPLEV135 ((u32) 0x00000010)
#define GPIO_GPLEV136 ((u32) 0x00000020)
#define GPIO_GPLEV137 ((u32) 0x00000040)
#define GPIO_GPLEV138 ((u32) 0x00000080)
#define GPIO_GPLEV139 ((u32) 0x00000100)
#define GPIO_GPLEV140 ((u32) 0x00000200)
#define GPIO_GPLEV141 ((u32) 0x00000400)
#define GPIO_GPLEV142 ((u32) 0x00000800)
#define GPIO_GPLEV143 ((u32) 0x00001000)
#define GPIO_GPLEV144 ((u32) 0x00002000)
#define GPIO_GPLEV145 ((u32) 0x00004000)
#define GPIO_GPLEV146 ((u32) 0x00008000)
#define GPIO_GPLEV147 ((u32) 0x00010000)
#define GPIO_GPLEV148 ((u32) 0x00020000)
#define GPIO_GPLEV149 ((u32) 0x00040000)
#define GPIO_GPLEV150 ((u32) 0x00080000)
#define GPIO_GPLEV151 ((u32) 0x00100000)
#define GPIO_GPLEV152 ((u32) 0x00200000)
#define GPIO_GPLEV153 ((u32) 0x00400000)

/* GPIO Pin Event Detect Status 0 Register (RW) */
#define GPIO_GPLEV0_OFS (0)
#define GPIO_GPLEV0_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPLEV00 ((u32) 0x00000001)
#define GPIO_GPLEV01 ((u32) 0x00000002)
#define GPIO_GPLEV02 ((u32) 0x00000004)
#define GPIO_GPLEV03 ((u32) 0x00000008)
#define GPIO_GPLEV04 ((u32) 0x00000010)
#define GPIO_GPLEV05 ((u32) 0x00000020)
#define GPIO_GPLEV06 ((u32) 0x00000040)
#define GPIO_GPLEV07 ((u32) 0x00000080)
#define GPIO_GPLEV08 ((u32) 0x00000100)
#define GPIO_GPLEV09 ((u32) 0x00000200)
#define GPIO_GPLEV010 ((u32) 0x00000400)
#define GPIO_GPLEV011 ((u32) 0x00000800)
#define GPIO_GPLEV012 ((u32) 0x00001000)
#define GPIO_GPLEV013 ((u32) 0x00002000)
#define GPIO_GPLEV014 ((u32) 0x00004000)
#define GPIO_GPLEV015 ((u32) 0x00008000)
#define GPIO_GPLEV016 ((u32) 0x00010000)
#define GPIO_GPLEV017 ((u32) 0x00020000)
#define GPIO_GPLEV018 ((u32) 0x00040000)
#define GPIO_GPLEV019 ((u32) 0x00080000)
#define GPIO_GPLEV020 ((u32) 0x00100000)
#define GPIO_GPLEV021 ((u32) 0x00200000)
#define GPIO_GPLEV022 ((u32) 0x00400000)
#define GPIO_GPLEV023 ((u32) 0x00800000)
#define GPIO_GPLEV024 ((u32) 0x01000000)
#define GPIO_GPLEV025 ((u32) 0x02000000)
#define GPIO_GPLEV026 ((u32) 0x04000000)
#define GPIO_GPLEV027 ((u32) 0x08000000)
#define GPIO_GPLEV028 ((u32) 0x10000000)
#define GPIO_GPLEV029 ((u32) 0x20000000)
#define GPIO_GPLEV030 ((u32) 0x40000000)
#define GPIO_GPLEV031 ((u32) 0x80000000)

/* GPIO Pin Event Detect Status 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPLEV1_OFS (0)
#define GPIO_GPLEV1_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPLEV132 ((u32) 0x00000001)
#define GPIO_GPLEV133 ((u32) 0x00000002)
#define GPIO_GPLEV134 ((u32) 0x00000004)
#define GPIO_GPLEV134 ((u32) 0x00000008)
#define GPIO_GPLEV135 ((u32) 0x00000010)
#define GPIO_GPLEV136 ((u32) 0x00000020)
#define GPIO_GPLEV137 ((u32) 0x00000040)
#define GPIO_GPLEV138 ((u32) 0x00000080)
#define GPIO_GPLEV139 ((u32) 0x00000100)
#define GPIO_GPLEV140 ((u32) 0x00000200)
#define GPIO_GPLEV141 ((u32) 0x00000400)
#define GPIO_GPLEV142 ((u32) 0x00000800)
#define GPIO_GPLEV143 ((u32) 0x00001000)
#define GPIO_GPLEV144 ((u32) 0x00002000)
#define GPIO_GPLEV145 ((u32) 0x00004000)
#define GPIO_GPLEV146 ((u32) 0x00008000)
#define GPIO_GPLEV147 ((u32) 0x00010000)
#define GPIO_GPLEV148 ((u32) 0x00020000)
#define GPIO_GPLEV149 ((u32) 0x00040000)
#define GPIO_GPLEV150 ((u32) 0x00080000)
#define GPIO_GPLEV151 ((u32) 0x00100000)
#define GPIO_GPLEV152 ((u32) 0x00200000)
#define GPIO_GPLEV153 ((u32) 0x00400000)

/* GPIO Pin Rising Edge Detect Enable 0 Register (RW) */
#define GPIO_GPREN0_OFS (0)
#define GPIO_GPREN0_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPREN00 ((u32) 0x00000001)
#define GPIO_GPREN01 ((u32) 0x00000002)
#define GPIO_GPREN02 ((u32) 0x00000004)
#define GPIO_GPREN03 ((u32) 0x00000008)
#define GPIO_GPREN04 ((u32) 0x00000010)
#define GPIO_GPREN05 ((u32) 0x00000020)
#define GPIO_GPREN06 ((u32) 0x00000040)
#define GPIO_GPREN07 ((u32) 0x00000080)
#define GPIO_GPREN08 ((u32) 0x00000100)
#define GPIO_GPREN09 ((u32) 0x00000200)
#define GPIO_GPREN010 ((u32) 0x00000400)
#define GPIO_GPREN011 ((u32) 0x00000800)
#define GPIO_GPREN012 ((u32) 0x00001000)
#define GPIO_GPREN013 ((u32) 0x00002000)
#define GPIO_GPREN014 ((u32) 0x00004000)
#define GPIO_GPREN015 ((u32) 0x00008000)
#define GPIO_GPREN016 ((u32) 0x00010000)
#define GPIO_GPREN017 ((u32) 0x00020000)
#define GPIO_GPREN018 ((u32) 0x00040000)
#define GPIO_GPREN019 ((u32) 0x00080000)
#define GPIO_GPREN020 ((u32) 0x00100000)
#define GPIO_GPREN021 ((u32) 0x00200000)
#define GPIO_GPREN022 ((u32) 0x00400000)
#define GPIO_GPREN023 ((u32) 0x00800000)
#define GPIO_GPREN024 ((u32) 0x01000000)
#define GPIO_GPREN025 ((u32) 0x02000000)
#define GPIO_GPREN026 ((u32) 0x04000000)
#define GPIO_GPREN027 ((u32) 0x08000000)
#define GPIO_GPREN028 ((u32) 0x10000000)
#define GPIO_GPREN029 ((u32) 0x20000000)
#define GPIO_GPREN030 ((u32) 0x40000000)
#define GPIO_GPREN031 ((u32) 0x80000000)

/* GPIO Pin Rising Edge Detect Enable 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPREN1_OFS (0)
#define GPIO_GPREN1_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPREN132 ((u32) 0x00000001)
#define GPIO_GPREN133 ((u32) 0x00000002)
#define GPIO_GPREN134 ((u32) 0x00000004)
#define GPIO_GPREN134 ((u32) 0x00000008)
#define GPIO_GPREN135 ((u32) 0x00000010)
#define GPIO_GPREN136 ((u32) 0x00000020)
#define GPIO_GPREN137 ((u32) 0x00000040)
#define GPIO_GPREN138 ((u32) 0x00000080)
#define GPIO_GPREN139 ((u32) 0x00000100)
#define GPIO_GPREN140 ((u32) 0x00000200)
#define GPIO_GPREN141 ((u32) 0x00000400)
#define GPIO_GPREN142 ((u32) 0x00000800)
#define GPIO_GPREN143 ((u32) 0x00001000)
#define GPIO_GPREN144 ((u32) 0x00002000)
#define GPIO_GPREN145 ((u32) 0x00004000)
#define GPIO_GPREN146 ((u32) 0x00008000)
#define GPIO_GPREN147 ((u32) 0x00010000)
#define GPIO_GPREN148 ((u32) 0x00020000)
#define GPIO_GPREN149 ((u32) 0x00040000)
#define GPIO_GPREN150 ((u32) 0x00080000)
#define GPIO_GPREN151 ((u32) 0x00100000)
#define GPIO_GPREN152 ((u32) 0x00200000)
#define GPIO_GPREN153 ((u32) 0x00400000)

/* GPIO Pin Falling Edge Detect Enable 0 Register (RW) */
#define GPIO_GPFEN0_OFS (0)
#define GPIO_GPFEN0_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPFEN00 ((u32) 0x00000001)
#define GPIO_GPFEN01 ((u32) 0x00000002)
#define GPIO_GPFEN02 ((u32) 0x00000004)
#define GPIO_GPFEN03 ((u32) 0x00000008)
#define GPIO_GPFEN04 ((u32) 0x00000010)
#define GPIO_GPFEN05 ((u32) 0x00000020)
#define GPIO_GPFEN06 ((u32) 0x00000040)
#define GPIO_GPFEN07 ((u32) 0x00000080)
#define GPIO_GPFEN08 ((u32) 0x00000100)
#define GPIO_GPFEN09 ((u32) 0x00000200)
#define GPIO_GPFEN010 ((u32) 0x00000400)
#define GPIO_GPFEN011 ((u32) 0x00000800)
#define GPIO_GPFEN012 ((u32) 0x00001000)
#define GPIO_GPFEN013 ((u32) 0x00002000)
#define GPIO_GPFEN014 ((u32) 0x00004000)
#define GPIO_GPFEN015 ((u32) 0x00008000)
#define GPIO_GPFEN016 ((u32) 0x00010000)
#define GPIO_GPFEN017 ((u32) 0x00020000)
#define GPIO_GPFEN018 ((u32) 0x00040000)
#define GPIO_GPFEN019 ((u32) 0x00080000)
#define GPIO_GPFEN020 ((u32) 0x00100000)
#define GPIO_GPFEN021 ((u32) 0x00200000)
#define GPIO_GPFEN022 ((u32) 0x00400000)
#define GPIO_GPFEN023 ((u32) 0x00800000)
#define GPIO_GPFEN024 ((u32) 0x01000000)
#define GPIO_GPFEN025 ((u32) 0x02000000)
#define GPIO_GPFEN026 ((u32) 0x04000000)
#define GPIO_GPFEN027 ((u32) 0x08000000)
#define GPIO_GPFEN028 ((u32) 0x10000000)
#define GPIO_GPFEN029 ((u32) 0x20000000)
#define GPIO_GPFEN030 ((u32) 0x40000000)
#define GPIO_GPFEN031 ((u32) 0x80000000)

/* GPIO Pin Falling Edge Detect Enable 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPFEN1_OFS (0)
#define GPIO_GPFEN1_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPFEN132 ((u32) 0x00000001)
#define GPIO_GPFEN133 ((u32) 0x00000002)
#define GPIO_GPFEN134 ((u32) 0x00000004)
#define GPIO_GPFEN134 ((u32) 0x00000008)
#define GPIO_GPFEN135 ((u32) 0x00000010)
#define GPIO_GPFEN136 ((u32) 0x00000020)
#define GPIO_GPFEN137 ((u32) 0x00000040)
#define GPIO_GPFEN138 ((u32) 0x00000080)
#define GPIO_GPFEN139 ((u32) 0x00000100)
#define GPIO_GPFEN140 ((u32) 0x00000200)
#define GPIO_GPFEN141 ((u32) 0x00000400)
#define GPIO_GPFEN142 ((u32) 0x00000800)
#define GPIO_GPFEN143 ((u32) 0x00001000)
#define GPIO_GPFEN144 ((u32) 0x00002000)
#define GPIO_GPFEN145 ((u32) 0x00004000)
#define GPIO_GPFEN146 ((u32) 0x00008000)
#define GPIO_GPFEN147 ((u32) 0x00010000)
#define GPIO_GPFEN148 ((u32) 0x00020000)
#define GPIO_GPFEN149 ((u32) 0x00040000)
#define GPIO_GPFEN150 ((u32) 0x00080000)
#define GPIO_GPFEN151 ((u32) 0x00100000)
#define GPIO_GPFEN152 ((u32) 0x00200000)
#define GPIO_GPFEN153 ((u32) 0x00400000)

/* GPIO Pin High Detect Enable 0 Register (RW) */
#define GPIO_GPHEN0_OFS (0)
#define GPIO_GPHEN0_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPHEN00 ((u32) 0x00000001)
#define GPIO_GPHEN01 ((u32) 0x00000002)
#define GPIO_GPHEN02 ((u32) 0x00000004)
#define GPIO_GPHEN03 ((u32) 0x00000008)
#define GPIO_GPHEN04 ((u32) 0x00000010)
#define GPIO_GPHEN05 ((u32) 0x00000020)
#define GPIO_GPHEN06 ((u32) 0x00000040)
#define GPIO_GPHEN07 ((u32) 0x00000080)
#define GPIO_GPHEN08 ((u32) 0x00000100)
#define GPIO_GPHEN09 ((u32) 0x00000200)
#define GPIO_GPHEN010 ((u32) 0x00000400)
#define GPIO_GPHEN011 ((u32) 0x00000800)
#define GPIO_GPHEN012 ((u32) 0x00001000)
#define GPIO_GPHEN013 ((u32) 0x00002000)
#define GPIO_GPHEN014 ((u32) 0x00004000)
#define GPIO_GPHEN015 ((u32) 0x00008000)
#define GPIO_GPHEN016 ((u32) 0x00010000)
#define GPIO_GPHEN017 ((u32) 0x00020000)
#define GPIO_GPHEN018 ((u32) 0x00040000)
#define GPIO_GPHEN019 ((u32) 0x00080000)
#define GPIO_GPHEN020 ((u32) 0x00100000)
#define GPIO_GPHEN021 ((u32) 0x00200000)
#define GPIO_GPHEN022 ((u32) 0x00400000)
#define GPIO_GPHEN023 ((u32) 0x00800000)
#define GPIO_GPHEN024 ((u32) 0x01000000)
#define GPIO_GPHEN025 ((u32) 0x02000000)
#define GPIO_GPHEN026 ((u32) 0x04000000)
#define GPIO_GPHEN027 ((u32) 0x08000000)
#define GPIO_GPHEN028 ((u32) 0x10000000)
#define GPIO_GPHEN029 ((u32) 0x20000000)
#define GPIO_GPHEN030 ((u32) 0x40000000)
#define GPIO_GPHEN031 ((u32) 0x80000000)

/* GPIO Pin High Detect Enable 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPHEN1_OFS (0)
#define GPIO_GPHEN1_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPHEN132 ((u32) 0x00000001)
#define GPIO_GPHEN133 ((u32) 0x00000002)
#define GPIO_GPHEN134 ((u32) 0x00000004)
#define GPIO_GPHEN134 ((u32) 0x00000008)
#define GPIO_GPHEN135 ((u32) 0x00000010)
#define GPIO_GPHEN136 ((u32) 0x00000020)
#define GPIO_GPHEN137 ((u32) 0x00000040)
#define GPIO_GPHEN138 ((u32) 0x00000080)
#define GPIO_GPHEN139 ((u32) 0x00000100)
#define GPIO_GPHEN140 ((u32) 0x00000200)
#define GPIO_GPHEN141 ((u32) 0x00000400)
#define GPIO_GPHEN142 ((u32) 0x00000800)
#define GPIO_GPHEN143 ((u32) 0x00001000)
#define GPIO_GPHEN144 ((u32) 0x00002000)
#define GPIO_GPHEN145 ((u32) 0x00004000)
#define GPIO_GPHEN146 ((u32) 0x00008000)
#define GPIO_GPHEN147 ((u32) 0x00010000)
#define GPIO_GPHEN148 ((u32) 0x00020000)
#define GPIO_GPHEN149 ((u32) 0x00040000)
#define GPIO_GPHEN150 ((u32) 0x00080000)
#define GPIO_GPHEN151 ((u32) 0x00100000)
#define GPIO_GPHEN152 ((u32) 0x00200000)
#define GPIO_GPHEN153 ((u32) 0x00400000)

/* GPIO Pin Low Detect Enable 0 Register (RW) */
#define GPIO_GPLEN0_OFS (0)
#define GPIO_GPLEN0_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPLEN00 ((u32) 0x00000001)
#define GPIO_GPLEN01 ((u32) 0x00000002)
#define GPIO_GPLEN02 ((u32) 0x00000004)
#define GPIO_GPLEN03 ((u32) 0x00000008)
#define GPIO_GPLEN04 ((u32) 0x00000010)
#define GPIO_GPLEN05 ((u32) 0x00000020)
#define GPIO_GPLEN06 ((u32) 0x00000040)
#define GPIO_GPLEN07 ((u32) 0x00000080)
#define GPIO_GPLEN08 ((u32) 0x00000100)
#define GPIO_GPLEN09 ((u32) 0x00000200)
#define GPIO_GPLEN010 ((u32) 0x00000400)
#define GPIO_GPLEN011 ((u32) 0x00000800)
#define GPIO_GPLEN012 ((u32) 0x00001000)
#define GPIO_GPLEN013 ((u32) 0x00002000)
#define GPIO_GPLEN014 ((u32) 0x00004000)
#define GPIO_GPLEN015 ((u32) 0x00008000)
#define GPIO_GPLEN016 ((u32) 0x00010000)
#define GPIO_GPLEN017 ((u32) 0x00020000)
#define GPIO_GPLEN018 ((u32) 0x00040000)
#define GPIO_GPLEN019 ((u32) 0x00080000)
#define GPIO_GPLEN020 ((u32) 0x00100000)
#define GPIO_GPLEN021 ((u32) 0x00200000)
#define GPIO_GPLEN022 ((u32) 0x00400000)
#define GPIO_GPLEN023 ((u32) 0x00800000)
#define GPIO_GPLEN024 ((u32) 0x01000000)
#define GPIO_GPLEN025 ((u32) 0x02000000)
#define GPIO_GPLEN026 ((u32) 0x04000000)
#define GPIO_GPLEN027 ((u32) 0x08000000)
#define GPIO_GPLEN028 ((u32) 0x10000000)
#define GPIO_GPLEN029 ((u32) 0x20000000)
#define GPIO_GPLEN030 ((u32) 0x40000000)
#define GPIO_GPLEN031 ((u32) 0x80000000)

/* GPIO Pin Low Detect Enable 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPLEN1_OFS (0)
#define GPIO_GPLEN1_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPLEN132 ((u32) 0x00000001)
#define GPIO_GPLEN133 ((u32) 0x00000002)
#define GPIO_GPLEN134 ((u32) 0x00000004)
#define GPIO_GPLEN134 ((u32) 0x00000008)
#define GPIO_GPLEN135 ((u32) 0x00000010)
#define GPIO_GPLEN136 ((u32) 0x00000020)
#define GPIO_GPLEN137 ((u32) 0x00000040)
#define GPIO_GPLEN138 ((u32) 0x00000080)
#define GPIO_GPLEN139 ((u32) 0x00000100)
#define GPIO_GPLEN140 ((u32) 0x00000200)
#define GPIO_GPLEN141 ((u32) 0x00000400)
#define GPIO_GPLEN142 ((u32) 0x00000800)
#define GPIO_GPLEN143 ((u32) 0x00001000)
#define GPIO_GPLEN144 ((u32) 0x00002000)
#define GPIO_GPLEN145 ((u32) 0x00004000)
#define GPIO_GPLEN146 ((u32) 0x00008000)
#define GPIO_GPLEN147 ((u32) 0x00010000)
#define GPIO_GPLEN148 ((u32) 0x00020000)
#define GPIO_GPLEN149 ((u32) 0x00040000)
#define GPIO_GPLEN150 ((u32) 0x00080000)
#define GPIO_GPLEN151 ((u32) 0x00100000)
#define GPIO_GPLEN152 ((u32) 0x00200000)
#define GPIO_GPLEN153 ((u32) 0x00400000)

/* GPIO Pin Async. Rising Edge Detect 0 Register (RW) */
#define GPIO_GPAREN0_OFS (0)
#define GPIO_GPAREN0_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPAREN00 ((u32) 0x00000001)
#define GPIO_GPAREN01 ((u32) 0x00000002)
#define GPIO_GPAREN02 ((u32) 0x00000004)
#define GPIO_GPAREN03 ((u32) 0x00000008)
#define GPIO_GPAREN04 ((u32) 0x00000010)
#define GPIO_GPAREN05 ((u32) 0x00000020)
#define GPIO_GPAREN06 ((u32) 0x00000040)
#define GPIO_GPAREN07 ((u32) 0x00000080)
#define GPIO_GPAREN08 ((u32) 0x00000100)
#define GPIO_GPAREN09 ((u32) 0x00000200)
#define GPIO_GPAREN010 ((u32) 0x00000400)
#define GPIO_GPAREN011 ((u32) 0x00000800)
#define GPIO_GPAREN012 ((u32) 0x00001000)
#define GPIO_GPAREN013 ((u32) 0x00002000)
#define GPIO_GPAREN014 ((u32) 0x00004000)
#define GPIO_GPAREN015 ((u32) 0x00008000)
#define GPIO_GPAREN016 ((u32) 0x00010000)
#define GPIO_GPAREN017 ((u32) 0x00020000)
#define GPIO_GPAREN018 ((u32) 0x00040000)
#define GPIO_GPAREN019 ((u32) 0x00080000)
#define GPIO_GPAREN020 ((u32) 0x00100000)
#define GPIO_GPAREN021 ((u32) 0x00200000)
#define GPIO_GPAREN022 ((u32) 0x00400000)
#define GPIO_GPAREN023 ((u32) 0x00800000)
#define GPIO_GPAREN024 ((u32) 0x01000000)
#define GPIO_GPAREN025 ((u32) 0x02000000)
#define GPIO_GPAREN026 ((u32) 0x04000000)
#define GPIO_GPAREN027 ((u32) 0x08000000)
#define GPIO_GPAREN028 ((u32) 0x10000000)
#define GPIO_GPAREN029 ((u32) 0x20000000)
#define GPIO_GPAREN030 ((u32) 0x40000000)
#define GPIO_GPAREN031 ((u32) 0x80000000)

/* GPIO Pin Async. Rising Edge Detect 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPAREN1_OFS (0)
#define GPIO_GPAREN1_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 32 (R) */
#define GPIO_GPAREN132 ((u32) 0x00000001)
#define GPIO_GPAREN133 ((u32) 0x00000002)
#define GPIO_GPAREN134 ((u32) 0x00000004)
#define GPIO_GPAREN134 ((u32) 0x00000008)
#define GPIO_GPAREN135 ((u32) 0x00000010)
#define GPIO_GPAREN136 ((u32) 0x00000020)
#define GPIO_GPAREN137 ((u32) 0x00000040)
#define GPIO_GPAREN138 ((u32) 0x00000080)
#define GPIO_GPAREN139 ((u32) 0x00000100)
#define GPIO_GPAREN140 ((u32) 0x00000200)
#define GPIO_GPAREN141 ((u32) 0x00000400)
#define GPIO_GPAREN142 ((u32) 0x00000800)
#define GPIO_GPAREN143 ((u32) 0x00001000)
#define GPIO_GPAREN144 ((u32) 0x00002000)
#define GPIO_GPAREN145 ((u32) 0x00004000)
#define GPIO_GPAREN146 ((u32) 0x00008000)
#define GPIO_GPAREN147 ((u32) 0x00010000)
#define GPIO_GPAREN148 ((u32) 0x00020000)
#define GPIO_GPAREN149 ((u32) 0x00040000)
#define GPIO_GPAREN150 ((u32) 0x00080000)
#define GPIO_GPAREN151 ((u32) 0x00100000)
#define GPIO_GPAREN152 ((u32) 0x00200000)
#define GPIO_GPAREN153 ((u32) 0x00400000)

/* GPIO Pin Async. Falling Edge Detect 0 Register (RW) */
#define GPIO_GPAFEN0_OFS (0)
#define GPIO_GPAFEN0_MASK ((u32) 0xFFFFFFFF)
/* Read GPIO pin 0 (R) */
#define GPIO_GPAFEN00 ((u32) 0x00000001)
#define GPIO_GPAFEN01 ((u32) 0x00000002)
#define GPIO_GPAFEN02 ((u32) 0x00000004)
#define GPIO_GPAFEN03 ((u32) 0x00000008)
#define GPIO_GPAFEN04 ((u32) 0x00000010)
#define GPIO_GPAFEN05 ((u32) 0x00000020)
#define GPIO_GPAFEN06 ((u32) 0x00000040)
#define GPIO_GPAFEN07 ((u32) 0x00000080)
#define GPIO_GPAFEN08 ((u32) 0x00000100)
#define GPIO_GPAFEN09 ((u32) 0x00000200)
#define GPIO_GPAFEN010 ((u32) 0x00000400)
#define GPIO_GPAFEN011 ((u32) 0x00000800)
#define GPIO_GPAFEN012 ((u32) 0x00001000)
#define GPIO_GPAFEN013 ((u32) 0x00002000)
#define GPIO_GPAFEN014 ((u32) 0x00004000)
#define GPIO_GPAFEN015 ((u32) 0x00008000)
#define GPIO_GPAFEN016 ((u32) 0x00010000)
#define GPIO_GPAFEN017 ((u32) 0x00020000)
#define GPIO_GPAFEN018 ((u32) 0x00040000)
#define GPIO_GPAFEN019 ((u32) 0x00080000)
#define GPIO_GPAFEN020 ((u32) 0x00100000)
#define GPIO_GPAFEN021 ((u32) 0x00200000)
#define GPIO_GPAFEN022 ((u32) 0x00400000)
#define GPIO_GPAFEN023 ((u32) 0x00800000)
#define GPIO_GPAFEN024 ((u32) 0x01000000)
#define GPIO_GPAFEN025 ((u32) 0x02000000)
#define GPIO_GPAFEN026 ((u32) 0x04000000)
#define GPIO_GPAFEN027 ((u32) 0x08000000)
#define GPIO_GPAFEN028 ((u32) 0x10000000)
#define GPIO_GPAFEN029 ((u32) 0x20000000)
#define GPIO_GPAFEN030 ((u32) 0x40000000)
#define GPIO_GPAFEN031 ((u32) 0x80000000)

/* GPIO Pin Async. Falling Edge Detect 1 Register (RW) */
/* Read value of the GPIO pin 32..53 */
#define GPIO_GPAFEN1_OFS (0)
#define GPIO_GPAFEN1_MASK ((u32) 0xFFFFFFFF)
/* Read the value of GPIO pin 32 (R) */
#define GPIO_GPAFEN132 ((u32) 0x00000001)
#define GPIO_GPAFEN133 ((u32) 0x00000002)
#define GPIO_GPAFEN134 ((u32) 0x00000004)
#define GPIO_GPAFEN134 ((u32) 0x00000008)
#define GPIO_GPAFEN135 ((u32) 0x00000010)
#define GPIO_GPAFEN136 ((u32) 0x00000020)
#define GPIO_GPAFEN137 ((u32) 0x00000040)
#define GPIO_GPAFEN138 ((u32) 0x00000080)
#define GPIO_GPAFEN139 ((u32) 0x00000100)
#define GPIO_GPAFEN140 ((u32) 0x00000200)
#define GPIO_GPAFEN141 ((u32) 0x00000400)
#define GPIO_GPAFEN142 ((u32) 0x00000800)
#define GPIO_GPAFEN143 ((u32) 0x00001000)
#define GPIO_GPAFEN144 ((u32) 0x00002000)
#define GPIO_GPAFEN145 ((u32) 0x00004000)
#define GPIO_GPAFEN146 ((u32) 0x00008000)
#define GPIO_GPAFEN147 ((u32) 0x00010000)
#define GPIO_GPAFEN148 ((u32) 0x00020000)
#define GPIO_GPAFEN149 ((u32) 0x00040000)
#define GPIO_GPAFEN150 ((u32) 0x00080000)
#define GPIO_GPAFEN151 ((u32) 0x00100000)
#define GPIO_GPAFEN152 ((u32) 0x00200000)
#define GPIO_GPAFEN153 ((u32) 0x00400000)

/* GPIO Pin Pull-up/down Enable Register (W) */
#define GPIO_GPPUD_PUD_OFS (0)
#define GPIO_GPPUD_PUD_MASK ((u32) 0x00000003)
#define GPIO_GPPUD_PUD0 ((u32) 0x00000001)
#define GPIO_GPPUD_PUD1 ((u32) 0x00000002)
#define GPIO_GPPUD_PUD_0 ((u32) 0x00000000)
#define GPIO_GPPUD_PUD_1 ((u32) 0x00000001)
#define GPIO_GPPUD_PUD_2 ((u32) 0x00000002)
#define GPIO_GPPUD_PUD_3 ((u32) 0x00000003)
#define GPIO_GPPUD_PUD__OFF ((u32) 0x00000000)
#define GPIO_GPPUD_PUD__PULLDOWN_ENABLE ((u32) 0x00000001)
#define GPIO_GPPUD_PUD__PULLUP_ENABLE ((u32) 0x00000002)
#define GPIO_GPPUD_PUD__RESERVED ((u32) 0x00000003)

/* GPIO Pin Pull-up/down Enable Clock 0 Register (W) */
#define GPIO_GPPUDCLK0_OFS (0)
#define GPIO_GPPUDCLK0_MASK ((u32) 0xFFFFFFFF)
/* Clock the control signal into GPIO pad pull-up/down logic 0 (W) */
#define GPIO_GPPUDCLK00 ((u32) 0x00000001)
#define GPIO_GPPUDCLK01 ((u32) 0x00000002)
#define GPIO_GPPUDCLK02 ((u32) 0x00000004)
#define GPIO_GPPUDCLK03 ((u32) 0x00000008)
#define GPIO_GPPUDCLK04 ((u32) 0x00000010)
#define GPIO_GPPUDCLK05 ((u32) 0x00000020)
#define GPIO_GPPUDCLK06 ((u32) 0x00000040)
#define GPIO_GPPUDCLK07 ((u32) 0x00000080)
#define GPIO_GPPUDCLK08 ((u32) 0x00000100)
#define GPIO_GPPUDCLK09 ((u32) 0x00000200)
#define GPIO_GPPUDCLK010 ((u32) 0x00000400)
#define GPIO_GPPUDCLK011 ((u32) 0x00000800)
#define GPIO_GPPUDCLK012 ((u32) 0x00001000)
#define GPIO_GPPUDCLK013 ((u32) 0x00002000)
#define GPIO_GPPUDCLK014 ((u32) 0x00004000)
#define GPIO_GPPUDCLK015 ((u32) 0x00008000)
#define GPIO_GPPUDCLK016 ((u32) 0x00010000)
#define GPIO_GPPUDCLK017 ((u32) 0x00020000)
#define GPIO_GPPUDCLK018 ((u32) 0x00040000)
#define GPIO_GPPUDCLK019 ((u32) 0x00080000)
#define GPIO_GPPUDCLK020 ((u32) 0x00100000)
#define GPIO_GPPUDCLK021 ((u32) 0x00200000)
#define GPIO_GPPUDCLK022 ((u32) 0x00400000)
#define GPIO_GPPUDCLK023 ((u32) 0x00800000)
#define GPIO_GPPUDCLK024 ((u32) 0x01000000)
#define GPIO_GPPUDCLK025 ((u32) 0x02000000)
#define GPIO_GPPUDCLK026 ((u32) 0x04000000)
#define GPIO_GPPUDCLK027 ((u32) 0x08000000)
#define GPIO_GPPUDCLK028 ((u32) 0x10000000)
#define GPIO_GPPUDCLK029 ((u32) 0x20000000)
#define GPIO_GPPUDCLK030 ((u32) 0x40000000)
#define GPIO_GPPUDCLK031 ((u32) 0x80000000)

/* GPIO Pin Pull-up/down Enable Clock 1 Register (W) */
#define GPIO_GPPUDCLK1_OFS (0)
#define GPIO_GPPUDCLK1_MASK ((u32) 0xFFFFFFFF)
/* Clock the control signal into GPIO pad pull-up/down logic 0 (W) */
#define GPIO_GPPUDCLK132 ((u32) 0x00000001)
#define GPIO_GPPUDCLK133 ((u32) 0x00000002)
#define GPIO_GPPUDCLK134 ((u32) 0x00000004)
#define GPIO_GPPUDCLK134 ((u32) 0x00000008)
#define GPIO_GPPUDCLK135 ((u32) 0x00000010)
#define GPIO_GPPUDCLK136 ((u32) 0x00000020)
#define GPIO_GPPUDCLK137 ((u32) 0x00000040)
#define GPIO_GPPUDCLK138 ((u32) 0x00000080)
#define GPIO_GPPUDCLK139 ((u32) 0x00000100)
#define GPIO_GPPUDCLK140 ((u32) 0x00000200)
#define GPIO_GPPUDCLK141 ((u32) 0x00000400)
#define GPIO_GPPUDCLK142 ((u32) 0x00000800)
#define GPIO_GPPUDCLK143 ((u32) 0x00001000)
#define GPIO_GPPUDCLK144 ((u32) 0x00002000)
#define GPIO_GPPUDCLK145 ((u32) 0x00004000)
#define GPIO_GPPUDCLK146 ((u32) 0x00008000)
#define GPIO_GPPUDCLK147 ((u32) 0x00010000)
#define GPIO_GPPUDCLK148 ((u32) 0x00020000)
#define GPIO_GPPUDCLK149 ((u32) 0x00040000)
#define GPIO_GPPUDCLK150 ((u32) 0x00080000)
#define GPIO_GPPUDCLK151 ((u32) 0x00100000)
#define GPIO_GPPUDCLK152 ((u32) 0x00200000)
#define GPIO_GPPUDCLK153 ((u32) 0x00400000)

/* GPIO Pads control bits */
/* GPIO Pads 0-27 Control register */
/* Drive strength bits (RW) */
#define GPIO_PADS0_27_DRIVE_OFS (0)
#define GPIO_PADS0_27_DRIVE_MASK ((u32) 0x00000007)
#define GPIO_PADS0_27_DRIVE0 ((u32) 0x00000001)
#define GPIO_PADS0_27_DRIVE1 ((u32) 0x00000002)
#define GPIO_PADS0_27_DRIVE2 ((u32) 0x00000004)
#define GPIO_PADS0_27_DRIVE_0 ((u32) 0x00000000)
#define GPIO_PADS0_27_DRIVE_1 ((u32) 0x00000001)
#define GPIO_PADS0_27_DRIVE_2 ((u32) 0x00000002)
#define GPIO_PADS0_27_DRIVE_3 ((u32) 0x00000003)
#define GPIO_PADS0_27_DRIVE_4 ((u32) 0x00000004)
#define GPIO_PADS0_27_DRIVE_5 ((u32) 0x00000005)
#define GPIO_PADS0_27_DRIVE_6 ((u32) 0x00000006)
#define GPIO_PADS0_27_DRIVE_7 ((u32) 0x00000007)
#define GPIO_PADS0_27_DRIVE__2MILLIAMPS ((u32) 0x00000000)
#define GPIO_PADS0_27_DRIVE__4MILLIAMPS ((u32) 0x00000001)
#define GPIO_PADS0_27_DRIVE__6MILLIAMPS ((u32) 0x00000002)
#define GPIO_PADS0_27_DRIVE__8MILLIAMPS ((u32) 0x00000003)
#define GPIO_PADS0_27_DRIVE__10MILLIAMPS ((u32) 0x00000004)
#define GPIO_PADS0_27_DRIVE__12MILLIAMPS ((u32) 0x00000005)
#define GPIO_PADS0_27_DRIVE__14MILLIAMPS ((u32) 0x00000006)
#define GPIO_PADS0_27_DRIVE__16MILLIAMPS ((u32) 0x00000007)
/* Enable input hysteresis bit (RW) */
#define GPIO_PADS0_27_HYSTERESIS_OFS (3)
#define GPIO_PADS0_27_HYSTERESIS ((u32) 0x00000008)
/* Slew Rate limit bit (RW) */
#define GPIO_PADS0_27_SLEW_RATE_OFS (4)
#define GPIO_PADS0_27_SLEW_RATE ((u32) 0x00000010)
/* Password (0x5A) bits (RW) */
#define GPIO_PADS0_27_PASSWD_OFS (24)
#define GPIO_PADS0_27_PASSWD_MASK ((u32) 0xFF000000)
/* GPIO Pads 28-45 Control register */
/* Drive strength bits (RW) */
#define GPIO_PADS28_45_DRIVE_OFS (0)
#define GPIO_PADS28_45_DRIVE_MASK ((u32) 0x00000007)
#define GPIO_PADS28_45_DRIVE0 ((u32) 0x00000001)
#define GPIO_PADS28_45_DRIVE1 ((u32) 0x00000002)
#define GPIO_PADS28_45_DRIVE2 ((u32) 0x00000004)
#define GPIO_PADS28_45_DRIVE_0 ((u32) 0x00000000)
#define GPIO_PADS28_45_DRIVE_1 ((u32) 0x00000001)
#define GPIO_PADS28_45_DRIVE_2 ((u32) 0x00000002)
#define GPIO_PADS28_45_DRIVE_3 ((u32) 0x00000003)
#define GPIO_PADS28_45_DRIVE_4 ((u32) 0x00000004)
#define GPIO_PADS28_45_DRIVE_5 ((u32) 0x00000005)
#define GPIO_PADS28_45_DRIVE_6 ((u32) 0x00000006)
#define GPIO_PADS28_45_DRIVE_7 ((u32) 0x00000007)
#define GPIO_PADS28_45_DRIVE__2MILLIAMPS ((u32) 0x00000000)
#define GPIO_PADS28_45_DRIVE__4MILLIAMPS ((u32) 0x00000001)
#define GPIO_PADS28_45_DRIVE__6MILLIAMPS ((u32) 0x00000002)
#define GPIO_PADS28_45_DRIVE__8MILLIAMPS ((u32) 0x00000003)
#define GPIO_PADS28_45_DRIVE__10MILLIAMPS ((u32) 0x00000004)
#define GPIO_PADS28_45_DRIVE__12MILLIAMPS ((u32) 0x00000005)
#define GPIO_PADS28_45_DRIVE__14MILLIAMPS ((u32) 0x00000006)
#define GPIO_PADS28_45_DRIVE__16MILLIAMPS ((u32) 0x00000007)
/* Enable input hysteresis bit (RW) */
#define GPIO_PADS28_45_HYSTERESIS_OFS (3)
#define GPIO_PADS28_45_HYSTERESIS ((u32) 0x00000008)
/* Slew Rate limit bit (RW) */
#define GPIO_PADS28_45_SLEW_RATE_OFS (4)
#define GPIO_PADS28_45_SLEW_RATE ((u32) 0x00000010)
/* Password (0x5A) bits (RW) */
#define GPIO_PADS28_45_PASSWD_OFS (24)
#define GPIO_PADS28_45_PASSWD_MASK ((u32) 0xFF000000)

/* GPIO Pads 46-53 Control register */
/* Drive strength bits (RW) */
#define GPIO_PADS46_53_DRIVE_OFS (0)
#define GPIO_PADS46_53_DRIVE_MASK ((u32) 0x00000007)
#define GPIO_PADS46_53_DRIVE0 ((u32) 0x00000001)
#define GPIO_PADS46_53_DRIVE1 ((u32) 0x00000002)
#define GPIO_PADS46_53_DRIVE2 ((u32) 0x00000004)
#define GPIO_PADS46_53_DRIVE_0 ((u32) 0x00000000)
#define GPIO_PADS46_53_DRIVE_1 ((u32) 0x00000001)
#define GPIO_PADS46_53_DRIVE_2 ((u32) 0x00000002)
#define GPIO_PADS46_53_DRIVE_3 ((u32) 0x00000003)
#define GPIO_PADS46_53_DRIVE_4 ((u32) 0x00000004)
#define GPIO_PADS46_53_DRIVE_5 ((u32) 0x00000005)
#define GPIO_PADS46_53_DRIVE_6 ((u32) 0x00000006)
#define GPIO_PADS46_53_DRIVE_7 ((u32) 0x00000007)
#define GPIO_PADS46_53_DRIVE__2MILLIAMPS ((u32) 0x00000000)
#define GPIO_PADS46_53_DRIVE__4MILLIAMPS ((u32) 0x00000001)
#define GPIO_PADS46_53_DRIVE__6MILLIAMPS ((u32) 0x00000002)
#define GPIO_PADS46_53_DRIVE__8MILLIAMPS ((u32) 0x00000003)
#define GPIO_PADS46_53_DRIVE__10MILLIAMPS ((u32) 0x00000004)
#define GPIO_PADS46_53_DRIVE__12MILLIAMPS ((u32) 0x00000005)
#define GPIO_PADS46_53_DRIVE__14MILLIAMPS ((u32) 0x00000006)
#define GPIO_PADS46_53_DRIVE__16MILLIAMPS ((u32) 0x00000007)
/* Enable input hysteresis bit (RW) */
#define GPIO_PADS46_53_HYSTERESIS_OFS (3)
#define GPIO_PADS46_53_HYSTERESIS ((u32) 0x00000008)
/* Slew Rate limit bit (RW) */
#define GPIO_PADS46_53_SLEW_RATE_OFS (4)
#define GPIO_PADS46_53_SLEW_RATE ((u32) 0x00000010)
/* Password (0x5A) bits (RW) */
#define GPIO_PADS46_53_PASSWD_OFS (24)
#define GPIO_PADS46_53_PASSWD_MASK ((u32) 0xFF000000)

/* CM Bits */
/* Clock Manager General Purpose Clocks Control Register 0 (RW) */
/* SRC - Clock source (RW) */
#define CM_GPCTL0_SRC_OFS (0)
#define CM_GPCTL0_SRC_MASK ((u32) 0x0000000F)
#define CM_GPCTL0_SRC0 ((u32) 0x00000001)
#define CM_GPCTL0_SRC1 ((u32) 0x00000002)
#define CM_GPCTL0_SRC2 ((u32) 0x00000004)
#define CM_GPCTL0_SRC3 ((u32) 0x00000008)
#define CM_GPCTL0_SRC_0 ((u32) 0x00000000)
#define CM_GPCTL0_SRC_1 ((u32) 0x00000001)
#define CM_GPCTL0_SRC_2 ((u32) 0x00000002)
#define CM_GPCTL0_SRC_3 ((u32) 0x00000003)
#define CM_GPCTL0_SRC_4 ((u32) 0x00000004)
#define CM_GPCTL0_SRC_5 ((u32) 0x00000005)
#define CM_GPCTL0_SRC_6 ((u32) 0x00000006)
#define CM_GPCTL0_SRC_7 ((u32) 0x00000007)
#define CM_GPCTL0_SRC_8 ((u32) 0x00000008)
#define CM_GPCTL0_SRC_9 ((u32) 0x00000009)
#define CM_GPCTL0_SRC_10 ((u32) 0x0000000A)
#define CM_GPCTL0_SRC_11 ((u32) 0x0000000B)
#define CM_GPCTL0_SRC_12 ((u32) 0x0000000C)
#define CM_GPCTL0_SRC_13 ((u32) 0x0000000D)
#define CM_GPCTL0_SRC_14 ((u32) 0x0000000E)
#define CM_GPCTL0_SRC_15 ((u32) 0x0000000F)
#define CM_GPCTL0_SRC__GND ((u32) 0x00000000)
#define CM_GPCTL0_SRC__OSCILLATOR ((u32) 0x00000001)
#define CM_GPCTL0_SRC__TESTDEBUG0 ((u32) 0x00000002)
#define CM_GPCTL0_SRC__TESTDEBUG1 ((u32) 0x00000003)
#define CM_GPCTL0_SRC__PLLA ((u32) 0x00000004)
#define CM_GPCTL0_SRC__PLLC ((u32) 0x00000005)
#define CM_GPCTL0_SRC__PLLD ((u32) 0x00000006)
#define CM_GPCTL0_SRC__HDMI_AUXILIARY ((u32) 0x00000007)
/* ENAB - Enable the clock generator (RW) */
#define CM_GPCTL0_ENAB_OFS (4)
#define CM_GPCTL0_ENAB ((u32) 0x00000010)
/* KILL - Kill the clock generator (RW) */
#define CM_GPCTL0_KILL_OFS (5)
#define CM_GPCTL0_KILL ((u32) 0x00000020)
/* BUSY - Clock generator is running (R) */
#define CM_GPCTL0_BUSY_OFS (7)
#define CM_GPCTL0_BUSY ((u32) 0x00000080)
/* FLIP - Invert the clock generator output (RW) */
#define CM_GPCTL0_FLIP_OFS (8)
#define CM_GPCTL0_FLIP ((u32) 0x00000100)
/* MASH - MASH control (RW) */
#define CM_GPCTL0_MASH_OFS (9)
#define CM_GPCTL0_MASH_MASK ((u32) 0x00000600)
#define CM_GPCTL0_MASH0 ((u32) 0x00000200)
#define CM_GPCTL0_MASH1 ((u32) 0x00000400)
#define CM_GPCTL0_MASH_0 ((u32) 0x00000000)
#define CM_GPCTL0_MASH_1 ((u32) 0x00000200)
#define CM_GPCTL0_MASH_2 ((u32) 0x00000400)
#define CM_GPCTL0_MASH_3 ((u32) 0x00000600)
#define CM_GPCTL0_MASH__INTEGER_DIVISION ((u32) 0x00000000)
#define CM_GPCTL0_MASH__1STAGE_MASH ((u32) 0x00000200)
#define CM_GPCTL0_MASH__2STAGE_MASH ((u32) 0x00000400)
#define CM_GPCTL0_MASH__3STAGE_MASH ((u32) 0x00000600)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPCTL0_PASSWD_OFS (24)
#define CM_GPCTL0_PASSWD_MASK ((u32) 0xFF000000)
#define CM_GPCTL0_PASSWD0 ((u32) 0x01000000)
#define CM_GPCTL0_PASSWD1 ((u32) 0x02000000)
#define CM_GPCTL0_PASSWD2 ((u32) 0x04000000)
#define CM_GPCTL0_PASSWD3 ((u32) 0x08000000)
#define CM_GPCTL0_PASSWD4 ((u32) 0x10000000)
#define CM_GPCTL0_PASSWD5 ((u32) 0x20000000)
#define CM_GPCTL0_PASSWD6 ((u32) 0x40000000)
#define CM_GPCTL0_PASSWD7 ((u32) 0x80000000)

/* Clock Manager General Purpose Clocks Control Register 1 (RW) */
/* SRC - Clock source (RW) */
#define CM_GPCTL1_SRC_OFS (0)
#define CM_GPCTL1_SRC_MASK ((u32) 0x0000000F)
#define CM_GPCTL1_SRC0 ((u32) 0x00000001)
#define CM_GPCTL1_SRC1 ((u32) 0x00000002)
#define CM_GPCTL1_SRC2 ((u32) 0x00000004)
#define CM_GPCTL1_SRC3 ((u32) 0x00000008)
#define CM_GPCTL1_SRC_0 ((u32) 0x00000000)
#define CM_GPCTL1_SRC_1 ((u32) 0x00000001)
#define CM_GPCTL1_SRC_2 ((u32) 0x00000002)
#define CM_GPCTL1_SRC_3 ((u32) 0x00000003)
#define CM_GPCTL1_SRC_4 ((u32) 0x00000004)
#define CM_GPCTL1_SRC_5 ((u32) 0x00000005)
#define CM_GPCTL1_SRC_6 ((u32) 0x00000006)
#define CM_GPCTL1_SRC_7 ((u32) 0x00000007)
#define CM_GPCTL1_SRC_8 ((u32) 0x00000008)
#define CM_GPCTL1_SRC_9 ((u32) 0x00000009)
#define CM_GPCTL1_SRC_10 ((u32) 0x0000000A)
#define CM_GPCTL1_SRC_11 ((u32) 0x0000000B)
#define CM_GPCTL1_SRC_12 ((u32) 0x0000000C)
#define CM_GPCTL1_SRC_13 ((u32) 0x0000000D)
#define CM_GPCTL1_SRC_14 ((u32) 0x0000000E)
#define CM_GPCTL1_SRC_15 ((u32) 0x0000000F)
#define CM_GPCTL1_SRC__GND ((u32) 0x00000000)
#define CM_GPCTL1_SRC__OSCILLATOR ((u32) 0x00000001)
#define CM_GPCTL1_SRC__TESTDEBUG0 ((u32) 0x00000002)
#define CM_GPCTL1_SRC__TESTDEBUG1 ((u32) 0x00000003)
#define CM_GPCTL1_SRC__PLLA ((u32) 0x00000004)
#define CM_GPCTL1_SRC__PLLC ((u32) 0x00000005)
#define CM_GPCTL1_SRC__PLLD ((u32) 0x00000006)
#define CM_GPCTL1_SRC__HDMI_AUXILIARY ((u32) 0x00000007)
/* ENAB - Enable the clock generator (RW) */
#define CM_GPCTL1_ENAB_OFS (4)
#define CM_GPCTL1_ENAB ((u32) 0x00000010)
/* KILL - Kill the clock generator (RW) */
#define CM_GPCTL1_KILL_OFS (5)
#define CM_GPCTL1_KILL ((u32) 0x00000020)
/* BUSY - Clock generator is running (R) */
#define CM_GPCTL1_BUSY_OFS (7)
#define CM_GPCTL1_BUSY ((u32) 0x00000080)
/* FLIP - Invert the clock generator output (RW) */
#define CM_GPCTL1_FLIP_OFS (8)
#define CM_GPCTL1_FLIP ((u32) 0x00000100)
/* MASH - MASH control (RW) */
#define CM_GPCTL1_MASH_OFS (9)
#define CM_GPCTL1_MASH_MASK ((u32) 0x00000600)
#define CM_GPCTL1_MASH0 ((u32) 0x00000200)
#define CM_GPCTL1_MASH1 ((u32) 0x00000400)
#define CM_GPCTL1_MASH_0 ((u32) 0x00000000)
#define CM_GPCTL1_MASH_1 ((u32) 0x00000200)
#define CM_GPCTL1_MASH_2 ((u32) 0x00000400)
#define CM_GPCTL1_MASH_3 ((u32) 0x00000600)
#define CM_GPCTL1_MASH__INTEGER_DIVISION ((u32) 0x00000000)
#define CM_GPCTL1_MASH__1STAGE_MASH ((u32) 0x00000200)
#define CM_GPCTL1_MASH__2STAGE_MASH ((u32) 0x00000400)
#define CM_GPCTL1_MASH__3STAGE_MASH ((u32) 0x00000600)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPCTL1_PASSWD_OFS (24)
#define CM_GPCTL1_PASSWD_MASK ((u32) 0xFF000000)
#define CM_GPCTL1_PASSWD0 ((u32) 0x01000000)
#define CM_GPCTL1_PASSWD1 ((u32) 0x02000000)
#define CM_GPCTL1_PASSWD2 ((u32) 0x04000000)
#define CM_GPCTL1_PASSWD3 ((u32) 0x08000000)
#define CM_GPCTL1_PASSWD4 ((u32) 0x10000000)
#define CM_GPCTL1_PASSWD5 ((u32) 0x20000000)
#define CM_GPCTL1_PASSWD6 ((u32) 0x40000000)
#define CM_GPCTL1_PASSWD7 ((u32) 0x80000000)

/* Clock Manager General Purpose Clocks Control Register 2 (RW) */
/* SRC - Clock source (RW) */
#define CM_GPCTL2_SRC_OFS (0)
#define CM_GPCTL2_SRC_MASK ((u32) 0x0000000F)
#define CM_GPCTL2_SRC0 ((u32) 0x00000001)
#define CM_GPCTL2_SRC1 ((u32) 0x00000002)
#define CM_GPCTL2_SRC2 ((u32) 0x00000004)
#define CM_GPCTL2_SRC3 ((u32) 0x00000008)
#define CM_GPCTL2_SRC_0 ((u32) 0x00000000)
#define CM_GPCTL2_SRC_1 ((u32) 0x00000001)
#define CM_GPCTL2_SRC_2 ((u32) 0x00000002)
#define CM_GPCTL2_SRC_3 ((u32) 0x00000003)
#define CM_GPCTL2_SRC_4 ((u32) 0x00000004)
#define CM_GPCTL2_SRC_5 ((u32) 0x00000005)
#define CM_GPCTL2_SRC_6 ((u32) 0x00000006)
#define CM_GPCTL2_SRC_7 ((u32) 0x00000007)
#define CM_GPCTL2_SRC_8 ((u32) 0x00000008)
#define CM_GPCTL2_SRC_9 ((u32) 0x00000009)
#define CM_GPCTL2_SRC_10 ((u32) 0x0000000A)
#define CM_GPCTL2_SRC_11 ((u32) 0x0000000B)
#define CM_GPCTL2_SRC_12 ((u32) 0x0000000C)
#define CM_GPCTL2_SRC_13 ((u32) 0x0000000D)
#define CM_GPCTL2_SRC_14 ((u32) 0x0000000E)
#define CM_GPCTL2_SRC_15 ((u32) 0x0000000F)
#define CM_GPCTL2_SRC__GND ((u32) 0x00000000)
#define CM_GPCTL2_SRC__OSCILLATOR ((u32) 0x00000001)
#define CM_GPCTL2_SRC__TESTDEBUG0 ((u32) 0x00000002)
#define CM_GPCTL2_SRC__TESTDEBUG1 ((u32) 0x00000003)
#define CM_GPCTL2_SRC__PLLA ((u32) 0x00000004)
#define CM_GPCTL2_SRC__PLLC ((u32) 0x00000005)
#define CM_GPCTL2_SRC__PLLD ((u32) 0x00000006)
#define CM_GPCTL2_SRC__HDMI_AUXILIARY ((u32) 0x00000007)
/* ENAB - Enable the clock generator (RW) */
#define CM_GPCTL2_ENAB_OFS (4)
#define CM_GPCTL2_ENAB ((u32) 0x00000010)
/* KILL - Kill the clock generator (RW) */
#define CM_GPCTL2_KILL_OFS (5)
#define CM_GPCTL2_KILL ((u32) 0x00000020)
/* BUSY - Clock generator is running (R) */
#define CM_GPCTL2_BUSY_OFS (7)
#define CM_GPCTL2_BUSY ((u32) 0x00000080)
/* FLIP - Invert the clock generator output (RW) */
#define CM_GPCTL2_FLIP_OFS (8)
#define CM_GPCTL2_FLIP ((u32) 0x00000100)
/* MASH - MASH control (RW) */
#define CM_GPCTL2_MASH_OFS (9)
#define CM_GPCTL2_MASH_MASK ((u32) 0x00000600)
#define CM_GPCTL2_MASH0 ((u32) 0x00000200)
#define CM_GPCTL2_MASH1 ((u32) 0x00000400)
#define CM_GPCTL2_MASH_0 ((u32) 0x00000000)
#define CM_GPCTL2_MASH_1 ((u32) 0x00000200)
#define CM_GPCTL2_MASH_2 ((u32) 0x00000400)
#define CM_GPCTL2_MASH_3 ((u32) 0x00000600)
#define CM_GPCTL2_MASH__INTEGER_DIVISION ((u32) 0x00000000)
#define CM_GPCTL2_MASH__1STAGE_MASH ((u32) 0x00000200)
#define CM_GPCTL2_MASH__2STAGE_MASH ((u32) 0x00000400)
#define CM_GPCTL2_MASH__3STAGE_MASH ((u32) 0x00000600)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPCTL2_PASSWD_OFS (24)
#define CM_GPCTL2_PASSWD_MASK ((u32) 0xFF000000)
#define CM_GPCTL2_PASSWD0 ((u32) 0x01000000)
#define CM_GPCTL2_PASSWD1 ((u32) 0x02000000)
#define CM_GPCTL2_PASSWD2 ((u32) 0x04000000)
#define CM_GPCTL2_PASSWD3 ((u32) 0x08000000)
#define CM_GPCTL2_PASSWD4 ((u32) 0x10000000)
#define CM_GPCTL2_PASSWD5 ((u32) 0x20000000)
#define CM_GPCTL2_PASSWD6 ((u32) 0x40000000)
#define CM_GPCTL2_PASSWD7 ((u32) 0x80000000)

/* Clock Manager General Purpose Clock Divisors Register 0 (RW) */
/* DIVF - Fractional part of divisor (RW) */
#define CM_GPDIV0_DIVF_OFS (0)
#define CM_GPDIV0_DIVF_MASK ((u32) 0x00000FFF)
#define CM_GPDIV0_DIVF0 ((u32) 0x00000001)
#define CM_GPDIV0_DIVF1 ((u32) 0x00000002)
#define CM_GPDIV0_DIVF2 ((u32) 0x00000004)
#define CM_GPDIV0_DIVF3 ((u32) 0x00000008)
#define CM_GPDIV0_DIVF4 ((u32) 0x00000010)
#define CM_GPDIV0_DIVF5 ((u32) 0x00000020)
#define CM_GPDIV0_DIVF6 ((u32) 0x00000040)
#define CM_GPDIV0_DIVF7 ((u32) 0x00000080)
#define CM_GPDIV0_DIVF8 ((u32) 0x00000100)
#define CM_GPDIV0_DIVF9 ((u32) 0x00000200)
#define CM_GPDIV0_DIVF10 ((u32) 0x00000400)
#define CM_GPDIV0_DIVF11 ((u32) 0x00000800)
/* DIVI - Integer part of divisor (RW) */
#define CM_GPDIV0_DIVI_OFS (12)
#define CM_GPDIV0_DIVI_MASK ((u32) 0x00FFF000)
#define CM_GPDIV0_DIVI0 ((u32) 0x00001000)
#define CM_GPDIV0_DIVI1 ((u32) 0x00002000)
#define CM_GPDIV0_DIVI2 ((u32) 0x00004000)
#define CM_GPDIV0_DIVI3 ((u32) 0x00008000)
#define CM_GPDIV0_DIVI4 ((u32) 0x00010000)
#define CM_GPDIV0_DIVI5 ((u32) 0x00020000)
#define CM_GPDIV0_DIVI6 ((u32) 0x00040000)
#define CM_GPDIV0_DIVI7 ((u32) 0x00080000)
#define CM_GPDIV0_DIVI8 ((u32) 0x00100000)
#define CM_GPDIV0_DIVI9 ((u32) 0x00200000)
#define CM_GPDIV0_DIVI10 ((u32) 0x00400000)
#define CM_GPDIV0_DIVI11 ((u32) 0x00800000)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPDIV0_PASSWD_OFS (24)
#define CM_GPDIV0_PASSWD_MASK ((u32) 0xFF000000)
#define CM_GPDIV0_PASSWD0 ((u32) 0x01000000)
#define CM_GPDIV0_PASSWD1 ((u32) 0x02000000)
#define CM_GPDIV0_PASSWD2 ((u32) 0x04000000)
#define CM_GPDIV0_PASSWD3 ((u32) 0x08000000)
#define CM_GPDIV0_PASSWD4 ((u32) 0x10000000)
#define CM_GPDIV0_PASSWD5 ((u32) 0x20000000)
#define CM_GPDIV0_PASSWD6 ((u32) 0x40000000)
#define CM_GPDIV0_PASSWD7 ((u32) 0x80000000)
/* Clock Manager General Purpose Clock Divisors Register 1 (RW) */
/* DIVF - Fractional part of divisor (RW) */
#define CM_GPDIV1_DIVF_OFS (0)
#define CM_GPDIV1_DIVF_MASK ((u32) 0x00000FFF)
#define CM_GPDIV1_DIVF0 ((u32) 0x00000001)
#define CM_GPDIV1_DIVF1 ((u32) 0x00000002)
#define CM_GPDIV1_DIVF2 ((u32) 0x00000004)
#define CM_GPDIV1_DIVF3 ((u32) 0x00000008)
#define CM_GPDIV1_DIVF4 ((u32) 0x00000010)
#define CM_GPDIV1_DIVF5 ((u32) 0x00000020)
#define CM_GPDIV1_DIVF6 ((u32) 0x00000040)
#define CM_GPDIV1_DIVF7 ((u32) 0x00000080)
#define CM_GPDIV1_DIVF8 ((u32) 0x00000100)
#define CM_GPDIV1_DIVF9 ((u32) 0x00000200)
#define CM_GPDIV1_DIVF10 ((u32) 0x00000400)
#define CM_GPDIV1_DIVF11 ((u32) 0x00000800)
/* DIVI - Integer part of divisor (RW) */
#define CM_GPDIV1_DIVI_OFS (12)
#define CM_GPDIV1_DIVI_MASK ((u32) 0x00FFF000)
#define CM_GPDIV1_DIVI0 ((u32) 0x00001000)
#define CM_GPDIV1_DIVI1 ((u32) 0x00002000)
#define CM_GPDIV1_DIVI2 ((u32) 0x00004000)
#define CM_GPDIV1_DIVI3 ((u32) 0x00008000)
#define CM_GPDIV1_DIVI4 ((u32) 0x00010000)
#define CM_GPDIV1_DIVI5 ((u32) 0x00020000)
#define CM_GPDIV1_DIVI6 ((u32) 0x00040000)
#define CM_GPDIV1_DIVI7 ((u32) 0x00080000)
#define CM_GPDIV1_DIVI8 ((u32) 0x00100000)
#define CM_GPDIV1_DIVI9 ((u32) 0x00200000)
#define CM_GPDIV1_DIVI10 ((u32) 0x00400000)
#define CM_GPDIV1_DIVI11 ((u32) 0x00800000)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPDIV1_PASSWD_OFS (24)
#define CM_GPDIV1_PASSWD_MASK ((u32) 0xFF000000)
#define CM_GPDIV1_PASSWD0 ((u32) 0x01000000)
#define CM_GPDIV1_PASSWD1 ((u32) 0x02000000)
#define CM_GPDIV1_PASSWD2 ((u32) 0x04000000)
#define CM_GPDIV1_PASSWD3 ((u32) 0x08000000)
#define CM_GPDIV1_PASSWD4 ((u32) 0x10000000)
#define CM_GPDIV1_PASSWD5 ((u32) 0x20000000)
#define CM_GPDIV1_PASSWD6 ((u32) 0x40000000)
#define CM_GPDIV1_PASSWD7 ((u32) 0x80000000)
/* Clock Manager General Purpose Clock Divisors Register 2 (RW) */
/* DIVF - Fractional part of divisor (RW) */
#define CM_GPDIV2_DIVF_OFS (0)
#define CM_GPDIV2_DIVF_MASK ((u32) 0x00000FFF)
#define CM_GPDIV2_DIVF0 ((u32) 0x00000001)
#define CM_GPDIV2_DIVF1 ((u32) 0x00000002)
#define CM_GPDIV2_DIVF2 ((u32) 0x00000004)
#define CM_GPDIV2_DIVF3 ((u32) 0x00000008)
#define CM_GPDIV2_DIVF4 ((u32) 0x00000010)
#define CM_GPDIV2_DIVF5 ((u32) 0x00000020)
#define CM_GPDIV2_DIVF6 ((u32) 0x00000040)
#define CM_GPDIV2_DIVF7 ((u32) 0x00000080)
#define CM_GPDIV2_DIVF8 ((u32) 0x00000100)
#define CM_GPDIV2_DIVF9 ((u32) 0x00000200)
#define CM_GPDIV2_DIVF10 ((u32) 0x00000400)
#define CM_GPDIV2_DIVF11 ((u32) 0x00000800)
/* DIVI - Integer part of divisor (RW) */
#define CM_GPDIV2_DIVI_OFS (12)
#define CM_GPDIV2_DIVI_MASK ((u32) 0x00FFF000)
#define CM_GPDIV2_DIVI0 ((u32) 0x00001000)
#define CM_GPDIV2_DIVI1 ((u32) 0x00002000)
#define CM_GPDIV2_DIVI2 ((u32) 0x00004000)
#define CM_GPDIV2_DIVI3 ((u32) 0x00008000)
#define CM_GPDIV2_DIVI4 ((u32) 0x00010000)
#define CM_GPDIV2_DIVI5 ((u32) 0x00020000)
#define CM_GPDIV2_DIVI6 ((u32) 0x00040000)
#define CM_GPDIV2_DIVI7 ((u32) 0x00080000)
#define CM_GPDIV2_DIVI8 ((u32) 0x00100000)
#define CM_GPDIV2_DIVI9 ((u32) 0x00200000)
#define CM_GPDIV2_DIVI10 ((u32) 0x00400000)
#define CM_GPDIV2_DIVI11 ((u32) 0x00800000)
/* PASSWD - Clock Manager Password "5a" (W) */
#define CM_GPDIV2_PASSWD_OFS (24)
#define CM_GPDIV2_PASSWD_MASK ((u32) 0xFF000000)
#define CM_GPDIV2_PASSWD0 ((u32) 0x01000000)
#define CM_GPDIV2_PASSWD1 ((u32) 0x02000000)
#define CM_GPDIV2_PASSWD2 ((u32) 0x04000000)
#define CM_GPDIV2_PASSWD3 ((u32) 0x08000000)
#define CM_GPDIV2_PASSWD4 ((u32) 0x10000000)
#define CM_GPDIV2_PASSWD5 ((u32) 0x20000000)
#define CM_GPDIV2_PASSWD6 ((u32) 0x40000000)
#define CM_GPDIV2_PASSWD7 ((u32) 0x80000000)

/* Interrupt Controller Bits */
/* IRQ Basic Pending bits (R) */
#define IRQ_BP_ARM_TIMER_OFS (0)
#define IRQ_BP_ARM_TIMER ((u32) 0x00000001)
#define IRQ_BP_ARM_MAILBOX_OFS (1)
#define IRQ_BP_ARM_MAILBOX ((u32) 0x00000002)
#define IRQ_BP_ARM_DOORBELL_0_OFS (2)
#define IRQ_BP_ARM_DOORBELL_0 ((u32) 0x00000004)
#define IRQ_BP_ARM_DOORBELL_1_OFS (3)
#define IRQ_BP_ARM_DOORBELL_1 ((u32) 0x00000008)
#define IRQ_BP_GPU0_HALTED_OFS (4)
#define IRQ_BP_GPU0_HALTED ((u32) 0x00000010)
#define IRQ_BP_GPU1_HALTED_OFS (5)
#define IRQ_BP_GPU1_HALTED ((u32) 0x00000020)
#define IRQ_BP_ILLEGAL_ACCESS_TYPE_1_OFS (6)
#define IRQ_BP_ILLEGAL_ACCESS_TYPE_1 ((u32) 0x00000040)
#define IRQ_BP_ILLEGAL_ACCESS_TYPE_0_OFS (7)
#define IRQ_BP_ILLEGAL_ACCESS_TYPE_0 ((u32) 0x00000080)
#define IRQ_BP_BITS_SET_IN_PR1_OFS (8)
#define IRQ_BP_BITS_SET_IN_PR1 ((u32) 0x00000100)
#define IRQ_BP_BITS_SET_IN_PR2_OFS (9)
#define IRQ_BP_BITS_SET_IN_PR2 ((u32) 0x00000200)
#define IRQ_BP_GPU_IRQ7_OFS (10)
#define IRQ_BP_GPU_IRQ7 ((u32) 0x00000400)
#define IRQ_BP_GPU_IRQ9_OFS (11)
#define IRQ_BP_GPU_IRQ9 ((u32) 0x00000800)
#define IRQ_BP_GPU_IRQ10_OFS (12)
#define IRQ_BP_GPU_IRQ10 ((u32) 0x00001000)
#define IRQ_BP_GPU_IRQ18_OFS (13)
#define IRQ_BP_GPU_IRQ18 ((u32) 0x00002000)
#define IRQ_BP_GPU_IRQ19_OFS (14)
#define IRQ_BP_GPU_IRQ19 ((u32) 0x00004000)
#define IRQ_BP_GPU_IRQ53_OFS (15)
#define IRQ_BP_GPU_IRQ53 ((u32) 0x00008000)
#define IRQ_BP_GPU_IRQ54_OFS (16)
#define IRQ_BP_GPU_IRQ54 ((u32) 0x00010000)
#define IRQ_BP_GPU_IRQ55_OFS (17)
#define IRQ_BP_GPU_IRQ55 ((u32) 0x00020000)
#define IRQ_BP_GPU_IRQ56_OFS (18)
#define IRQ_BP_GPU_IRQ56 ((u32) 0x00040000)
#define IRQ_BP_GPU_IRQ57_OFS (19)
#define IRQ_BP_GPU_IRQ57 ((u32) 0x00080000)
#define IRQ_BP_GPU_IRQ60_OFS (20)
#define IRQ_BP_GPU_IRQ60 ((u32) 0x00100000)
/* IRQ pending 1 (R) */
#define IRQ_PENDING1_SRC_OFS (0)
#define IRQ_PENDING1_SRC_MASK ((u32) 0xFFFFFFFF)
#define IRQ_PENDING10 ((u32) 0x00000001)
#define IRQ_PENDING11 ((u32) 0x00000002)
#define IRQ_PENDING12 ((u32) 0x00000004)
#define IRQ_PENDING13 ((u32) 0x00000008)
#define IRQ_PENDING14 ((u32) 0x00000010)
#define IRQ_PENDING15 ((u32) 0x00000020)
#define IRQ_PENDING16 ((u32) 0x00000040)
#define IRQ_PENDING17 ((u32) 0x00000080)
#define IRQ_PENDING18 ((u32) 0x00000100)
#define IRQ_PENDING19 ((u32) 0x00000200)
#define IRQ_PENDING110 ((u32) 0x00000400)
#define IRQ_PENDING111 ((u32) 0x00000800)
#define IRQ_PENDING112 ((u32) 0x00001000)
#define IRQ_PENDING113 ((u32) 0x00002000)
#define IRQ_PENDING114 ((u32) 0x00004000)
#define IRQ_PENDING115 ((u32) 0x00008000)
#define IRQ_PENDING116 ((u32) 0x00010000)
#define IRQ_PENDING117 ((u32) 0x00020000)
#define IRQ_PENDING118 ((u32) 0x00040000)
#define IRQ_PENDING119 ((u32) 0x00080000)
#define IRQ_PENDING120 ((u32) 0x00100000)
#define IRQ_PENDING121 ((u32) 0x00200000)
#define IRQ_PENDING122 ((u32) 0x00400000)
#define IRQ_PENDING123 ((u32) 0x00800000)
#define IRQ_PENDING124 ((u32) 0x01000000)
#define IRQ_PENDING125 ((u32) 0x02000000)
#define IRQ_PENDING126 ((u32) 0x04000000)
#define IRQ_PENDING127 ((u32) 0x08000000)
#define IRQ_PENDING128 ((u32) 0x10000000)
#define IRQ_PENDING129 ((u32) 0x20000000)
#define IRQ_PENDING130 ((u32) 0x40000000)
#define IRQ_PENDING131 ((u32) 0x80000000)
#define IRQ_PENDING1_0 ((u32) 0x00000001)
#define IRQ_PENDING1_1 ((u32) 0x00000002)
#define IRQ_PENDING1_2 ((u32) 0x00000004)
#define IRQ_PENDING1_3 ((u32) 0x00000008)
#define IRQ_PENDING1_4 ((u32) 0x00000010)
#define IRQ_PENDING1_5 ((u32) 0x00000020)
#define IRQ_PENDING1_6 ((u32) 0x00000040)
#define IRQ_PENDING1_7 ((u32) 0x00000080)
#define IRQ_PENDING1_8 ((u32) 0x00000100)
#define IRQ_PENDING1_9 ((u32) 0x00000200)
#define IRQ_PENDING1_10 ((u32) 0x00000400)
#define IRQ_PENDING1_11 ((u32) 0x00000800)
#define IRQ_PENDING1_12 ((u32) 0x00001000)
#define IRQ_PENDING1_13 ((u32) 0x00002000)
#define IRQ_PENDING1_14 ((u32) 0x00004000)
#define IRQ_PENDING1_15 ((u32) 0x00008000)
#define IRQ_PENDING1_16 ((u32) 0x00010000)
#define IRQ_PENDING1_17 ((u32) 0x00020000)
#define IRQ_PENDING1_18 ((u32) 0x00040000)
#define IRQ_PENDING1_19 ((u32) 0x00080000)
#define IRQ_PENDING1_20 ((u32) 0x00100000)
#define IRQ_PENDING1_21 ((u32) 0x00200000)
#define IRQ_PENDING1_22 ((u32) 0x00400000)
#define IRQ_PENDING1_23 ((u32) 0x00800000)
#define IRQ_PENDING1_24 ((u32) 0x01000000)
#define IRQ_PENDING1_25 ((u32) 0x02000000)
#define IRQ_PENDING1_26 ((u32) 0x04000000)
#define IRQ_PENDING1_27 ((u32) 0x08000000)
#define IRQ_PENDING1_28 ((u32) 0x10000000)
#define IRQ_PENDING1_29 ((u32) 0x20000000)
#define IRQ_PENDING1_30 ((u32) 0x40000000)
#define IRQ_PENDING1_31 ((u32) 0x80000000)
#define IRQ_PENDING1__SYSTMR_MATCH_1 ((u32) 0x00000001)
#define IRQ_PENDING1__SYSTMR_MATCH_3 ((u32) 0x00000008)
#define IRQ_PENDING1__USB_CONTROLLER ((u32) 0x00000200)
#define IRQ_PENDING1__AUX_INT ((u32) 0x20000000)

/* IRQ pending 2 (R) */
#define IRQ_PENDING2_SRC_OFS (0)
#define IRQ_PENDING2_SRC_MASK ((u32) 0xFFFFFFFF)
#define IRQ_PENDING20 ((u32) 0x00000001)
#define IRQ_PENDING21 ((u32) 0x00000002)
#define IRQ_PENDING22 ((u32) 0x00000004)
#define IRQ_PENDING23 ((u32) 0x00000008)
#define IRQ_PENDING24 ((u32) 0x00000010)
#define IRQ_PENDING25 ((u32) 0x00000020)
#define IRQ_PENDING26 ((u32) 0x00000040)
#define IRQ_PENDING27 ((u32) 0x00000080)
#define IRQ_PENDING28 ((u32) 0x00000100)
#define IRQ_PENDING29 ((u32) 0x00000200)
#define IRQ_PENDING210 ((u32) 0x00000400)
#define IRQ_PENDING211 ((u32) 0x00000800)
#define IRQ_PENDING212 ((u32) 0x00001000)
#define IRQ_PENDING213 ((u32) 0x00002000)
#define IRQ_PENDING214 ((u32) 0x00004000)
#define IRQ_PENDING215 ((u32) 0x00008000)
#define IRQ_PENDING216 ((u32) 0x00010000)
#define IRQ_PENDING217 ((u32) 0x00020000)
#define IRQ_PENDING218 ((u32) 0x00040000)
#define IRQ_PENDING219 ((u32) 0x00080000)
#define IRQ_PENDING220 ((u32) 0x00100000)
#define IRQ_PENDING221 ((u32) 0x00200000)
#define IRQ_PENDING222 ((u32) 0x00400000)
#define IRQ_PENDING223 ((u32) 0x00800000)
#define IRQ_PENDING224 ((u32) 0x01000000)
#define IRQ_PENDING225 ((u32) 0x02000000)
#define IRQ_PENDING226 ((u32) 0x04000000)
#define IRQ_PENDING227 ((u32) 0x08000000)
#define IRQ_PENDING228 ((u32) 0x10000000)
#define IRQ_PENDING229 ((u32) 0x20000000)
#define IRQ_PENDING230 ((u32) 0x40000000)
#define IRQ_PENDING231 ((u32) 0x80000000)
#define IRQ_PENDING2_0 ((u32) 0x00000001)
#define IRQ_PENDING2_1 ((u32) 0x00000002)
#define IRQ_PENDING2_2 ((u32) 0x00000004)
#define IRQ_PENDING2_3 ((u32) 0x00000008)
#define IRQ_PENDING2_4 ((u32) 0x00000010)
#define IRQ_PENDING2_5 ((u32) 0x00000020)
#define IRQ_PENDING2_6 ((u32) 0x00000040)
#define IRQ_PENDING2_7 ((u32) 0x00000080)
#define IRQ_PENDING2_8 ((u32) 0x00000100)
#define IRQ_PENDING2_9 ((u32) 0x00000200)
#define IRQ_PENDING2_10 ((u32) 0x00000400)
#define IRQ_PENDING2_11 ((u32) 0x00000800)
#define IRQ_PENDING2_12 ((u32) 0x00001000)
#define IRQ_PENDING2_13 ((u32) 0x00002000)
#define IRQ_PENDING2_14 ((u32) 0x00004000)
#define IRQ_PENDING2_15 ((u32) 0x00008000)
#define IRQ_PENDING2_16 ((u32) 0x00010000)
#define IRQ_PENDING2_17 ((u32) 0x00020000)
#define IRQ_PENDING2_18 ((u32) 0x00040000)
#define IRQ_PENDING2_19 ((u32) 0x00080000)
#define IRQ_PENDING2_20 ((u32) 0x00100000)
#define IRQ_PENDING2_21 ((u32) 0x00200000)
#define IRQ_PENDING2_22 ((u32) 0x00400000)
#define IRQ_PENDING2_23 ((u32) 0x00800000)
#define IRQ_PENDING2_24 ((u32) 0x01000000)
#define IRQ_PENDING2_25 ((u32) 0x02000000)
#define IRQ_PENDING2_26 ((u32) 0x04000000)
#define IRQ_PENDING2_27 ((u32) 0x08000000)
#define IRQ_PENDING2_28 ((u32) 0x10000000)
#define IRQ_PENDING2_29 ((u32) 0x20000000)
#define IRQ_PENDING2_30 ((u32) 0x40000000)
#define IRQ_PENDING2_31 ((u32) 0x80000000)
#define IRQ_PENDING2__I2C_SPI_SLV_INT ((u32) 0x00000800)
#define IRQ_PENDING2__PWA0 ((u32) 0x00002000)
#define IRQ_PENDING2__PWA1 ((u32) 0x00004000)
#define IRQ_PENDING2__SMI ((u32) 0x00010000)
#define IRQ_PENDING2__GPIO_INT_0 ((u32) 0x00020000)
#define IRQ_PENDING2__GPIO_INT_1 ((u32) 0x00040000)
#define IRQ_PENDING2__GPIO_INT_2 ((u32) 0x00080000)
#define IRQ_PENDING2__GPIO_INT_3 ((u32) 0x00100000)
#define IRQ_PENDING2__I2C_INT ((u32) 0x00200000)
#define IRQ_PENDING2__SPI_INT ((u32) 0x00400000)
#define IRQ_PENDING2__PCM_INT ((u32) 0x00800000)
#define IRQ_PENDING2__UART_INT ((u32) 0x02000000)

/* FIQ control (RW) */
#define IRQ_FIQ_SRC_OFS (0)
#define IRQ_FIQ_SRC_MASK ((u32) 0x0000007F)
#define IRQ_FIQ_SRC0 ((u32) 0x00000001)
#define IRQ_FIQ_SRC1 ((u32) 0x00000002)
#define IRQ_FIQ_SRC2 ((u32) 0x00000004)
#define IRQ_FIQ_SRC3 ((u32) 0x00000008)
#define IRQ_FIQ_SRC4 ((u32) 0x00000010)
#define IRQ_FIQ_SRC5 ((u32) 0x00000020)
#define IRQ_FIQ_SRC6 ((u32) 0x00000040)
#define IRQ_FIQ_SRC_0 ((u32) 0x00000000)
#define IRQ_FIQ_SRC_1 ((u32) 0x00000001)
#define IRQ_FIQ_SRC_2 ((u32) 0x00000002)
#define IRQ_FIQ_SRC_3 ((u32) 0x00000003)
#define IRQ_FIQ_SRC_4 ((u32) 0x00000004)
#define IRQ_FIQ_SRC_5 ((u32) 0x00000005)
#define IRQ_FIQ_SRC_6 ((u32) 0x00000006)
#define IRQ_FIQ_SRC_7 ((u32) 0x00000007)
#define IRQ_FIQ_SRC_8 ((u32) 0x00000008)
#define IRQ_FIQ_SRC_9 ((u32) 0x00000009)
#define IRQ_FIQ_SRC_10 ((u32) 0x0000000A)
#define IRQ_FIQ_SRC_11 ((u32) 0x0000000B)
#define IRQ_FIQ_SRC_12 ((u32) 0x0000000C)
#define IRQ_FIQ_SRC_13 ((u32) 0x0000000D)
#define IRQ_FIQ_SRC_14 ((u32) 0x0000000E)
#define IRQ_FIQ_SRC_15 ((u32) 0x0000000F)
#define IRQ_FIQ_SRC_16 ((u32) 0x00000010)
#define IRQ_FIQ_SRC_17 ((u32) 0x00000011)
#define IRQ_FIQ_SRC_18 ((u32) 0x00000012)
#define IRQ_FIQ_SRC_19 ((u32) 0x00000013)
#define IRQ_FIQ_SRC_20 ((u32) 0x00000014)
#define IRQ_FIQ_SRC_21 ((u32) 0x00000015)
#define IRQ_FIQ_SRC_22 ((u32) 0x00000016)
#define IRQ_FIQ_SRC_23 ((u32) 0x00000017)
#define IRQ_FIQ_SRC_24 ((u32) 0x00000018)
#define IRQ_FIQ_SRC_25 ((u32) 0x00000019)
#define IRQ_FIQ_SRC_26 ((u32) 0x0000001A)
#define IRQ_FIQ_SRC_27 ((u32) 0x0000001B)
#define IRQ_FIQ_SRC_28 ((u32) 0x0000001C)
#define IRQ_FIQ_SRC_29 ((u32) 0x0000001D)
#define IRQ_FIQ_SRC_30 ((u32) 0x0000001E)
#define IRQ_FIQ_SRC_31 ((u32) 0x0000001F)
#define IRQ_FIQ_SRC_32 ((u32) 0x00000020)
#define IRQ_FIQ_SRC_33 ((u32) 0x00000021)
#define IRQ_FIQ_SRC_34 ((u32) 0x00000022)
#define IRQ_FIQ_SRC_35 ((u32) 0x00000023)
#define IRQ_FIQ_SRC_36 ((u32) 0x00000024)
#define IRQ_FIQ_SRC_37 ((u32) 0x00000025)
#define IRQ_FIQ_SRC_38 ((u32) 0x00000026)
#define IRQ_FIQ_SRC_39 ((u32) 0x00000027)
#define IRQ_FIQ_SRC_40 ((u32) 0x00000028)
#define IRQ_FIQ_SRC_41 ((u32) 0x00000029)
#define IRQ_FIQ_SRC_42 ((u32) 0x0000002A)
#define IRQ_FIQ_SRC_43 ((u32) 0x0000002B)
#define IRQ_FIQ_SRC_44 ((u32) 0x0000002C)
#define IRQ_FIQ_SRC_45 ((u32) 0x0000002D)
#define IRQ_FIQ_SRC_46 ((u32) 0x0000002E)
#define IRQ_FIQ_SRC_47 ((u32) 0x0000002F)
#define IRQ_FIQ_SRC_48 ((u32) 0x00000030)
#define IRQ_FIQ_SRC_49 ((u32) 0x00000031)
#define IRQ_FIQ_SRC_50 ((u32) 0x00000032)
#define IRQ_FIQ_SRC_51 ((u32) 0x00000033)
#define IRQ_FIQ_SRC_52 ((u32) 0x00000034)
#define IRQ_FIQ_SRC_53 ((u32) 0x00000035)
#define IRQ_FIQ_SRC_54 ((u32) 0x00000036)
#define IRQ_FIQ_SRC_55 ((u32) 0x00000037)
#define IRQ_FIQ_SRC_56 ((u32) 0x00000038)
#define IRQ_FIQ_SRC_57 ((u32) 0x00000039)
#define IRQ_FIQ_SRC_58 ((u32) 0x0000003A)
#define IRQ_FIQ_SRC_59 ((u32) 0x0000003B)
#define IRQ_FIQ_SRC_60 ((u32) 0x0000003C)
#define IRQ_FIQ_SRC_61 ((u32) 0x0000003D)
#define IRQ_FIQ_SRC_62 ((u32) 0x0000003E)
#define IRQ_FIQ_SRC_63 ((u32) 0x0000003F)
#define IRQ_FIQ_SRC_64 ((u32) 0x00000040)
#define IRQ_FIQ_SRC_65 ((u32) 0x00000041)
#define IRQ_FIQ_SRC_66 ((u32) 0x00000042)
#define IRQ_FIQ_SRC_67 ((u32) 0x00000043)
#define IRQ_FIQ_SRC_68 ((u32) 0x00000044)
#define IRQ_FIQ_SRC_69 ((u32) 0x00000045)
#define IRQ_FIQ_SRC_70 ((u32) 0x00000046)
#define IRQ_FIQ_SRC_71 ((u32) 0x00000047)
/* 72-127 Do Not Use */
#define IRQ_FIQ_SRC_72 ((u32) 0x00000048)
#define IRQ_FIQ_SRC_73 ((u32) 0x00000049)
#define IRQ_FIQ_SRC_74 ((u32) 0x0000004A)
#define IRQ_FIQ_SRC_75 ((u32) 0x0000004B)
#define IRQ_FIQ_SRC_76 ((u32) 0x0000004C)
#define IRQ_FIQ_SRC_77 ((u32) 0x0000004D)
#define IRQ_FIQ_SRC_78 ((u32) 0x0000004E)
#define IRQ_FIQ_SRC_79 ((u32) 0x0000004F)
#define IRQ_FIQ_SRC_80 ((u32) 0x00000050)
#define IRQ_FIQ_SRC_81 ((u32) 0x00000051)
#define IRQ_FIQ_SRC_82 ((u32) 0x00000052)
#define IRQ_FIQ_SRC_83 ((u32) 0x00000053)
#define IRQ_FIQ_SRC_84 ((u32) 0x00000054)
#define IRQ_FIQ_SRC_85 ((u32) 0x00000055)
#define IRQ_FIQ_SRC_86 ((u32) 0x00000056)
#define IRQ_FIQ_SRC_87 ((u32) 0x00000057)
#define IRQ_FIQ_SRC_88 ((u32) 0x00000058)
#define IRQ_FIQ_SRC_89 ((u32) 0x00000059)
#define IRQ_FIQ_SRC_90 ((u32) 0x0000005A)
#define IRQ_FIQ_SRC_91 ((u32) 0x0000005B)
#define IRQ_FIQ_SRC_92 ((u32) 0x0000005C)
#define IRQ_FIQ_SRC_93 ((u32) 0x0000005D)
#define IRQ_FIQ_SRC_94 ((u32) 0x0000005E)
#define IRQ_FIQ_SRC_95 ((u32) 0x0000005F)
#define IRQ_FIQ_SRC_96 ((u32) 0x00000060)
#define IRQ_FIQ_SRC_97 ((u32) 0x00000061)
#define IRQ_FIQ_SRC_98 ((u32) 0x00000062)
#define IRQ_FIQ_SRC_99 ((u32) 0x00000063)
#define IRQ_FIQ_SRC_100 ((u32) 0x00000064)
#define IRQ_FIQ_SRC_101 ((u32) 0x00000065)
#define IRQ_FIQ_SRC_102 ((u32) 0x00000066)
#define IRQ_FIQ_SRC_103 ((u32) 0x00000067)
#define IRQ_FIQ_SRC_104 ((u32) 0x00000068)
#define IRQ_FIQ_SRC_105 ((u32) 0x00000069)
#define IRQ_FIQ_SRC_106 ((u32) 0x0000006A)
#define IRQ_FIQ_SRC_107 ((u32) 0x0000006B)
#define IRQ_FIQ_SRC_108 ((u32) 0x0000006C)
#define IRQ_FIQ_SRC_109 ((u32) 0x0000006D)
#define IRQ_FIQ_SRC_110 ((u32) 0x0000006E)
#define IRQ_FIQ_SRC_111 ((u32) 0x0000006F)
#define IRQ_FIQ_SRC_112 ((u32) 0x00000070)
#define IRQ_FIQ_SRC_113 ((u32) 0x00000071)
#define IRQ_FIQ_SRC_114 ((u32) 0x00000072)
#define IRQ_FIQ_SRC_115 ((u32) 0x00000073)
#define IRQ_FIQ_SRC_116 ((u32) 0x00000074)
#define IRQ_FIQ_SRC_117 ((u32) 0x00000075)
#define IRQ_FIQ_SRC_118 ((u32) 0x00000076)
#define IRQ_FIQ_SRC_119 ((u32) 0x00000077)
#define IRQ_FIQ_SRC_120 ((u32) 0x00000078)
#define IRQ_FIQ_SRC_121 ((u32) 0x00000079)
#define IRQ_FIQ_SRC_122 ((u32) 0x0000007A)
#define IRQ_FIQ_SRC_123 ((u32) 0x0000007B)
#define IRQ_FIQ_SRC_124 ((u32) 0x0000007C)
#define IRQ_FIQ_SRC_125 ((u32) 0x0000007D)
#define IRQ_FIQ_SRC_126 ((u32) 0x0000007E)
#define IRQ_FIQ_SRC_127 ((u32) 0x0000007F)
#define IRQ_FIQ_SRC__SYSTMR_MATCH_1 ((u32) 0x00000001)
#define IRQ_FIQ_SRC__SYSTMR_MATCH_3 ((u32) 0x00000003)
#define IRQ_FIQ_SRC__USB_CONTROLLER ((u32) 0x00000009)
#define IRQ_FIQ_SRC__AUX_INT ((u32) 0x0000001D)
#define IRQ_FIQ_SRC__I2C_SPI_SLV_INT ((u32) 0x0000002B)
#define IRQ_FIQ_SRC__PWA0 ((u32) 0x0000002D)
#define IRQ_FIQ_SRC__PWA1 ((u32) 0x0000002E)
#define IRQ_FIQ_SRC__SMI ((u32) 0x00000030)
#define IRQ_FIQ_SRC__GPIO_INT_0 ((u32) 0x00000031)
#define IRQ_FIQ_SRC__GPIO_INT_1 ((u32) 0x00000032)
#define IRQ_FIQ_SRC__GPIO_INT_2 ((u32) 0x00000033)
#define IRQ_FIQ_SRC__GPIO_INT_3 ((u32) 0x00000034)
#define IRQ_FIQ_SRC__I2C_INT ((u32) 0x00000035)
#define IRQ_FIQ_SRC__SPI_INT ((u32) 0x00000036)
#define IRQ_FIQ_SRC__PCM_INT ((u32) 0x00000037)
#define IRQ_FIQ_SRC__UART_INT ((u32) 0x00000039)
#define IRQ_FIQ_SRC__ARM_TIMER ((u32) 0x00000040)
#define IRQ_FIQ_SRC__ARM_MAILBOX ((u32) 0x00000041)
#define IRQ_FIQ_SRC__ARM_DOORBELL_0 ((u32) 0x00000042)
#define IRQ_FIQ_SRC__ARM_DOORBELL_1 ((u32) 0x00000043)
#define IRQ_FIQ_SRC__GPU0_HALTED ((u32) 0x00000044)
#define IRQ_FIQ_SRC__GPU1_HALTED ((u32) 0x00000045)
#define IRQ_FIQ_SRC__ILLEGAL_ACCESS_T1 ((u32) 0x00000046)
#define IRQ_FIQ_SRC__ILLEGAL_ACCESS_T2 ((u32) 0x00000047)
#define IRQ_FIQ_ENABLE_OFS (7)
#define IRQ_FIQ_ENABLE ((u32) 0x00000080)

/* Enable IRQs 1 (W) */
#define IRQ_IER1_OFS (0)
#define IRQ_IER1_MASK ((u32) 0xFFFFFFFF)
#define IRQ_IER10 ((u32) 0x00000001)
#define IRQ_IER11 ((u32) 0x00000002)
#define IRQ_IER12 ((u32) 0x00000004)
#define IRQ_IER13 ((u32) 0x00000008)
#define IRQ_IER14 ((u32) 0x00000010)
#define IRQ_IER15 ((u32) 0x00000020)
#define IRQ_IER16 ((u32) 0x00000040)
#define IRQ_IER17 ((u32) 0x00000080)
#define IRQ_IER18 ((u32) 0x00000100)
#define IRQ_IER19 ((u32) 0x00000200)
#define IRQ_IER110 ((u32) 0x00000400)
#define IRQ_IER111 ((u32) 0x00000800)
#define IRQ_IER112 ((u32) 0x00001000)
#define IRQ_IER113 ((u32) 0x00002000)
#define IRQ_IER114 ((u32) 0x00004000)
#define IRQ_IER115 ((u32) 0x00008000)
#define IRQ_IER116 ((u32) 0x00010000)
#define IRQ_IER117 ((u32) 0x00020000)
#define IRQ_IER118 ((u32) 0x00040000)
#define IRQ_IER119 ((u32) 0x00080000)
#define IRQ_IER120 ((u32) 0x00100000)
#define IRQ_IER121 ((u32) 0x00200000)
#define IRQ_IER122 ((u32) 0x00400000)
#define IRQ_IER123 ((u32) 0x00800000)
#define IRQ_IER124 ((u32) 0x01000000)
#define IRQ_IER125 ((u32) 0x02000000)
#define IRQ_IER126 ((u32) 0x04000000)
#define IRQ_IER127 ((u32) 0x08000000)
#define IRQ_IER128 ((u32) 0x10000000)
#define IRQ_IER129 ((u32) 0x20000000)
#define IRQ_IER130 ((u32) 0x40000000)
#define IRQ_IER131 ((u32) 0x80000000)
#define IRQ_IER1_0 ((u32) 0x00000001)
#define IRQ_IER1_1 ((u32) 0x00000002)
#define IRQ_IER1_2 ((u32) 0x00000004)
#define IRQ_IER1_3 ((u32) 0x00000008)
#define IRQ_IER1_4 ((u32) 0x00000010)
#define IRQ_IER1_5 ((u32) 0x00000020)
#define IRQ_IER1_6 ((u32) 0x00000040)
#define IRQ_IER1_7 ((u32) 0x00000080)
#define IRQ_IER1_8 ((u32) 0x00000100)
#define IRQ_IER1_9 ((u32) 0x00000200)
#define IRQ_IER1_10 ((u32) 0x00000400)
#define IRQ_IER1_11 ((u32) 0x00000800)
#define IRQ_IER1_12 ((u32) 0x00001000)
#define IRQ_IER1_13 ((u32) 0x00002000)
#define IRQ_IER1_14 ((u32) 0x00004000)
#define IRQ_IER1_15 ((u32) 0x00008000)
#define IRQ_IER1_16 ((u32) 0x00010000)
#define IRQ_IER1_17 ((u32) 0x00020000)
#define IRQ_IER1_18 ((u32) 0x00040000)
#define IRQ_IER1_19 ((u32) 0x00080000)
#define IRQ_IER1_20 ((u32) 0x00100000)
#define IRQ_IER1_21 ((u32) 0x00200000)
#define IRQ_IER1_22 ((u32) 0x00400000)
#define IRQ_IER1_23 ((u32) 0x00800000)
#define IRQ_IER1_24 ((u32) 0x01000000)
#define IRQ_IER1_25 ((u32) 0x02000000)
#define IRQ_IER1_26 ((u32) 0x04000000)
#define IRQ_IER1_27 ((u32) 0x08000000)
#define IRQ_IER1_28 ((u32) 0x10000000)
#define IRQ_IER1_29 ((u32) 0x20000000)
#define IRQ_IER1_30 ((u32) 0x40000000)
#define IRQ_IER1_31 ((u32) 0x80000000)
#define IRQ_IER1__SYSTMR_MATCH_1 ((u32) 0x00000001)
#define IRQ_IER1__SYSTMR_MATCH_3 ((u32) 0x00000008)
#define IRQ_IER1__USB_CONTROLLER ((u32) 0x00000200)
#define IRQ_IER1__AUX_INT ((u32) 0x20000000)
/* Enable IRQs 2 (W) */
#define IRQ_IER2_OFS (0)
#define IRQ_IER2_MASK ((u32) 0xFFFFFFFF)
#define IRQ_IER20 ((u32) 0x00000001)
#define IRQ_IER21 ((u32) 0x00000002)
#define IRQ_IER22 ((u32) 0x00000004)
#define IRQ_IER23 ((u32) 0x00000008)
#define IRQ_IER24 ((u32) 0x00000010)
#define IRQ_IER25 ((u32) 0x00000020)
#define IRQ_IER26 ((u32) 0x00000040)
#define IRQ_IER27 ((u32) 0x00000080)
#define IRQ_IER28 ((u32) 0x00000100)
#define IRQ_IER29 ((u32) 0x00000200)
#define IRQ_IER210 ((u32) 0x00000400)
#define IRQ_IER211 ((u32) 0x00000800)
#define IRQ_IER212 ((u32) 0x00001000)
#define IRQ_IER213 ((u32) 0x00002000)
#define IRQ_IER214 ((u32) 0x00004000)
#define IRQ_IER215 ((u32) 0x00008000)
#define IRQ_IER216 ((u32) 0x00010000)
#define IRQ_IER217 ((u32) 0x00020000)
#define IRQ_IER218 ((u32) 0x00040000)
#define IRQ_IER219 ((u32) 0x00080000)
#define IRQ_IER220 ((u32) 0x00100000)
#define IRQ_IER221 ((u32) 0x00200000)
#define IRQ_IER222 ((u32) 0x00400000)
#define IRQ_IER223 ((u32) 0x00800000)
#define IRQ_IER224 ((u32) 0x01000000)
#define IRQ_IER225 ((u32) 0x02000000)
#define IRQ_IER226 ((u32) 0x04000000)
#define IRQ_IER227 ((u32) 0x08000000)
#define IRQ_IER228 ((u32) 0x10000000)
#define IRQ_IER229 ((u32) 0x20000000)
#define IRQ_IER230 ((u32) 0x40000000)
#define IRQ_IER231 ((u32) 0x80000000)
#define IRQ_IER2_0 ((u32) 0x00000001)
#define IRQ_IER2_1 ((u32) 0x00000002)
#define IRQ_IER2_2 ((u32) 0x00000004)
#define IRQ_IER2_3 ((u32) 0x00000008)
#define IRQ_IER2_4 ((u32) 0x00000010)
#define IRQ_IER2_5 ((u32) 0x00000020)
#define IRQ_IER2_6 ((u32) 0x00000040)
#define IRQ_IER2_7 ((u32) 0x00000080)
#define IRQ_IER2_8 ((u32) 0x00000100)
#define IRQ_IER2_9 ((u32) 0x00000200)
#define IRQ_IER2_10 ((u32) 0x00000400)
#define IRQ_IER2_11 ((u32) 0x00000800)
#define IRQ_IER2_12 ((u32) 0x00001000)
#define IRQ_IER2_13 ((u32) 0x00002000)
#define IRQ_IER2_14 ((u32) 0x00004000)
#define IRQ_IER2_15 ((u32) 0x00008000)
#define IRQ_IER2_16 ((u32) 0x00010000)
#define IRQ_IER2_17 ((u32) 0x00020000)
#define IRQ_IER2_18 ((u32) 0x00040000)
#define IRQ_IER2_19 ((u32) 0x00080000)
#define IRQ_IER2_20 ((u32) 0x00100000)
#define IRQ_IER2_21 ((u32) 0x00200000)
#define IRQ_IER2_22 ((u32) 0x00400000)
#define IRQ_IER2_23 ((u32) 0x00800000)
#define IRQ_IER2_24 ((u32) 0x01000000)
#define IRQ_IER2_25 ((u32) 0x02000000)
#define IRQ_IER2_26 ((u32) 0x04000000)
#define IRQ_IER2_27 ((u32) 0x08000000)
#define IRQ_IER2_28 ((u32) 0x10000000)
#define IRQ_IER2_29 ((u32) 0x20000000)
#define IRQ_IER2_30 ((u32) 0x40000000)
#define IRQ_IER2_31 ((u32) 0x80000000)
#define IRQ_IER2__I2C_SPI_SLV_INT ((u32) 0x00000800)
#define IRQ_IER2__PWA0 ((u32) 0x00002000)
#define IRQ_IER2__PWA1 ((u32) 0x00004000)
#define IRQ_IER2__SMI ((u32) 0x00010000)
#define IRQ_IER2__GPIO_INT_0 ((u32) 0x00020000)
#define IRQ_IER2__GPIO_INT_1 ((u32) 0x00040000)
#define IRQ_IER2__GPIO_INT_2 ((u32) 0x00080000)
#define IRQ_IER2__GPIO_INT_3 ((u32) 0x00100000)
#define IRQ_IER2__I2C_INT ((u32) 0x00200000)
#define IRQ_IER2__SPI_INT ((u32) 0x00400000)
#define IRQ_IER2__PCM_INT ((u32) 0x00800000)
#define IRQ_IER2__UART_INT ((u32) 0x02000000)

/* Enable Basic IRQs (RW) */
#define IRQ_BASIC_IRQ_ENABLE_ARM_TIMER_OFS (0)
#define IRQ_BASIC_IRQ_ENABLE_ARM_TIMER ((u32) 0x00000001)
#define IRQ_BASIC_IRQ_ENABLE_ARM_MAILBOX_OFS (1)
#define IRQ_BASIC_IRQ_ENABLE_ARM_MAILBOX ((u32) 0x00000002)
#define IRQ_BASIC_IRQ_ENABLE_ARM_DOORBELL_0_OFS (2)
#define IRQ_BASIC_IRQ_ENABLE_ARM_DOORBELL_0 ((u32) 0x00000004)
#define IRQ_BASIC_IRQ_ENABLE_ARM_DOORBELL_1_OFS (3)
#define IRQ_BASIC_IRQ_ENABLE_ARM_DOORBELL_1 ((u32) 0x00000008)
#define IRQ_BASIC_IRQ_ENABLE_GPU0_HALTED_OFS (4)
#define IRQ_BASIC_IRQ_ENABLE_GPU0_HALTED ((u32) 0x00000010)
#define IRQ_BASIC_IRQ_ENABLE_GPU1_HALTED_OFS (5)
#define IRQ_BASIC_IRQ_ENABLE_GPU1_HALTED ((u32) 0x00000020)
#define IRQ_BASIC_IRQ_ENABLE_ACCESS_ERROR_T1_OFS (6)
#define IRQ_BASIC_IRQ_ENABLE_ACCESS_ERROR_T1 ((u32) 0x00000040)
#define IRQ_BASIC_IRQ_ENABLE_ACCESS_ERROR_T0_OFS (7)
#define IRQ_BASIC_IRQ_ENABLE_ACCESS_ERROR_T0 ((u32) 0x00000080)

/* Disable IRQs 1 */
#define IRQ_IDR1_OFS (0)
#define IRQ_IDR1_MASK ((u32) 0xFFFFFFFF)
#define IRQ_IDR10 ((u32) 0x00000001)
#define IRQ_IDR11 ((u32) 0x00000002)
#define IRQ_IDR12 ((u32) 0x00000004)
#define IRQ_IDR13 ((u32) 0x00000008)
#define IRQ_IDR14 ((u32) 0x00000010)
#define IRQ_IDR15 ((u32) 0x00000020)
#define IRQ_IDR16 ((u32) 0x00000040)
#define IRQ_IDR17 ((u32) 0x00000080)
#define IRQ_IDR18 ((u32) 0x00000100)
#define IRQ_IDR19 ((u32) 0x00000200)
#define IRQ_IDR110 ((u32) 0x00000400)
#define IRQ_IDR111 ((u32) 0x00000800)
#define IRQ_IDR112 ((u32) 0x00001000)
#define IRQ_IDR113 ((u32) 0x00002000)
#define IRQ_IDR114 ((u32) 0x00004000)
#define IRQ_IDR115 ((u32) 0x00008000)
#define IRQ_IDR116 ((u32) 0x00010000)
#define IRQ_IDR117 ((u32) 0x00020000)
#define IRQ_IDR118 ((u32) 0x00040000)
#define IRQ_IDR119 ((u32) 0x00080000)
#define IRQ_IDR120 ((u32) 0x00100000)
#define IRQ_IDR121 ((u32) 0x00200000)
#define IRQ_IDR122 ((u32) 0x00400000)
#define IRQ_IDR123 ((u32) 0x00800000)
#define IRQ_IDR124 ((u32) 0x01000000)
#define IRQ_IDR125 ((u32) 0x02000000)
#define IRQ_IDR126 ((u32) 0x04000000)
#define IRQ_IDR127 ((u32) 0x08000000)
#define IRQ_IDR128 ((u32) 0x10000000)
#define IRQ_IDR129 ((u32) 0x20000000)
#define IRQ_IDR130 ((u32) 0x40000000)
#define IRQ_IDR131 ((u32) 0x80000000)
#define IRQ_IDR1_0 ((u32) 0x00000001)
#define IRQ_IDR1_1 ((u32) 0x00000002)
#define IRQ_IDR1_2 ((u32) 0x00000004)
#define IRQ_IDR1_3 ((u32) 0x00000008)
#define IRQ_IDR1_4 ((u32) 0x00000010)
#define IRQ_IDR1_5 ((u32) 0x00000020)
#define IRQ_IDR1_6 ((u32) 0x00000040)
#define IRQ_IDR1_7 ((u32) 0x00000080)
#define IRQ_IDR1_8 ((u32) 0x00000100)
#define IRQ_IDR1_9 ((u32) 0x00000200)
#define IRQ_IDR1_10 ((u32) 0x00000400)
#define IRQ_IDR1_11 ((u32) 0x00000800)
#define IRQ_IDR1_12 ((u32) 0x00001000)
#define IRQ_IDR1_13 ((u32) 0x00002000)
#define IRQ_IDR1_14 ((u32) 0x00004000)
#define IRQ_IDR1_15 ((u32) 0x00008000)
#define IRQ_IDR1_16 ((u32) 0x00010000)
#define IRQ_IDR1_17 ((u32) 0x00020000)
#define IRQ_IDR1_18 ((u32) 0x00040000)
#define IRQ_IDR1_19 ((u32) 0x00080000)
#define IRQ_IDR1_20 ((u32) 0x00100000)
#define IRQ_IDR1_21 ((u32) 0x00200000)
#define IRQ_IDR1_22 ((u32) 0x00400000)
#define IRQ_IDR1_23 ((u32) 0x00800000)
#define IRQ_IDR1_24 ((u32) 0x01000000)
#define IRQ_IDR1_25 ((u32) 0x02000000)
#define IRQ_IDR1_26 ((u32) 0x04000000)
#define IRQ_IDR1_27 ((u32) 0x08000000)
#define IRQ_IDR1_28 ((u32) 0x10000000)
#define IRQ_IDR1_29 ((u32) 0x20000000)
#define IRQ_IDR1_30 ((u32) 0x40000000)
#define IRQ_IDR1_31 ((u32) 0x80000000)
#define IRQ_IDR1__SYSTMR_MATCH_1 ((u32) 0x00000001)
#define IRQ_IDR1__SYSTMR_MATCH_3 ((u32) 0x00000008)
#define IRQ_IDR1__USB_CONTROLLER ((u32) 0x00000200)
#define IRQ_IDR1__AUX_INT ((u32) 0x20000000)

/* Disable IRQs 2 (RW) */
#define IRQ_IDR2_OFS (0)
#define IRQ_IDR2_MASK ((u32) 0xFFFFFFFF)
#define IRQ_IDR20 ((u32) 0x00000001)
#define IRQ_IDR21 ((u32) 0x00000002)
#define IRQ_IDR22 ((u32) 0x00000004)
#define IRQ_IDR23 ((u32) 0x00000008)
#define IRQ_IDR24 ((u32) 0x00000010)
#define IRQ_IDR25 ((u32) 0x00000020)
#define IRQ_IDR26 ((u32) 0x00000040)
#define IRQ_IDR27 ((u32) 0x00000080)
#define IRQ_IDR28 ((u32) 0x00000100)
#define IRQ_IDR29 ((u32) 0x00000200)
#define IRQ_IDR210 ((u32) 0x00000400)
#define IRQ_IDR211 ((u32) 0x00000800)
#define IRQ_IDR212 ((u32) 0x00001000)
#define IRQ_IDR213 ((u32) 0x00002000)
#define IRQ_IDR214 ((u32) 0x00004000)
#define IRQ_IDR215 ((u32) 0x00008000)
#define IRQ_IDR216 ((u32) 0x00010000)
#define IRQ_IDR217 ((u32) 0x00020000)
#define IRQ_IDR218 ((u32) 0x00040000)
#define IRQ_IDR219 ((u32) 0x00080000)
#define IRQ_IDR220 ((u32) 0x00100000)
#define IRQ_IDR221 ((u32) 0x00200000)
#define IRQ_IDR222 ((u32) 0x00400000)
#define IRQ_IDR223 ((u32) 0x00800000)
#define IRQ_IDR224 ((u32) 0x01000000)
#define IRQ_IDR225 ((u32) 0x02000000)
#define IRQ_IDR226 ((u32) 0x04000000)
#define IRQ_IDR227 ((u32) 0x08000000)
#define IRQ_IDR228 ((u32) 0x10000000)
#define IRQ_IDR229 ((u32) 0x20000000)
#define IRQ_IDR230 ((u32) 0x40000000)
#define IRQ_IDR231 ((u32) 0x80000000)
#define IRQ_IDR2_0 ((u32) 0x00000001)
#define IRQ_IDR2_1 ((u32) 0x00000002)
#define IRQ_IDR2_2 ((u32) 0x00000004)
#define IRQ_IDR2_3 ((u32) 0x00000008)
#define IRQ_IDR2_4 ((u32) 0x00000010)
#define IRQ_IDR2_5 ((u32) 0x00000020)
#define IRQ_IDR2_6 ((u32) 0x00000040)
#define IRQ_IDR2_7 ((u32) 0x00000080)
#define IRQ_IDR2_8 ((u32) 0x00000100)
#define IRQ_IDR2_9 ((u32) 0x00000200)
#define IRQ_IDR2_10 ((u32) 0x00000400)
#define IRQ_IDR2_11 ((u32) 0x00000800)
#define IRQ_IDR2_12 ((u32) 0x00001000)
#define IRQ_IDR2_13 ((u32) 0x00002000)
#define IRQ_IDR2_14 ((u32) 0x00004000)
#define IRQ_IDR2_15 ((u32) 0x00008000)
#define IRQ_IDR2_16 ((u32) 0x00010000)
#define IRQ_IDR2_17 ((u32) 0x00020000)
#define IRQ_IDR2_18 ((u32) 0x00040000)
#define IRQ_IDR2_19 ((u32) 0x00080000)
#define IRQ_IDR2_20 ((u32) 0x00100000)
#define IRQ_IDR2_21 ((u32) 0x00200000)
#define IRQ_IDR2_22 ((u32) 0x00400000)
#define IRQ_IDR2_23 ((u32) 0x00800000)
#define IRQ_IDR2_24 ((u32) 0x01000000)
#define IRQ_IDR2_25 ((u32) 0x02000000)
#define IRQ_IDR2_26 ((u32) 0x04000000)
#define IRQ_IDR2_27 ((u32) 0x08000000)
#define IRQ_IDR2_28 ((u32) 0x10000000)
#define IRQ_IDR2_29 ((u32) 0x20000000)
#define IRQ_IDR2_30 ((u32) 0x40000000)
#define IRQ_IDR2_31 ((u32) 0x80000000)
#define IRQ_IDR2__I2C_SPI_SLV_INT ((u32) 0x00000800)
#define IRQ_IDR2__PWA0 ((u32) 0x00002000)
#define IRQ_IDR2__PWA1 ((u32) 0x00004000)
#define IRQ_IDR2__SMI ((u32) 0x00010000)
#define IRQ_IDR2__GPIO_INT_0 ((u32) 0x00020000)
#define IRQ_IDR2__GPIO_INT_1 ((u32) 0x00040000)
#define IRQ_IDR2__GPIO_INT_2 ((u32) 0x00080000)
#define IRQ_IDR2__GPIO_INT_3 ((u32) 0x00100000)
#define IRQ_IDR2__I2C_INT ((u32) 0x00200000)
#define IRQ_IDR2__SPI_INT ((u32) 0x00400000)
#define IRQ_IDR2__PCM_INT ((u32) 0x00800000)
#define IRQ_IDR2__UART_INT ((u32) 0x02000000)
/* Disable Basic IRQs (RW) */
#define IRQ_BASIC_IRQ_DISABLE_ARM_TIMER_OFS (0)
#define IRQ_BASIC_IRQ_DISABLE_ARM_TIMER ((u32) 0x00000001)
#define IRQ_BASIC_IRQ_DISABLE_ARM_MAILBOX_OFS (1)
#define IRQ_BASIC_IRQ_DISABLE_ARM_MAILBOX ((u32) 0x00000002)
#define IRQ_BASIC_IRQ_DISABLE_ARM_DOORBELL_0_OFS (2)
#define IRQ_BASIC_IRQ_DISABLE_ARM_DOORBELL_0 ((u32) 0x00000004)
#define IRQ_BASIC_IRQ_DISABLE_ARM_DOORBELL_1_OFS (3)
#define IRQ_BASIC_IRQ_DISABLE_ARM_DOORBELL_1 ((u32) 0x00000008)
#define IRQ_BASIC_IRQ_DISABLE_GPU0_HALTED_OFS (4)
#define IRQ_BASIC_IRQ_DISABLE_GPU0_HALTED ((u32) 0x00000010)
#define IRQ_BASIC_IRQ_DISABLE_GPU1_HALTED_OFS (5)
#define IRQ_BASIC_IRQ_DISABLE_GPU1_HALTED ((u32) 0x00000020)
#define IRQ_BASIC_IRQ_DISABLE_ACCESS_ERROR_T1_OFS (6)
#define IRQ_BASIC_IRQ_DISABLE_ACCESS_ERROR_T1 ((u32) 0x00000040)
#define IRQ_BASIC_IRQ_DISABLE_ACCESS_ERROR_T0_OFS (7)
#define IRQ_BASIC_IRQ_DISABLE_ACCESS_ERROR_T0 ((u32) 0x00000080)

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
#define SYSTIMER_CS_M0 ((u32) 0x00000001)
#define SYSTIMER_CS_M1_OFS (1)
#define SYSTIMER_CS_M1 ((u32) 0x00000002)
#define SYSTIMER_CS_M2_OFS (2)
#define SYSTIMER_CS_M2 ((u32) 0x00000004)
#define SYSTIMER_CS_M3_OFS (3)
#define SYSTIMER_CS_M3 ((u32) 0x00000008)
/* CLO - System Timer Counter Lower 32 bits (R) */
#define SYSTIMER_CLO_CNT_OFS (0)
#define SYSTIMER_CLO_CNT_MASK ((u32) 0xFFFFFFFF)
/* CHI - System Timer Counter Higher 32 bits (R) */
#define SYSTIMER_CHI_CNT_OFS (0)
#define SYSTIMER_CHI_CNT_MASK ((u32) 0xFFFFFFFF)
/* C0 - System Timer Compare 0 */
#define SYSTIMER_C0_CMP_OFS (0)
#define SYSTIMER_C0_CMP_MASK ((u32) 0xFFFFFFFF)
/* C1 - System Timer Compare 1 */
#define SYSTIMER_C1_CMP_OFS (0)
#define SYSTIMER_C1_CMP_MASK ((u32) 0xFFFFFFFF)
/* C2 - System Timer Compare 2 */
#define SYSTIMER_C2_CMP_OFS (0)
#define SYSTIMER_C2_CMP_MASK ((u32) 0xFFFFFFFF)
/* C3 - System Timer Compare 3 */
#define SYSTIMER_C3_CMP_OFS (0)
#define SYSTIMER_C3_CMP_MASK ((u32) 0xFFFFFFFF)

/* Timer (ARM side) Register bits */
/* This Timer and free-running counter are running */
/* from the APB clock */
/* Load Register (W) */
#define ARMTIMER_LDR_OFS (0)
#define ARMTIMER_LDR_MASK ((u32) 0xFFFFFFFF)

/* Value Register (R) */
#define ARMTIMER_RDR_OFS (0)
#define ARMTIMER_RDR_MASK ((u32) 0xFFFFFFFF)

/* Control Register (RW) */
#define ARMTIMER_CTL_COUNTER_LEN_OFS (1)
#define ARMTIMER_CTL_COUNTER_LEN ((u32) 0x00000002)
#define ARMTIMER_CTL_PRESCALE_OFS (2)
#define ARMTIMER_CTL_PRESCALE_MASK ((u32) 0x0000000C)
#define ARMTIMER_CTL_PRESCALE0 ((u32) 0x00000004)
#define ARMTIMER_CTL_PRESCALE1 ((u32) 0x00000008)
#define ARMTIMER_CTL_PRESCALE_0 ((u32) 0x00000000)
#define ARMTIMER_CTL_PRESCALE_1 ((u32) 0x00000004)
#define ARMTIMER_CTL_PRESCALE_2 ((u32) 0x00000008)
#define ARMTIMER_CTL_PRESCALE_3 ((u32) 0x0000000C)
#define ARMTIMER_CTL_PRESCALE__NO_PRESCALE ((u32) 0x00000000)
#define ARMTIMER_CTL_PRESCALE__16 ((u32) 0x00000004)
#define ARMTIMER_CTL_PRESCALE__256 ((u32) 0x00000008)
/* Undefined in SP804 */
#define ARMTIMER_CTL_PRESCALE__NO_PRESCALE1 ((u32) 0x0000000C)
#define ARMTIMER_CTL_TIMER_IRQ_EN_OFS (5)
#define ARMTIMER_CTL_TIMER_IRQ_EN ((u32) 0x00000020)
#define ARMTIMER_CTL_TIMER_EN_OFS (7)
#define ARMTIMER_CTL_TIMER_EN ((u32) 0x00000080)
/* Undefined in SP804 */
#define ARMTIMER_CTL_DEBUG_HALTED_MODE_OFS (8)
#define ARMTIMER_CTL_DEBUG_HALTED_MODE ((u32) 0x00000100)
/* Undefined in SP804 */
#define ARMTIMER_CTL_FRCTR_EN_OFS (9)
#define ARMTIMER_CTL_FRCTR_EN ((u32) 0x00000200)
#define ARMTIMER_CTL_FRCTR_PRESCALE_OFS (16)
#define ARMTIMER_CTL_FRCTR_PRESCALE0 ((u32) 0x00010000)
#define ARMTIMER_CTL_FRCTR_PRESCALE1 ((u32) 0x00020000)
#define ARMTIMER_CTL_FRCTR_PRESCALE2 ((u32) 0x00040000)
#define ARMTIMER_CTL_FRCTR_PRESCALE3 ((u32) 0x00080000)
#define ARMTIMER_CTL_FRCTR_PRESCALE4 ((u32) 0x00100000)
#define ARMTIMER_CTL_FRCTR_PRESCALE5 ((u32) 0x00200000)
#define ARMTIMER_CTL_FRCTR_PRESCALE6 ((u32) 0x00400000)
#define ARMTIMER_CTL_FRCTR_PRESCALE7 ((u32) 0x00800000)
/* IRQ Clear/Ack Register (W) */
#define ARMTIMER_IRQ_CLR_OFS (0)
#define ARMTIMER_IRQ_CLR_MASK ((u32) 0xFFFFFFFF)
/* RAW IRQ Register (R) */
#define ARMTIMER_IRQ_RAW_STATUS_OFS (0)
#define ARMTIMER_IRQ_RAW_STATUS ((u32) 0x00000001)
/* Masked IRQ Register (R) */
#define ARMTIMER_IRQ_MASKED_STATUS_OFS (0)
#define ARMTIMER_IRQ_MASKED_STATUS ((u32) 0x00000001)
/* Reload Regster (W) */
#define ARMTIMER_RLDR_OFS (0)
#define ARMTIMER_RLDR_MASK ((u32) 0xFFFFFFFF)
/* Pre-divider Register (RW) */
#define ARMTIMER_PRE_DIVIDER_OFS (0)
#define ARMTIMER_PRE_DIVIDER_MASK ((u32) 0x000003FF)
/* 32-bit Free-running counter Register (R) */
#define ARMTIMER_FRCTR_RDR_OFS (0)
#define ARMTIMER_FRCTR_RDR_MASK ((u32) 0xFFFFFFFF)
/* BSC Bits (I2C single master only operation) */
/* EMMC Bits (External Mass Media Controller) */
/* PCM/I2S Audio Bits */
/* SPI/BSC Slave Bits */
/* USB Bits */

/* ARM Control Logic Module Registers */
/* 0x40000000: Control Register (RW) */
#define ARMCTL_CTL_CORETIMER_CLK_SRC_OFS (8)
#define ARMCTL_CTL_CORETIMER_CLK_SRC ((u32) 0x00000100)
#define ARMCTL_CTL_CORETIMER_INCREMENTS_OFS (9)
#define ARMCTL_CTL_CORETIMER_INCREMENTS ((u32) 0x00000200)
/* 0x40000008: Core Timer pre-scaler (RW) */
#define ARMCTL_CORETIMER_PRESCALER_OFS (0)
#define ARMCTL_CORETIMER_PRESCALER_MASK ((u32) 0xFFFFFFFF)
/* 0x4000001C: Core Timer read: LS 32 bits */
/* Write: LS-32 holding register */
#define ARMCTL_CORETIMER_BITS_LO_OFS (0)
#define ARMCTL_CORETIMER_BITS_LO_MASK ((u32) 0xFFFFFFFF)
/* 0x40000020: Core Timer read: Stored MS 32 bits register */
/* Write: MS 32 bits */
#define ARMCTL_CORETIMER_BITS_HI_OFS (0)
#define ARMCTL_CORETIMER_BITS_HI_MASK ((u32) 0xFFFFFFFF)
/* 0x4000000C: GPU interrupt routing */
#define ARMCTL_GPU_IRQ_ROUTING_OFS (0)
#define ARMCTL_GPU_IRQ_ROUTING_MASK ((u32) 0x00000003)
#define ARMCTL_GPU_IRQ_ROUTING0 ((u32) 0x00000001)
#define ARMCTL_GPU_IRQ_ROUTING1 ((u32) 0x00000002)
#define ARMCTL_GPU_IRQ_ROUTING_0 ((u32) 0x00000000)
#define ARMCTL_GPU_IRQ_ROUTING_1 ((u32) 0x00000001)
#define ARMCTL_GPU_IRQ_ROUTING_2 ((u32) 0x00000002)
#define ARMCTL_GPU_IRQ_ROUTING_3 ((u32) 0x00000003)
#define ARMCTL_GPU_IRQ_ROUTING__CORE0_IRQ_INPUT ((u32) 0x00000000)
#define ARMCTL_GPU_IRQ_ROUTING__CORE1_IRQ_INPUT ((u32) 0x00000001)
#define ARMCTL_GPU_IRQ_ROUTING__CORE2_IRQ_INPUT ((u32) 0x00000002)
#define ARMCTL_GPU_IRQ_ROUTING__CORE3_IRQ_INPUT ((u32) 0x00000003)
#define ARMCTL_GPU_FIQ_ROUTING_OFS (2)
#define ARMCTL_GPU_FIQ_ROUTING_MASK ((u32) 0x0000000C)
#define ARMCTL_GPU_FIQ_ROUTING0 ((u32) 0x00000004)
#define ARMCTL_GPU_FIQ_ROUTING1 ((u32) 0x00000008)
#define ARMCTL_GPU_FIQ_ROUTING_0 ((u32) 0x00000000)
#define ARMCTL_GPU_FIQ_ROUTING_1 ((u32) 0x00000004)
#define ARMCTL_GPU_FIQ_ROUTING_2 ((u32) 0x00000008)
#define ARMCTL_GPU_FIQ_ROUTING_3 ((u32) 0x0000000C)
#define ARMCTL_GPU_FIQ_ROUTING__CORE0_FIQ_INPUT ((u32) 0x00000000)
#define ARMCTL_GPU_FIQ_ROUTING__CORE1_FIQ_INPUT ((u32) 0x00000004)
#define ARMCTL_GPU_FIQ_ROUTING__CORE2_FIQ_INPUT ((u32) 0x00000008)
#define ARMCTL_GPU_FIQ_ROUTING__CORE3_FIQ_INPUT ((u32) 0x0000000C)
/* 0x40000010 Performance monitor interrupt routing write-set */
/* Register */
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ0CTL_OFS (0)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ0CTL ((u32) 0x00000001)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ1CTL_OFS (1)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ1CTL ((u32) 0x00000002)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ2CTL_OFS (2)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ2CTL ((u32) 0x00000004)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ3CTL_OFS (3)
#define ARMCTL_PMU_IRQ_ROUTING_SET_IRQ3CTL ((u32) 0x00000008)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ0CTL_OFS (4)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ0CTL ((u32) 0x00000010)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ1CTL_OFS (5)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ1CTL ((u32) 0x00000020)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ2CTL_OFS (6)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ2CTL ((u32) 0x00000040)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ3CTL_OFS (7)
#define ARMCTL_PMU_IRQ_ROUTING_SET_FIQ3CTL ((u32) 0x00000080)
/* 0x40000014 Performance monitor interrupt routing write-clear */
/* Register */
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ0CTL_OFS (0)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ0CTL ((u32) 0x00000001)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ1CTL_OFS (1)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ1CTL ((u32) 0x00000002)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ2CTL_OFS (2)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ2CTL ((u32) 0x00000004)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ3CTL_OFS (3)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_IRQ3CTL ((u32) 0x00000008)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ0CTL_OFS (4)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ0CTL ((u32) 0x00000010)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ1CTL_OFS (5)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ1CTL ((u32) 0x00000020)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ2CTL_OFS (6)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ2CTL ((u32) 0x00000040)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ3CTL_OFS (7)
#define ARMCTL_PMU_IRQ_ROUTING_CLEAR_FIQ3CTL ((u32) 0x00000080)
/* 0x40000040 Core0 Timers interrupt control */
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPS_IRQ_OFS (0)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPS_IRQ ((u32) 0x00000001)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPNS_IRQ_OFS (1)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPNS_IRQ ((u32) 0x00000002)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_THP_IRQ_OFS (2)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_THP_IRQ ((u32) 0x00000004)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TV_IRQ_OFS (3)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TV_IRQ ((u32) 0x00000008)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPS_FIQ_OFS (4)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPS_FIQ ((u32) 0x00000010)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPNS_FIQ_OFS (5)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TPNS_FIQ ((u32) 0x00000020)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_THP_FIQ_OFS (6)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_THP_FIQ ((u32) 0x00000040)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TV_FIQ_OFS (7)
#define ARMCTL_CORE0_TIMERS_IRQ_CTL_TV_FIQ ((u32) 0x00000080)
/* 0x40000044 Core1 Timers interrupt control */
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPS_IRQ_OFS (0)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPS_IRQ ((u32) 0x00000001)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPNS_IRQ_OFS (1)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPNS_IRQ ((u32) 0x00000002)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_THP_IRQ_OFS (2)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_THP_IRQ ((u32) 0x00000004)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TV_IRQ_OFS (3)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TV_IRQ ((u32) 0x00000008)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPS_FIQ_OFS (4)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPS_FIQ ((u32) 0x00000010)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPNS_FIQ_OFS (5)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TPNS_FIQ ((u32) 0x00000020)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_THP_FIQ_OFS (6)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_THP_FIQ ((u32) 0x00000040)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TV_FIQ_OFS (7)
#define ARMCTL_CORE1_TIMERS_IRQ_CTL_TV_FIQ ((u32) 0x00000080)
/* 0x40000048 Core2 Timers interrupt control */
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPS_IRQ_OFS (0)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPS_IRQ ((u32) 0x00000001)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPNS_IRQ_OFS (1)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPNS_IRQ ((u32) 0x00000002)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_THP_IRQ_OFS (2)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_THP_IRQ ((u32) 0x00000004)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TV_IRQ_OFS (3)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TV_IRQ ((u32) 0x00000008)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPS_FIQ_OFS (4)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPS_FIQ ((u32) 0x00000010)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPNS_FIQ_OFS (5)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TPNS_FIQ ((u32) 0x00000020)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_THP_FIQ_OFS (6)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_THP_FIQ ((u32) 0x00000040)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TV_FIQ_OFS (7)
#define ARMCTL_CORE2_TIMERS_IRQ_CTL_TV_FIQ ((u32) 0x00000080)
/* 0x4000004C Core3 Timers interrupt control */
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPS_IRQ_OFS (0)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPS_IRQ ((u32) 0x00000001)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPNS_IRQ_OFS (1)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPNS_IRQ ((u32) 0x00000002)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_THP_IRQ_OFS (2)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_THP_IRQ ((u32) 0x00000004)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TV_IRQ_OFS (3)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TV_IRQ ((u32) 0x00000008)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPS_FIQ_OFS (4)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPS_FIQ ((u32) 0x00000010)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPNS_FIQ_OFS (5)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TPNS_FIQ ((u32) 0x00000020)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_THP_FIQ_OFS (6)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_THP_FIQ ((u32) 0x00000040)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TV_FIQ_OFS (7)
#define ARMCTL_CORE3_TIMERS_IRQ_CTL_TV_FIQ ((u32) 0x00000080)
/* 0x40000050 Core0 Mailboxes interrupt control */
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB0IRQ_OFS (0)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB0IRQ ((u32) 0x00000001)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB1IRQ_OFS (1)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB1IRQ ((u32) 0x00000002)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB2IRQ_OFS (2)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB2IRQ ((u32) 0x00000004)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB3IRQ_OFS (3)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB3IRQ ((u32) 0x00000008)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB0FIQ_OFS (4)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB0FIQ ((u32) 0x00000010)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB1FIQ_OFS (5)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB1FIQ ((u32) 0x00000020)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB2FIQ_OFS (6)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB2FIQ ((u32) 0x00000040)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB3FIQ_OFS (7)
#define ARMCTL_CORE0_MAILBOXES_IRQ_CTL_MB3FIQ ((u32) 0x00000080)
/* 0x40000054 Core1 Mailboxes interrupt control */
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB0IRQ_OFS (0)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB0IRQ ((u32) 0x00000001)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB1IRQ_OFS (1)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB1IRQ ((u32) 0x00000002)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB2IRQ_OFS (2)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB2IRQ ((u32) 0x00000004)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB3IRQ_OFS (3)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB3IRQ ((u32) 0x00000008)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB0FIQ_OFS (4)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB0FIQ ((u32) 0x00000010)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB1FIQ_OFS (5)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB1FIQ ((u32) 0x00000020)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB2FIQ_OFS (6)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB2FIQ ((u32) 0x00000040)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB3FIQ_OFS (7)
#define ARMCTL_CORE1_MAILBOXES_IRQ_CTL_MB3FIQ ((u32) 0x00000080)
/* 0x40000058 Core2 Mailboxes interrupt control */
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB0IRQ_OFS (0)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB0IRQ ((u32) 0x00000001)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB1IRQ_OFS (1)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB1IRQ ((u32) 0x00000002)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB2IRQ_OFS (2)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB2IRQ ((u32) 0x00000004)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB3IRQ_OFS (3)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB3IRQ ((u32) 0x00000008)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB0FIQ_OFS (4)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB0FIQ ((u32) 0x00000010)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB1FIQ_OFS (5)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB1FIQ ((u32) 0x00000020)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB2FIQ_OFS (6)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB2FIQ ((u32) 0x00000040)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB3FIQ_OFS (7)
#define ARMCTL_CORE2_MAILBOXES_IRQ_CTL_MB3FIQ ((u32) 0x00000080)
/* 0x4000005C Core3 Mailboxes interrupt control */
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB0IRQ_OFS (0)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB0IRQ ((u32) 0x00000001)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB1IRQ_OFS (1)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB1IRQ ((u32) 0x00000002)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB2IRQ_OFS (2)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB2IRQ ((u32) 0x00000004)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB3IRQ_OFS (3)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB3IRQ ((u32) 0x00000008)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB0FIQ_OFS (4)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB0FIQ ((u32) 0x00000010)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB1FIQ_OFS (5)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB1FIQ ((u32) 0x00000020)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB2FIQ_OFS (6)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB2FIQ ((u32) 0x00000040)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB3FIQ_OFS (7)
#define ARMCTL_CORE3_MAILBOXES_IRQ_CTL_MB3FIQ ((u32) 0x00000080)
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
#define ARMCTL_AXI_OUTSTANDING_COUNTERS_READS_MASK ((u32) 0x000003FF)
#define ARMCTL_AXI_OUTSTANDING_COUNTERS_WRITES_OFS (16)
#define ARMCTL_AXI_OUTSTANDING_COUNTERS_WRITES_MASK ((u32) 0x03FF0000)
/* 0x40000030 AXI outstanding interrupt */
#define ARMCTL_AXI_OUTSTANDING_IRQ_TIMEOUT_OFS (0)
#define ARMCTL_AXI_OUTSTANDING_IRQ_TIMEOUT_MASK ((u32) 0x000FFFFF)
#define ARMCTL_AXI_OUTSTANDING_IRQ_ENABLE_OFS (20)
#define ARMCTL_AXI_OUTSTANDING_IRQ_ENABLE ((u32) 0x00100000)
/* 0x40000060 Core0 interrupt source */
#define ARMCTL_CORE0_IRQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE0_IRQ_SRC_TPSIRQ ((u32) 0x00000001)
#define ARMCTL_CORE0_IRQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE0_IRQ_SRC_TPNSIRQ ((u32) 0x00000002)
#define ARMCTL_CORE0_IRQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE0_IRQ_SRC_THPIRQ ((u32) 0x00000004)
#define ARMCTL_CORE0_IRQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE0_IRQ_SRC_TVIRQ ((u32) 0x00000008)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX0 ((u32) 0x00000010)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX1 ((u32) 0x00000020)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX2 ((u32) 0x00000040)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE0_IRQ_SRC_MAILBOX3 ((u32) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE0_IRQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE0_IRQ_SRC_GPU ((u32) 0x00000100)
#define ARMCTL_CORE0_IRQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE0_IRQ_SRC_PMU ((u32) 0x00000200)
/* AXI IRQ for core 0 only, all others are 0 */
#define ARMCTL_CORE0_IRQ_SRC_AXI_OFS (10)
#define ARMCTL_CORE0_IRQ_SRC_AXI ((u32) 0x00000400)
#define ARMCTL_CORE0_IRQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE0_IRQ_SRC_LOCALTIMER ((u32) 0x00000800)
#define ARMCTL_CORE0_IRQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE0_IRQ_SRC_PERIPH ((u32) 0x00001000)

/* 0x40000064 Core1 interrupt source */
#define ARMCTL_CORE1_IRQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE1_IRQ_SRC_TPSIRQ ((u32) 0x00000001)
#define ARMCTL_CORE1_IRQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE1_IRQ_SRC_TPNSIRQ ((u32) 0x00000002)
#define ARMCTL_CORE1_IRQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE1_IRQ_SRC_THPIRQ ((u32) 0x00000004)
#define ARMCTL_CORE1_IRQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE1_IRQ_SRC_TVIRQ ((u32) 0x00000008)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX0 ((u32) 0x00000010)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX1 ((u32) 0x00000020)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX2 ((u32) 0x00000040)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE1_IRQ_SRC_MAILBOX3 ((u32) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE1_IRQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE1_IRQ_SRC_GPU ((u32) 0x00000100)
#define ARMCTL_CORE1_IRQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE1_IRQ_SRC_PMU ((u32) 0x00000200)
/* AXI IRQ for core 0 only, all others are 0 */
#define ARMCTL_CORE1_IRQ_SRC_AXI_OFS (10)
#define ARMCTL_CORE1_IRQ_SRC_AXI ((u32) 0x00000400)
#define ARMCTL_CORE1_IRQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE1_IRQ_SRC_LOCALTIMER ((u32) 0x00000800)
#define ARMCTL_CORE1_IRQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE1_IRQ_SRC_PERIPH ((u32) 0x00001000)

/* 0x40000068 Core2 interrupt source */
#define ARMCTL_CORE2_IRQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE2_IRQ_SRC_TPSIRQ ((u32) 0x00000001)
#define ARMCTL_CORE2_IRQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE2_IRQ_SRC_TPNSIRQ ((u32) 0x00000002)
#define ARMCTL_CORE2_IRQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE2_IRQ_SRC_THPIRQ ((u32) 0x00000004)
#define ARMCTL_CORE2_IRQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE2_IRQ_SRC_TVIRQ ((u32) 0x00000008)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX0 ((u32) 0x00000010)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX1 ((u32) 0x00000020)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX2 ((u32) 0x00000040)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE2_IRQ_SRC_MAILBOX3 ((u32) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE2_IRQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE2_IRQ_SRC_GPU ((u32) 0x00000100)
#define ARMCTL_CORE2_IRQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE2_IRQ_SRC_PMU ((u32) 0x00000200)
/* AXI IRQ for core 0 only, all others are 0 */
#define ARMCTL_CORE2_IRQ_SRC_AXI_OFS (10)
#define ARMCTL_CORE2_IRQ_SRC_AXI ((u32) 0x00000400)
#define ARMCTL_CORE2_IRQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE2_IRQ_SRC_LOCALTIMER ((u32) 0x00000800)
#define ARMCTL_CORE2_IRQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE2_IRQ_SRC_PERIPH ((u32) 0x00001000)

/* 0x4000006C Core3 interrupt source */
#define ARMCTL_CORE3_IRQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE3_IRQ_SRC_TPSIRQ ((u32) 0x00000001)
#define ARMCTL_CORE3_IRQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE3_IRQ_SRC_TPNSIRQ ((u32) 0x00000002)
#define ARMCTL_CORE3_IRQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE3_IRQ_SRC_THPIRQ ((u32) 0x00000004)
#define ARMCTL_CORE3_IRQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE3_IRQ_SRC_TVIRQ ((u32) 0x00000008)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX0 ((u32) 0x00000010)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX1 ((u32) 0x00000020)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX2 ((u32) 0x00000040)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE3_IRQ_SRC_MAILBOX3 ((u32) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE3_IRQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE3_IRQ_SRC_GPU ((u32) 0x00000100)
#define ARMCTL_CORE3_IRQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE3_IRQ_SRC_PMU ((u32) 0x00000200)
/* AXI IRQ for core 0 only, all others are 0 */
#define ARMCTL_CORE3_IRQ_SRC_AXI_OFS (10)
#define ARMCTL_CORE3_IRQ_SRC_AXI ((u32) 0x00000400)
#define ARMCTL_CORE3_IRQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE3_IRQ_SRC_LOCALTIMER ((u32) 0x00000800)
#define ARMCTL_CORE3_IRQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE3_IRQ_SRC_PERIPH ((u32) 0x00001000)

/* 0x40000070 Core0 fast interrupt source */
#define ARMCTL_CORE0_FIQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE0_FIQ_SRC_TPSIRQ ((u32) 0x00000001)
#define ARMCTL_CORE0_FIQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE0_FIQ_SRC_TPNSIRQ ((u32) 0x00000002)
#define ARMCTL_CORE0_FIQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE0_FIQ_SRC_THPIRQ ((u32) 0x00000004)
#define ARMCTL_CORE0_FIQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE0_FIQ_SRC_TVIRQ ((u32) 0x00000008)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX0 ((u32) 0x00000010)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX1 ((u32) 0x00000020)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX2 ((u32) 0x00000040)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE0_FIQ_SRC_MAILBOX3 ((u32) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE0_FIQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE0_FIQ_SRC_GPU ((u32) 0x00000100)
#define ARMCTL_CORE0_FIQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE0_FIQ_SRC_PMU ((u32) 0x00000200)
#define ARMCTL_CORE0_FIQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE0_FIQ_SRC_LOCALTIMER ((u32) 0x00000800)
#define ARMCTL_CORE0_FIQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE0_FIQ_SRC_PERIPH ((u32) 0x00001000)

/* 0x40000074 Core1 fast interrupt source */
#define ARMCTL_CORE1_FIQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE1_FIQ_SRC_TPSIRQ ((u32) 0x00000001)
#define ARMCTL_CORE1_FIQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE1_FIQ_SRC_TPNSIRQ ((u32) 0x00000002)
#define ARMCTL_CORE1_FIQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE1_FIQ_SRC_THPIRQ ((u32) 0x00000004)
#define ARMCTL_CORE1_FIQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE1_FIQ_SRC_TVIRQ ((u32) 0x00000008)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX0 ((u32) 0x00000010)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX1 ((u32) 0x00000020)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX2 ((u32) 0x00000040)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE1_FIQ_SRC_MAILBOX3 ((u32) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE1_FIQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE1_FIQ_SRC_GPU ((u32) 0x00000100)
#define ARMCTL_CORE1_FIQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE1_FIQ_SRC_PMU ((u32) 0x00000200)
#define ARMCTL_CORE1_FIQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE1_FIQ_SRC_LOCALTIMER ((u32) 0x00000800)
#define ARMCTL_CORE1_FIQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE1_FIQ_SRC_PERIPH ((u32) 0x00001000)

/* 0x40000078 Core2 fast interrupt source */
#define ARMCTL_CORE2_FIQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE2_FIQ_SRC_TPSIRQ ((u32) 0x00000001)
#define ARMCTL_CORE2_FIQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE2_FIQ_SRC_TPNSIRQ ((u32) 0x00000002)
#define ARMCTL_CORE2_FIQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE2_FIQ_SRC_THPIRQ ((u32) 0x00000004)
#define ARMCTL_CORE2_FIQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE2_FIQ_SRC_TVIRQ ((u32) 0x00000008)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX0 ((u32) 0x00000010)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX1 ((u32) 0x00000020)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX2 ((u32) 0x00000040)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE2_FIQ_SRC_MAILBOX3 ((u32) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE2_FIQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE2_FIQ_SRC_GPU ((u32) 0x00000100)
#define ARMCTL_CORE2_FIQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE2_FIQ_SRC_PMU ((u32) 0x00000200)
#define ARMCTL_CORE2_FIQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE2_FIQ_SRC_LOCALTIMER ((u32) 0x00000800)
#define ARMCTL_CORE2_FIQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE2_FIQ_SRC_PERIPH ((u32) 0x00001000)

/* 0x4000007C Core3 fast interrupt source */
#define ARMCTL_CORE3_FIQ_SRC_TPSIRQ_OFS (0)
#define ARMCTL_CORE3_FIQ_SRC_TPSIRQ ((u32) 0x00000001)
#define ARMCTL_CORE3_FIQ_SRC_TPNSIRQ_OFS (1)
#define ARMCTL_CORE3_FIQ_SRC_TPNSIRQ ((u32) 0x00000002)
#define ARMCTL_CORE3_FIQ_SRC_THPIRQ_OFS (2)
#define ARMCTL_CORE3_FIQ_SRC_THPIRQ ((u32) 0x00000004)
#define ARMCTL_CORE3_FIQ_SRC_TVIRQ_OFS (3)
#define ARMCTL_CORE3_FIQ_SRC_TVIRQ ((u32) 0x00000008)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX0_OFS (4)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX0 ((u32) 0x00000010)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX1_OFS (5)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX1 ((u32) 0x00000020)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX2_OFS (6)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX2 ((u32) 0x00000040)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX3_OFS (7)
#define ARMCTL_CORE3_FIQ_SRC_MAILBOX3 ((u32) 0x00000080)
/* GPU IRQ Can be high in one core only */
#define ARMCTL_CORE3_FIQ_SRC_GPU_OFS (8)
#define ARMCTL_CORE3_FIQ_SRC_GPU ((u32) 0x00000100)
#define ARMCTL_CORE3_FIQ_SRC_PMU_OFS (9)
#define ARMCTL_CORE3_FIQ_SRC_PMU ((u32) 0x00000200)
#define ARMCTL_CORE3_FIQ_SRC_LOCALTIMER_OFS (11)
#define ARMCTL_CORE3_FIQ_SRC_LOCALTIMER ((u32) 0x00000800)
#define ARMCTL_CORE3_FIQ_SRC_PERIPH_OFS (12)
/* Currently not used */
#define ARMCTL_CORE3_FIQ_SRC_PERIPH ((u32) 0x00001000)

/* 0x40000034 Local Timer Control & Status */
#define ARMCTL_LTCSR_RELOADVAL_OFS (0)
#define ARMCTL_LTCSR_RELOADVAL_MASK ((u32) 0x0FFFFFFF)
#define ARMCTL_LTCSR_TIMER_ENABLE_OFS (28)
#define ARMCTL_LTCSR_TIMER_ENABLE ((u32) 0x10000000)
#define ARMCTL_LTCSR_IRQ_ENABLE_OFS (29)
#define ARMCTL_LTCSR_IRQ_ENABLE ((u32) 0x20000000)
#define ARMCTL_LTCSR_IRQ_FLAG_OFS (31)
#define ARMCTL_LTCSR_IRQ_FLAG ((u32) 0x80000000)
/* 0x40000038 Local Timer IRQ Clear & Reload (W) */
#define ARMCTL_LTWFR_RELOAD_OFS (30)
#define ARMCTL_LTWFR_RELOAD ((u32) 0x40000000)
#define ARMCTL_LTWFR_IRQ_FLAG_CLR_OFS (31)
#define ARMCTL_LTWFR_IRQ_FLAG_CLR ((u32) 0x80000000)
/* 0x40000024 Local Interrupt Routing */
/* Local Timer is the only local interrupt source present */
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_OFS (0)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_MASK ((u32) 0x00000007)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST0 ((u32) 0x00000001)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST1 ((u32) 0x00000002)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST2 ((u32) 0x00000004)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_0 ((u32) 0x00000000)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_1 ((u32) 0x00000001)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_2 ((u32) 0x00000002)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_3 ((u32) 0x00000003)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_4 ((u32) 0x00000004)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_5 ((u32) 0x00000005)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_6 ((u32) 0x00000006)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST_7 ((u32) 0x00000007)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE0IRQ ((u32) 0x00000000)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE1IRQ ((u32) 0x00000001)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE2IRQ ((u32) 0x00000002)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE3IRQ ((u32) 0x00000003)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE0FIQ ((u32) 0x00000004)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE1FIQ ((u32) 0x00000005)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE2FIQ ((u32) 0x00000006)
#define ARMCTL_LOCAL_IRQ_ROUTING_DEST__CORE3FIQ ((u32) 0x00000007)

#endif /* RPI_H_ */
