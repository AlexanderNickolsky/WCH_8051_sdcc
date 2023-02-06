/*--------------------------------------------------------------------------
CH554.H
Header file for CH554 microcontrollers.
****************************************
**  Copyright  (C)  W.ch  1999-2014   **
**  Web:              http://wch.cn   **
****************************************
--------------------------------------------------------------------------*/

#ifndef __CH554_H__
#define __CH554_H__

#include <compiler.h>

/* SFR address table */

#define SPI0_STAT_ADDR  0xF8
#define SPI0_DATA_ADDR  0xF9
#define SPI0_CTRL_ADDR  0xFA
#define SPI0_CK_SE_ADDR 0xFB
#define SPI0_S_PRE_ADDR 0xFB
#define SPI0_SETUP_ADDR 0xFC
#define RESET_KEEP_ADDR 0xFE
#define WDOG_COUNT_ADDR 0xFF

#define B_ADDR          0xF0

#define IE_EX_ADDR      0xE8
#define IP_EX_ADDR      0xE9
#define UEP4_1_MOD_ADDR 0xEA
#define UEP2_3_MOD_ADDR 0xEB
#define UEP0_DMA_L_ADDR 0xEC
#define UEP0_DMA_H_ADDR 0xED
#define UEP1_DMA_L_ADDR 0xEE
#define UEP1_DMA_H_ADDR 0xEF

#define ACC_ADDR         0xE0
#define USB_INT_EN_ADDR  0xE1
#define USB_CTRL_ADDR    0xE2
#define USB_DEV_AD_ADDR  0xE3
#define UEP2_DMA_L_ADDR  0xE4
#define UEP2_DMA_H_ADDR  0xE5
#define UEP3_DMA_L_ADDR  0xE6
#define UEP3_DMA_H_ADDR  0xE7

#define USB_INT_FG_ADDR 0xD8
#define USB_INT_ST_ADDR 0xD9
#define USB_MIS_ST_ADDR 0xDA
#define USB_RX_LEN_ADDR 0xDB
#define UEP0_CTRL_ADDR  0xDC
#define UEP0_T_LEN_ADDR 0xDD
#define UEP4_CTRL_ADDR  0xDE
#define UEP4_T_LEN_ADDR 0xDF

#define PSW_ADDR        0xD0
#define UDEV_CTRL_ADDR  0xD1
#define UEP1_CTRL_ADDR  0xD2
#define UEP1_T_LEN_ADDR 0xD3
#define UEP2_CTRL_ADDR  0xD4
#define UEP2_T_LEN_ADDR 0xD5
#define UEP3_CTRL_ADDR  0xD6
#define UEP3_T_LEN_ADDR 0xD7

#define T2CON_ADDR      0xC8
#define T2MOD_ADDR      0xC9
#define RCAP2L_ADDR     0xCA
#define RCAP2H_ADDR     0xCB
#define TL2_ADDR        0xCC
#define TH2_ADDR        0xCD
#define T2CAP1L_ADDR    0xCE
#define T2CAP1H_ADDR    0xCF

#define SCON1_ADDR      0xC0
#define SBUF1_ADDR      0xC1
#define SBAUD1_ADDR     0xC2
#define TKEY_CTRL_ADDR  0xC3
#define TKEY_DATL_ADDR  0xC4
#define TKEY_DATH_ADDR  0xC5
#define PIN_FUNC_ADDR   0xC6
#define GPIO_IE_ADDR    0xC7

#define IP_ADDR         0xB8
#define CLOCK_CFG_ADDR  0xB9

#define P3_ADDR         0xB0
#define GLOBAL_CFG_ADDR 0xB1

#define IE_ADDR         0xA8
#define WAKE_CTRL_ADDR  0xA9

#define P2_ADDR         0xA0
#define SAFE_MOD_ADDR   0xA1
#define CHIP_ID_ADDR    0xA1
#define XBUS_AUX_ADDR   0xA2

#define SCON_ADDR       0x98
#define SBUF_ADDR       0x99
#define ADC_CFG_ADDR    0x9A
#define PWM_DATA2_ADDR  0x9B
#define PWM_DATA1_ADDR  0x9C
#define PWM_CTRL_ADDR   0x9D
#define PWM_CK_SE_ADDR  0x9E
#define ADC_DATA_ADDR   0x9F

#define P1_ADDR         0x90
#define USB_C_CTRL_ADDR 0x91
#define P1_MOD_OC_ADDR  0x92
#define P1_DIR_PU_ADDR  0x93
#define P3_MOD_OC_ADDR  0x96
#define P3_DIR_PU_ADDR  0x97

#define TCON_ADDR           0x88
#define TMOD_ADDR           0x89
#define TL0_ADDR            0x8A
#define TL1_ADDR            0x8B
#define TH0_ADDR            0x8C
#define TH1_ADDR            0x8D
#define ROM_DATA_L_ADDR     0x8E
#define ROM_DATA_H_ADDR     0x8F

#define ADC_CTRL_ADDR       0x80
#define SP_ADDR             0x81
#define DPL_ADDR            0x82
#define DPH_ADDR            0x83
#define ROM_ADDR_L_ADDR     0x84
#define ROM_ADDR_H_ADDR     0x85
#define ROM_CTRL_ADDR       0x86
#define ROM_STATUS_ADDR     0x86
#define PCON_ADDR           0x87


/*----- SFR --------------------------------------------------------------*/
/*  sbit are bit addressable, others are byte addressable */

/*  System Registers  */
SFR(PSW,	PSW_ADDR);	// program status word
   SBIT(CY,	PSW_ADDR, 7);	// carry flag
   SBIT(AC,	PSW_ADDR, 6);	// auxiliary carry flag
   SBIT(F0,	PSW_ADDR, 5);	// bit addressable general purpose flag 0
   SBIT(RS1,	PSW_ADDR, 4);	// register R0-R7 bank selection high bit
   SBIT(RS0,	PSW_ADDR, 3);	// register R0-R7 bank selection low bit
#define MASK_PSW_RS       0x18      // bit mask of register R0-R7 bank selection
// RS1 & RS0: register R0-R7 bank selection
//    00 - bank 0, R0-R7 @ address 0x00-0x07
//    01 - bank 1, R0-R7 @ address 0x08-0x0F
//    10 - bank 2, R0-R7 @ address 0x10-0x17
//    11 - bank 3, R0-R7 @ address 0x18-0x1F
   SBIT(OV,	PSW_ADDR, 2);	// overflow flag
   SBIT(F1,	PSW_ADDR, 1);	// bit addressable general purpose flag 1
   SBIT(P,	PSW_ADDR, 0);	// ReadOnly: parity flag
SFR(ACC,	ACC_ADDR);	// accumulator
SFR(B,	B_ADDR);	// general purpose register B
SFR(SP,	SP_ADDR);	// stack pointer
//sfr16 DPTR          = 0x82;         // DPTR pointer, little-endian
SFR(DPL,	DPL_ADDR);	// data pointer low
SFR(DPH,	DPH_ADDR);	// data pointer high
SFR(SAFE_MOD,	SAFE_MOD_ADDR);	// WriteOnly: writing safe mode
SFR(CHIP_ID,	CHIP_ID_ADDR);
SFR(GLOBAL_CFG,	GLOBAL_CFG_ADDR);	// global config, Write@SafeMode
#define bBOOT_LOAD        0x20      // ReadOnly: boot loader status for discriminating BootLoader or Application: set 1 by power on reset, clear 0 by software reset
#define bSW_RESET         0x10      // software reset bit, auto clear by hardware
#define bCODE_WE          0x08      // enable flash-ROM (include code & Data-Flash) being program or erasing: 0=writing protect, 1=enable program and erase
#define bDATA_WE          0x04      // enable Data-Flash (flash-ROM data area) being program or erasing: 0=writing protect, 1=enable program and erase
#define bLDO3V3_OFF       0x02      // disable 5V->3.3V LDO: 0=enable LDO for USB and internal oscillator under 5V power, 1=disable LDO, V33 pin input external 3.3V power
#define bWDOG_EN          0x01      // enable watch-dog reset if watch-dog timer overflow: 0=as timer only, 1=enable reset if timer overflow

/* Clock and Sleep and Power Registers */
SFR(PCON,	PCON_ADDR);	// power control and reset flag
#define SMOD              0x80      // baud rate selection for UART0 mode 1/2/3: 0=slow(Fsys/128 @mode2, TF1/32 @mode1/3, no effect for TF2),
                                    //   1=fast(Fsys/32 @mode2, TF1/16 @mode1/3, no effect for TF2)
#define bRST_FLAG1        0x20      // ReadOnly: recent reset flag high bit
#define bRST_FLAG0        0x10      // ReadOnly: recent reset flag low bit
#define MASK_RST_FLAG     0x30      // ReadOnly: bit mask of recent reset flag
#define RST_FLAG_SW       0x00
#define RST_FLAG_POR      0x10
#define RST_FLAG_WDOG     0x20
#define RST_FLAG_PIN      0x30
// bPC_RST_FLAG1 & bPC_RST_FLAG0: recent reset flag
//    00 - software reset, by bSW_RESET=1 @(bBOOT_LOAD=0 or bWDOG_EN=1)
//    01 - power on reset
//    10 - watch-dog timer overflow reset
//    11 - external input manual reset by RST pin
#define GF1               0x08      // general purpose flag bit 1
#define GF0               0x04      // general purpose flag bit 0
#define PD                0x02      // power-down enable bit, auto clear by wake-up hardware
SFR(CLOCK_CFG,	CLOCK_CFG_ADDR);	// system clock config: lower 3 bits for system clock Fsys, Write@SafeMode
#define bOSC_EN_INT       0x80      // internal oscillator enable and original clock selection: 1=enable & select internal clock, 0=disable & select external clock
#define bOSC_EN_XT        0x40      // external oscillator enable, need quartz crystal or ceramic resonator between XI and XO pins
#define bWDOG_IF_TO       0x20      // ReadOnly: watch-dog timer overflow interrupt flag, cleared by reload watch-dog count or auto cleared when MCU enter interrupt routine
#define bROM_CLK_FAST     0x10      // flash-ROM clock frequency selection: 0=normal(for Fosc>=16MHz), 1=fast(for Fosc<16MHz)
#define bRST              0x08      // ReadOnly: pin RST input
#define bT2EX_            0x08      // alternate pin for T2EX
#define bCAP2_            0x08      // alternate pin for CAP2
#define MASK_SYS_CK_SEL   0x07      // bit mask of system clock Fsys selection
/*
   Fxt = 24MHz(8MHz~25MHz for non-USB application), from external oscillator @XI&XO
   Fosc = bOSC_EN_INT ? 24MHz : Fxt
   Fpll = Fosc * 4 => 96MHz (32MHz~100MHz for non-USB application)
   Fusb4x = Fpll / 2 => 48MHz (Fixed)
              MASK_SYS_CK_SEL[2] [1] [0]
   Fsys = Fpll/3   =  32MHz:  1   1   1
   Fsys = Fpll/4   =  24MHz:  1   1   0
   Fsys = Fpll/6   =  16MHz:  1   0   1
   Fsys = Fpll/8   =  12MHz:  1   0   0
   Fsys = Fpll/16  =   6MHz:  0   1   1
   Fsys = Fpll/32  =   3MHz:  0   1   0
   Fsys = Fpll/128 = 750KHz:  0   0   1
   Fsys = Fpll/512 =187.5KHz: 0   0   0
*/
SFR(WAKE_CTRL,	WAKE_CTRL_ADDR);	// wake-up control, Write@SafeMode
#define bWAK_BY_USB       0x80      // enable wake-up by USB event
#define bWAK_RXD1_LO      0x40      // enable wake-up by RXD1 low level
#define bWAK_P1_5_LO      0x20      // enable wake-up by pin P1.5 low level
#define bWAK_P1_4_LO      0x10      // enable wake-up by pin P1.4 low level
#define bWAK_P1_3_LO      0x08      // enable wake-up by pin P1.3 low level
#define bWAK_RST_HI       0x04      // enable wake-up by pin RST high level
#define bWAK_P3_2E_3L     0x02      // enable wake-up by pin P3.2 (INT0) edge or pin P3.3 (INT1) low level
#define bWAK_RXD0_LO      0x01      // enable wake-up by RXD0 low level
SFR(RESET_KEEP,	RESET_KEEP_ADDR);	// value keeper during reset
SFR(WDOG_COUNT,	WDOG_COUNT_ADDR);	// watch-dog count, count by clock frequency Fsys/65536

/*  Interrupt Registers  */
SFR(IE,	IE_ADDR);	// interrupt enable
   SBIT(EA,	IE_ADDR, 7);	// enable global interrupts: 0=disable, 1=enable if E_DIS=0
   SBIT(E_DIS,	IE_ADDR, 6);	// disable global interrupts, intend to inhibit interrupt during some flash-ROM operation: 0=enable if EA=1, 1=disable
   SBIT(ET2,	IE_ADDR, 5);	// enable timer2 interrupt
   SBIT(ES,	    IE_ADDR, 4);	// enable UART0 interrupt
   SBIT(ET1,	IE_ADDR, 3);	// enable timer1 interrupt
   SBIT(EX1,	IE_ADDR, 2);	// enable external interrupt INT1
   SBIT(ET0,	IE_ADDR, 1);	// enable timer0 interrupt
   SBIT(EX0,	IE_ADDR, 0);	// enable external interrupt INT0
SFR(IP,	IP_ADDR);	// interrupt priority and current priority
   SBIT(PH_FLAG,	IP_ADDR, 7);	// ReadOnly: high level priority action flag
   SBIT(PL_FLAG,	IP_ADDR, 6);	// ReadOnly: low level priority action flag
// PH_FLAG & PL_FLAG: current interrupt priority
//    00 - no interrupt now
//    01 - low level priority interrupt action now
//    10 - high level priority interrupt action now
//    11 - unknown error
   SBIT(PT2,	IP_ADDR, 5);	// timer2 interrupt priority level
   SBIT(PS,	    IP_ADDR, 4);	// UART0 interrupt priority level
   SBIT(PT1,	IP_ADDR, 3);	// timer1 interrupt priority level
   SBIT(PX1,	IP_ADDR, 2);	// external interrupt INT1 priority level
   SBIT(PT0,	IP_ADDR, 1);	// timer0 interrupt priority level
   SBIT(PX0,	IP_ADDR, 0);	// external interrupt INT0 priority level

SFR(IE_EX,	IE_EX_ADDR);	// extend interrupt enable
   SBIT(IE_WDOG,	IE_EX_ADDR, 7);	// enable watch-dog timer interrupt
   SBIT(IE_GPIO,	IE_EX_ADDR, 6);	// enable GPIO input interrupt
   SBIT(IE_PWMX,	IE_EX_ADDR, 5);	// enable PWM1/2 interrupt
   SBIT(IE_UART1,	IE_EX_ADDR, 4);	// enable UART1 interrupt
   SBIT(IE_ADC,	    IE_EX_ADDR, 3);	// enable ADC interrupt
   SBIT(IE_USB,	    IE_EX_ADDR, 2);	// enable USB interrupt
   SBIT(IE_TKEY,	IE_EX_ADDR, 1);	// enable touch-key timer interrupt
   SBIT(IE_SPI0,	IE_EX_ADDR, 0);	// enable SPI0 interrupt

SFR(IP_EX,	IP_EX_ADDR);	// extend interrupt priority
#define bIP_LEVEL         0x80      // ReadOnly: current interrupt nested level: 0=no interrupt or two levels, 1=one level
#define bIP_GPIO          0x40      // GPIO input interrupt priority level
#define bIP_PWMX          0x20      // PWM1/2 interrupt priority level
#define bIP_UART1         0x10      // UART1 interrupt priority level
#define bIP_ADC           0x08      // ADC interrupt priority level
#define bIP_USB           0x04      // USB interrupt priority level
#define bIP_TKEY          0x02      // touch-key timer interrupt priority level
#define bIP_SPI0          0x01      // SPI0 interrupt priority level

SFR(GPIO_IE,	GPIO_IE_ADDR);	// GPIO interrupt enable
#define bIE_IO_EDGE       0x80      // enable GPIO edge interrupt: 0=low/high level, 1=falling/rising edge
#define bIE_RXD1_LO       0x40      // enable interrupt by RXD1 low level / falling edge
#define bIE_P1_5_LO       0x20      // enable interrupt by pin P1.5 low level / falling edge
#define bIE_P1_4_LO       0x10      // enable interrupt by pin P1.4 low level / falling edge
#define bIE_P1_3_LO       0x08      // enable interrupt by pin P1.3 low level / falling edge
#define bIE_RST_HI        0x04      // enable interrupt by pin RST high level / rising edge
#define bIE_P3_1_LO       0x02      // enable interrupt by pin P3.1 low level / falling edge
#define bIE_RXD0_LO       0x01      // enable interrupt by RXD0 low level / falling edge

/*  FlashROM and Data-Flash Registers  */
SFR16(ROM_ADDR,	ROM_ADDR_L_ADDR);	// address for flash-ROM, little-endian
SFR(ROM_ADDR_L,	ROM_ADDR_L_ADDR);	// address low byte for flash-ROM
SFR(ROM_ADDR_H,	ROM_ADDR_H_ADDR);	// address high byte for flash-ROM
SFR16(ROM_DATA,	ROM_DATA_L_ADDR);	// data for flash-ROM writing, little-endian
SFR(ROM_DATA_L,	ROM_DATA_L_ADDR);	// data low byte for flash-ROM writing, data byte for Data-Flash reading/writing
SFR(ROM_DATA_H,	ROM_DATA_H_ADDR);	// data high byte for flash-ROM writing
SFR(ROM_CTRL,	ROM_CTRL_ADDR);	// WriteOnly: flash-ROM control
#define ROM_CMD_WRITE     0x9A      // WriteOnly: flash-ROM word or Data-Flash byte write operation command
#define ROM_CMD_READ      0x8E      // WriteOnly: Data-Flash byte read operation command
//sfr ROM_STATUS      = 0x86;         // ReadOnly: flash-ROM status
#define ROM_STATUS        ROM_CTRL
#define bROM_ADDR_OK      0x40      // ReadOnly: flash-ROM writing operation address valid flag, can be reviewed before or after operation: 0=invalid parameter, 1=address valid
#define bROM_CMD_ERR      0x02      // ReadOnly: flash-ROM operation command error flag: 0=command accepted, 1=unknown command

/*  Port Registers  */
SFR(P1,	P1_ADDR);	// port 1 input & output
   SBIT(P1_7,   P1_ADDR, 7);    // SCK/bSCK, TXD1/bTXD1, TIN5, P1.7
   SBIT(SCK,	P1_ADDR, 7);	// serial clock for SPI0
   #define bSCK       0x80
   SBIT(TXD1,	P1_ADDR, 7);	// TXD output for UART1
   #define bTXD1      0x80
   SBIT(TIN5,	P1_ADDR, 7);	// TIN5 for Touch-Key

   SBIT(P1_6,   P1_ADDR, 6);    // MISO/bMISO, RXD1/bRXD1, TIN4, P1.6
   SBIT(MISO,	P1_ADDR, 6);	// master serial data input or slave serial data output for SPI0
   #define bMISO      0x40
   SBIT(RXD1,	P1_ADDR, 6);	// RXD input for UART1
   #define bRXD1      0x40
   SBIT(TIN4,	P1_ADDR, 6);	// TIN4 for Touch-Key

   SBIT(P1_5,   P1_ADDR, 5);    // MOSI/bMOSI, PWM1/bPWM1, TIN3, UCC2, AIN2, P1.5
   SBIT(MOSI,	P1_ADDR, 5);	// master serial data output or slave serial data input for SPI0
   #define bMOSI      0x20
   SBIT(PWM1,	P1_ADDR, 5);	// PWM output for PWM1
   #define bPWM1      0x20
   SBIT(TIN3,	P1_ADDR, 5);	// TIN3 for Touch-Key
   SBIT(UCC2,	P1_ADDR, 5);	// CC2 for USB type-C
   #define bUCC2      0x20
   SBIT(AIN2,	P1_ADDR, 5);	// AIN2 for ADC
   #define bAIN2      0x20

   SBIT(P1_4,   P1_ADDR, 4);    // T2_/bT2_, CAP1_/bCAP1_, SCS/bSCS, TIN2, UCC1, AIN1, P1.4
   SBIT(T2_,	P1_ADDR, 4);	// alternate pin for T2
   #define bT2_       0x10
   SBIT(CAP1_,	P1_ADDR, 4);	// alternate pin for CAP1
   #define bCAP1_     0x10
   SBIT(SCS,	P1_ADDR, 4);	// slave chip-selection input for SPI0
   #define bSCS       0x10
   SBIT(TIN2,	P1_ADDR, 4);	// TIN2 for Touch-Key
   SBIT(UCC1,	P1_ADDR, 4);	// CC1 for USB type-C
   #define bUCC1      0x10
   SBIT(AIN1,	P1_ADDR, 4);	// AIN1 for ADC
   #define bAIN1      0x10

   SBIT(P1_3,   P1_ADDR, 3);    // XO, TXD_/bTXD_, P1.3
   SBIT(TXD_,	P1_ADDR, 3);	// alternate pin for TXD of UART0
   #define bTXD_      0x08

   SBIT(P1_2,   P1_ADDR, 2);    // XI, RXD_/bRXD_, P1.2
   SBIT(RXD_,	P1_ADDR, 2);	// alternate pin for RXD of UART0
   #define bRXD_      0x04

   SBIT(P1_1,   P1_ADDR, 1);    // T2EX/bT2EX, CAP2/bCAP2, TIN1, VBUS2, AIN0, P1.1
   SBIT(T2EX,	P1_ADDR, 1);	// external trigger input for timer2 reload & capture
   #define bT2EX      0x02
   SBIT(CAP2,	P1_ADDR, 1);	// capture2 input for timer2
   #define bCAP2      0x02
   SBIT(TIN1,	P1_ADDR, 1);	// TIN1 for Touch-Key
   SBIT(VBUS2,	P1_ADDR, 1);	// VBUS2 for USB type-C
   #define bVBUS2     0x02
   SBIT(AIN0,	P1_ADDR, 1);	// AIN0 for ADC
   #define bAIN0      0x02

   SBIT(P1_0,   P1_ADDR, 0);    //T2/bT2, CAP1/bCAP1, TIN0, P1.0
   SBIT(T2,	    P1_ADDR, 0);	// external count input or clock output for timer2
   #define bT2        0x01
   SBIT(CAP1,	P1_ADDR, 0);	// capture1 input for timer2
   #define bCAP1      0x01
   SBIT(TIN0,	P1_ADDR, 0);	// TIN0 for Touch-Key

SFR(P1_MOD_OC,	P1_MOD_OC_ADDR);	// port 1 output mode: 0=push-pull, 1=open-drain
SFR(P1_DIR_PU,	P1_DIR_PU_ADDR);	// port 1 direction for push-pull or pullup enable for open-drain
// Pn_MOD_OC & Pn_DIR_PU: pin input & output configuration for Pn (n=1/3)
//   0 0:  float input only, without pullup resistance
//   0 1:  push-pull output, strong driving high level and low level
//   1 0:  open-drain output and input without pullup resistance
//   1 1:  quasi-bidirectional (standard 8051 mode), open-drain output and input with pullup resistance, just driving high level strongly for 2 clocks if turning output level from low to high

SFR(P2,	P2_ADDR);	// port 2 is the internal port only

SFR(P3,	P3_ADDR);	// port 3 input & output
   SBIT(UDM,	P3_ADDR, 7);	// ReadOnly: pin UDM input
   #define bUDM              0x80
   SBIT(UDP,	P3_ADDR, 6);	// ReadOnly: pin UDP input
   #define bUDP              0x40

   SBIT(P3_5,	P3_ADDR, 5);    // T1/bT1, P3.5
   SBIT(T1,	    P3_ADDR, 5);	// external count input for timer1
   #define bT1        0x20

   SBIT(P3_4,	P3_ADDR, 4);    // PWM2/bPWM2, RXD1_/bRXD1_, T0/bT0, P3.4
   SBIT(PWM2,	P3_ADDR, 4);	// PWM output for PWM2
   #define bPWM2      0x10
   SBIT(RXD1_,	P3_ADDR, 4);	// alternate pin for RXD1
   #define bRXD1_     0x10
   SBIT(T0,	    P3_ADDR, 4);	// external count input for timer0
   #define bT0        0x10

   SBIT(P3_3,	P3_ADDR, 3);    // INT1/bINT1, P3.3
   SBIT(INT1,	P3_ADDR, 3);	// external interrupt 1 input
   #define bINT1      0x08

   SBIT(P3_2,	P3_ADDR, 2);    // TXD1_/bTXD1_, INT0/bINT0, VBUS1, AIN3, P3.2
   SBIT(TXD1_,	P3_ADDR, 2);	// alternate pin for TXD1
   #define bTXD1_     0x04
   SBIT(INT0,	P3_ADDR, 2);	// external interrupt 0 input
   #define bINT0      0x04
   SBIT(VBUS1,	P3_ADDR, 2);	// VBUS1 for USB type-C
   #define bVBUS1     0x04
   SBIT(AIN3,	P3_ADDR, 2);	// AIN3 for ADC
   #define bAIN3      0x04

   SBIT(P3_1,	P3_ADDR, 1);    // PWM2_/bPWM2_, TXD/bTXD, P3.1
   SBIT(PWM2_,	P3_ADDR, 1);	// alternate pin for PWM2
   #define bPWM2_     0x02
   SBIT(TXD,	P3_ADDR, 1);	// TXD output for UART0
   #define bTXD       0x02

   SBIT(P3_0,	P3_ADDR, 0);    // PWM1_/bPWM1_, RXD/bRXD, P3.0
   SBIT(PWM1_,	P3_ADDR, 0);	// alternate pin for PWM1
   #define bPWM1_     0x01
   SBIT(RXD,	P3_ADDR, 0);	// RXD input for UART0
   #define bRXD       0x01

SFR(P3_MOD_OC,	P3_MOD_OC_ADDR);	// port 3 output mode: 0=push-pull, 1=open-drain
SFR(P3_DIR_PU,	P3_DIR_PU_ADDR);	// port 3 direction for push-pull or pullup enable for open-drain



SFR(PIN_FUNC,	PIN_FUNC_ADDR);	// pin function selection
#define bUSB_IO_EN        0x80      // USB UDP/UDM I/O pin enable: 0=P3.6/P3.7 as GPIO, 1=P3.6/P3.7 as USB
#define bIO_INT_ACT       0x40      // ReadOnly: GPIO interrupt request action status
#define bUART1_PIN_X      0x20      // UART1 alternate pin enable: 0=RXD1/TXD1 on P1.6/P1.7, 1=RXD1/TXD1 on P3.4/P3.2
#define bUART0_PIN_X      0x10      // UART0 alternate pin enable: 0=RXD0/TXD0 on P3.0/P3.1, 1=RXD0/TXD0 on P1.2/P1.3
#define bPWM2_PIN_X       0x08      // PWM2 alternate pin enable: 0=PWM2 on P3.4, 1=PWM2 on P3.1
#define bPWM1_PIN_X       0x04      // PWM1 alternate pin enable: 0=PWM1 on P1.5, 1=PWM1 on P3.0
#define bT2EX_PIN_X       0x02      // T2EX/CAP2 alternate pin enable: 0=T2EX/CAP2 on P1.1, 1=T2EX/CAP2 on RST
#define bT2_PIN_X         0x01      // T2/CAP1 alternate pin enable: 0=T2/CAP1 on P1.1, 1=T2/CAP1 on P1.4
SFR(XBUS_AUX,	XBUS_AUX_ADDR);	// xBUS auxiliary setting
#define bUART0_TX         0x80      // ReadOnly: indicate UART0 transmittal status
#define bUART0_RX         0x40      // ReadOnly: indicate UART0 receiving status
#define bSAFE_MOD_ACT     0x20      // ReadOnly: safe mode action status
#define GF2               0x08      // general purpose flag bit 2
#define bDPTR_AUTO_INC    0x04      // enable DPTR auto increase if finished MOVX_@DPTR instruction
#define DPS               0x01      // dual DPTR selection: 0=DPTR0 selected, 1=DPTR1 selected

/*  Timer0/1 Registers  */
SFR(TCON,	TCON_ADDR);	// timer 0/1 control and external interrupt control
   SBIT(TF1,	TCON_ADDR, 7);	// timer1 overflow & interrupt flag, auto cleared when MCU enter interrupt routine
   SBIT(TR1,	TCON_ADDR, 6);	// timer1 run enable
   SBIT(TF0,	TCON_ADDR, 5);	// timer0 overflow & interrupt flag, auto cleared when MCU enter interrupt routine
   SBIT(TR0,	TCON_ADDR, 4);	// timer0 run enable
   SBIT(IE1,	TCON_ADDR, 3);	// INT1 interrupt flag, auto cleared when MCU enter interrupt routine
   SBIT(IT1,	TCON_ADDR, 2);	// INT1 interrupt type: 0=low level action, 1=falling edge action
   SBIT(IE0,	TCON_ADDR, 1);	// INT0 interrupt flag, auto cleared when MCU enter interrupt routine
   SBIT(IT0,	TCON_ADDR, 0);	// INT0 interrupt type: 0=low level action, 1=falling edge action
SFR(TMOD,	TMOD_ADDR);	// timer 0/1 mode
#define bT1_GATE          0x80      // gate control of timer1: 0=timer1 run enable while TR1=1, 1=timer1 run enable while P3.3 (INT1) pin is high and TR1=1
#define bT1_CT            0x40      // counter or timer mode selection for timer1: 0=timer, use internal clock, 1=counter, use P3.5 (T1) pin falling edge as clock
#define bT1_M1            0x20      // timer1 mode high bit
#define bT1_M0            0x10      // timer1 mode low bit
#define MASK_T1_MOD       0x30      // bit mask of timer1 mode
// bT1_M1 & bT1_M0: timer1 mode
//   00: mode 0, 13-bit timer or counter by cascaded TH1 and lower 5 bits of TL1, the upper 3 bits of TL1 are ignored
//   01: mode 1, 16-bit timer or counter by cascaded TH1 and TL1
//   10: mode 2, TL1 operates as 8-bit timer or counter, and TH1 provide initial value for TL1 auto-reload
//   11: mode 3, stop timer1
#define bT0_GATE          0x08      // gate control of timer0: 0=timer0 run enable while TR0=1, 1=timer0 run enable while P3.2 (INT0) pin is high and TR0=1
#define bT0_CT            0x04      // counter or timer mode selection for timer0: 0=timer, use internal clock, 1=counter, use P3.4 (T0) pin falling edge as clock
#define bT0_M1            0x02      // timer0 mode high bit
#define bT0_M0            0x01      // timer0 mode low bit
#define MASK_T0_MOD       0x03      // bit mask of timer0 mode
// bT0_M1 & bT0_M0: timer0 mode
//   00: mode 0, 13-bit timer or counter by cascaded TH0 and lower 5 bits of TL0, the upper 3 bits of TL0 are ignored
//   01: mode 1, 16-bit timer or counter by cascaded TH0 and TL0
//   10: mode 2, TL0 operates as 8-bit timer or counter, and TH0 provide initial value for TL0 auto-reload
//   11: mode 3, TL0 is 8-bit timer or counter controlled by standard timer0 bits, TH0 is 8-bit timer using TF1 and controlled by TR1, timer1 run enable if it is not mode 3
SFR(TL0,	TL0_ADDR);	// low byte of timer 0 count
SFR(TL1,	TL1_ADDR);	// low byte of timer 1 count
SFR(TH0,	TH0_ADDR);	// high byte of timer 0 count
SFR(TH1,	TH1_ADDR);	// high byte of timer 1 count

/*  UART0 Registers  */
SFR(SCON,	SCON_ADDR);	// UART0 control (serial port control)
   SBIT(SM0,	SCON_ADDR, 7);	// UART0 mode bit0, selection data bit: 0=8 bits data, 1=9 bits data
   SBIT(SM1,	SCON_ADDR, 6);	// UART0 mode bit1, selection baud rate: 0=fixed, 1=variable
// SM0 & SM1: UART0 mode
//    00 - mode 0, shift Register, baud rate fixed at: Fsys/12
//    01 - mode 1, 8-bit UART,     baud rate = variable by timer1 or timer2 overflow rate
//    10 - mode 2, 9-bit UART,     baud rate fixed at: Fsys/128@SMOD=0, Fsys/32@SMOD=1
//    11 - mode 3, 9-bit UART,     baud rate = variable by timer1 or timer2 overflow rate
   SBIT(SM2,	SCON_ADDR, 5);	// enable multi-device communication in mode 2/3
#define MASK_UART0_MOD    0xE0      // bit mask of UART0 mode
   SBIT(REN,	SCON_ADDR, 4);	// enable UART0 receiving
   SBIT(TB8,	SCON_ADDR, 3);	// the 9th transmitted data bit in mode 2/3
   SBIT(RB8,	SCON_ADDR, 2);	// 9th data bit received in mode 2/3, or stop bit received for mode 1
   SBIT(TI,	    SCON_ADDR, 1);	// transmit interrupt flag, set by hardware after completion of a serial transmittal, need software clear
   SBIT(RI,	    SCON_ADDR, 0);	// receive interrupt flag, set by hardware after completion of a serial receiving, need software clear
SFR(SBUF,	SBUF_ADDR);	// UART0 data buffer: reading for receiving, writing for transmittal

/*  Timer2/Capture2 Registers  */
SFR(T2CON,	T2CON_ADDR);	// timer 2 control
   SBIT(TF2,	T2CON_ADDR, 7);	// timer2 overflow & interrupt flag, need software clear, the flag will not be set when either RCLK=1 or TCLK=1
   SBIT(CAP1F,	T2CON_ADDR, 7);	// timer2 capture 1 interrupt flag, set by T2 edge trigger if bT2_CAP1_EN=1, need software clear
   SBIT(EXF2,	T2CON_ADDR, 6);	// timer2 external flag, set by T2EX edge trigger if EXEN2=1, need software clear
   SBIT(RCLK,	T2CON_ADDR, 5);	// selection UART0 receiving clock: 0=timer1 overflow pulse, 1=timer2 overflow pulse
   SBIT(TCLK,	T2CON_ADDR, 4);	// selection UART0 transmittal clock: 0=timer1 overflow pulse, 1=timer2 overflow pulse
   SBIT(EXEN2,	T2CON_ADDR, 3);	// enable T2EX trigger function: 0=ignore T2EX, 1=trigger reload or capture by T2EX edge
   SBIT(TR2,	T2CON_ADDR, 2);	// timer2 run enable
   SBIT(C_T2,	T2CON_ADDR, 1);	// timer2 clock source selection: 0=timer base internal clock, 1=external edge counter base T2 falling edge
   SBIT(CP_RL2,	T2CON_ADDR, 0);	// timer2 function selection (force 0 if RCLK=1 or TCLK=1): 0=timer and auto reload if count overflow or T2EX edge, 1=capture by T2EX edge
SFR(T2MOD,	T2MOD_ADDR);	// timer 2 mode and timer 0/1/2 clock mode
#define bTMR_CLK          0x80      // fastest internal clock mode for timer 0/1/2 under faster clock mode: 0=use divided clock, 1=use original Fsys as clock without dividing
#define bT2_CLK           0x40      // timer2 internal clock frequency selection: 0=standard clock, Fsys/12 for timer mode, Fsys/4 for UART0 clock mode,
                                    //   1=faster clock, Fsys/4 @bTMR_CLK=0 or Fsys @bTMR_CLK=1 for timer mode, Fsys/2 @bTMR_CLK=0 or Fsys @bTMR_CLK=1 for UART0 clock mode
#define bT1_CLK           0x20      // timer1 internal clock frequency selection: 0=standard clock, Fsys/12, 1=faster clock, Fsys/4 if bTMR_CLK=0 or Fsys if bTMR_CLK=1
#define bT0_CLK           0x10      // timer0 internal clock frequency selection: 0=standard clock, Fsys/12, 1=faster clock, Fsys/4 if bTMR_CLK=0 or Fsys if bTMR_CLK=1
#define bT2_CAP_M1        0x08      // timer2 capture mode high bit
#define bT2_CAP_M0        0x04      // timer2 capture mode low bit
// bT2_CAP_M1 & bT2_CAP_M0: timer2 capture point selection
//   x0: from falling edge to falling edge
//   01: from any edge to any edge (level changing)
//   11: from rising edge to rising edge
#define T2OE              0x02      // enable timer2 generated clock output: 0=disable output, 1=enable clock output at T2 pin, frequency = TF2/2
#define bT2_CAP1_EN       0x01      // enable T2 trigger function for capture 1 of timer2 if RCLK=0 & TCLK=0 & CP_RL2=1 & C_T2=0 & T2OE=0
SFR16(RCAP2,	RCAP2L_ADDR);	// reload & capture value, little-endian
SFR(RCAP2L,	RCAP2L_ADDR);	// low byte of reload & capture value
SFR(RCAP2H,	RCAP2H_ADDR);	// high byte of reload & capture value
SFR16(T2COUNT,	TL2_ADDR);	// counter, little-endian
SFR(TL2,	TL2_ADDR);	// low byte of timer 2 count
SFR(TH2,	TH2_ADDR);	// high byte of timer 2 count
SFR16(T2CAP1,	T2CAP1L_ADDR);	// ReadOnly: capture 1 value for timer2
SFR(T2CAP1L,	T2CAP1L_ADDR);	// ReadOnly: capture 1 value low byte for timer2
SFR(T2CAP1H,	T2CAP1H_ADDR);	// ReadOnly: capture 1 value high byte for timer2

/*  PWM1/2 Registers  */
SFR(PWM_DATA2,	PWM_DATA2_ADDR);	// PWM data for PWM2
SFR(PWM_DATA1,	PWM_DATA1_ADDR);	// PWM data for PWM1
SFR(PWM_CTRL,	PWM_CTRL_ADDR);	// PWM 1/2 control
#define bPWM_IE_END       0x80      // enable interrupt for PWM mode cycle end
#define bPWM2_POLAR       0x40      // PWM2 output polarity: 0=default low and high action, 1=default high and low action
#define bPWM1_POLAR       0x20      // PWM1 output polarity: 0=default low and high action, 1=default high and low action
#define bPWM_IF_END       0x10      // interrupt flag for cycle end, write 1 to clear or write PWM_CYCLE or load new data to clear
#define bPWM2_OUT_EN      0x08      // PWM2 output enable
#define bPWM1_OUT_EN      0x04      // PWM1 output enable
#define bPWM_CLR_ALL      0x02      // force clear FIFO and count of PWM1/2
SFR(PWM_CK_SE,	PWM_CK_SE_ADDR);	// clock divisor setting

/*  SPI0/Master0/Slave Registers  */
SFR(SPI0_STAT,	SPI0_STAT_ADDR);	// SPI 0 status
   SBIT(S0_FST_ACT,	SPI0_STAT_ADDR, 7);	// ReadOnly: indicate first byte received status for SPI0
   SBIT(S0_IF_OV,	SPI0_STAT_ADDR, 6);	// interrupt flag for slave mode FIFO overflow, direct bit address clear or write 1 to clear
   SBIT(S0_IF_FIRST,SPI0_STAT_ADDR, 5);	// interrupt flag for first byte received, direct bit address clear or write 1 to clear
   SBIT(S0_IF_BYTE,	SPI0_STAT_ADDR, 4);	// interrupt flag for a byte data exchanged, direct bit address clear or write 1 to clear or accessing FIFO to clear if bS0_AUTO_IF=1
   SBIT(S0_FREE,	SPI0_STAT_ADDR, 3);	// ReadOnly: SPI0 free status
   SBIT(S0_T_FIFO,	SPI0_STAT_ADDR, 2);	// ReadOnly: tx FIFO count for SPI0
   SBIT(S0_R_FIFO,	SPI0_STAT_ADDR, 0);	// ReadOnly: rx FIFO count for SPI0
SFR(SPI0_DATA,	SPI0_DATA_ADDR);	// FIFO data port: reading for receiving, writing for transmittal
SFR(SPI0_CTRL,	SPI0_CTRL_ADDR);	// SPI 0 control
#define bS0_MISO_OE       0x80      // SPI0 MISO output enable
#define bS0_MOSI_OE       0x40      // SPI0 MOSI output enable
#define bS0_SCK_OE        0x20      // SPI0 SCK output enable
#define bS0_DATA_DIR      0x10      // SPI0 data direction: 0=out(master_write), 1=in(master_read)
#define bS0_MST_CLK       0x08      // SPI0 master clock mode: 0=mode 0 with default low, 1=mode 3 with default high
#define bS0_2_WIRE        0x04      // enable SPI0 two wire mode: 0=3 wire (SCK+MOSI+MISO), 1=2 wire (SCK+MISO)
#define bS0_CLR_ALL       0x02      // force clear FIFO and count of SPI0
#define bS0_AUTO_IF       0x01      // enable FIFO accessing to auto clear S0_IF_BYTE interrupt flag
SFR(SPI0_CK_SE,	SPI0_CK_SE_ADDR);	// clock divisor setting
//sfr SPI0_S_PRE      = 0xFB;         // preset value for SPI slave
#define SPI0_S_PRE        SPI0_CK_SE
SFR(SPI0_SETUP,	SPI0_SETUP_ADDR);	// SPI 0 setup
#define bS0_MODE_SLV      0x80      // SPI0 slave mode: 0=master, 1=slave
#define bS0_IE_FIFO_OV    0x40      // enable interrupt for slave mode FIFO overflow
#define bS0_IE_FIRST      0x20      // enable interrupt for first byte received for SPI0 slave mode
#define bS0_IE_BYTE       0x10      // enable interrupt for a byte received
#define bS0_BIT_ORDER     0x08      // SPI0 bit data order: 0=MSB first, 1=LSB first
#define bS0_SLV_SELT      0x02      // ReadOnly: SPI0 slave mode chip selected status: 0=unselected, 1=selected
#define bS0_SLV_PRELOAD   0x01      // ReadOnly: SPI0 slave mode data pre-loading status just after chip-selection

/*  UART1 Registers  */
SFR(SCON1,	SCON1_ADDR);	// UART1 control (serial port control)
   SBIT(U1SM0,	SCON1_ADDR, 7);	// UART1 mode, selection data bit: 0=8 bits data, 1=9 bits data
   SBIT(U1SMOD,	SCON1_ADDR, 5);	// UART1 2X baud rate selection: 0=slow(Fsys/32/(256-SBAUD1)), 1=fast(Fsys/16/(256-SBAUD1))
   SBIT(U1REN,	SCON1_ADDR, 4);	// enable UART1 receiving
   SBIT(U1TB8,	SCON1_ADDR, 3);	// the 9th transmitted data bit in 9 bits data mode
   SBIT(U1RB8,	SCON1_ADDR, 2);	// 9th data bit received in 9 bits data mode, or stop bit received for 8 bits data mode
   SBIT(U1TI,	SCON1_ADDR, 1);	// transmit interrupt flag, set by hardware after completion of a serial transmittal, need software clear
   SBIT(U1RI,	SCON1_ADDR, 0);	// receive interrupt flag, set by hardware after completion of a serial receiving, need software clear
SFR(SBUF1,	SBUF1_ADDR);	// UART1 data buffer: reading for receiving, writing for transmittal
SFR(SBAUD1,	SBAUD1_ADDR);	// UART1 baud rate setting

/*  ADC and comparator Registers  */
SFR(ADC_CTRL, ADC_CTRL_ADDR);	// ADC control
   SBIT(CMPO,	    ADC_CTRL_ADDR, 7);	// ReadOnly: comparator result input
   SBIT(CMP_IF,	    ADC_CTRL_ADDR, 6);	// flag for comparator result changed, direct bit address clear
   SBIT(ADC_IF,	    ADC_CTRL_ADDR, 5);	// interrupt flag for ADC finished, direct bit address clear
   SBIT(ADC_START,	ADC_CTRL_ADDR, 4);	// set 1 to start ADC, auto cleared when ADC finished
   SBIT(CMP_CHAN,	ADC_CTRL_ADDR, 3);	// comparator IN- input channel selection: 0=AIN1, 1=AIN3
   SBIT(ADC_CHAN1,	ADC_CTRL_ADDR, 1);	// ADC/comparator IN+ channel selection high bit
   SBIT(ADC_CHAN0,	ADC_CTRL_ADDR, 0);	// ADC/comparator IN+ channel selection low bit
// ADC_CHAN1 & ADC_CHAN0: ADC/comparator IN+ channel selection
//   00: AIN0(P1.1)
//   01: AIN1(P1.4)
//   10: AIN2(P1.5)
//   11: AIN3(P3.2)
SFR(ADC_CFG,	ADC_CFG_ADDR);	// ADC config
#define bADC_EN           0x08      // control ADC power: 0=shut down ADC, 1=enable power for ADC
#define bCMP_EN           0x04      // control comparator power: 0=shut down comparator, 1=enable power for comparator
#define bADC_CLK          0x01      // ADC clock frequency selection: 0=slow clock, 384 Fosc cycles for each ADC, 1=fast clock, 96 Fosc cycles for each ADC
SFR(ADC_DATA,	ADC_DATA_ADDR);	// ReadOnly: ADC data

/*  Touch-key timer Registers  */
SFR(TKEY_CTRL,	TKEY_CTRL_ADDR);	// touch-key control
#define bTKC_IF           0x80      // ReadOnly: interrupt flag for touch-key timer, cleared by writing touch-key control or auto cleared when start touch-key checking
#define bTKC_2MS          0x10      // touch-key timer cycle selection: 0=1mS, 1=2mS
#define bTKC_CHAN2        0x04      // touch-key channel selection high bit
#define bTKC_CHAN1        0x02      // touch-key channel selection middle bit
#define bTKC_CHAN0        0x01      // touch-key channel selection low bit
// bTKC_CHAN2 & bTKC_CHAN1 & bTKC_CHAN0: touch-key channel selection
//   000: disable touch-key
//   001: TIN0(P1.0)
//   010: TIN1(P1.1)
//   011: TIN2(P1.4)
//   100: TIN3(P1.5)
//   101: TIN4(P1.6)
//   110: TIN5(P1.7)
//   111: enable touch-key but disable all channel
SFR16(TKEY_DAT,	TKEY_DATL_ADDR);	// ReadOnly: touch-key data, little-endian
SFR(TKEY_DATL,	TKEY_DATL_ADDR);	// ReadOnly: low byte of touch-key data
SFR(TKEY_DATH,	TKEY_DATH_ADDR);	// ReadOnly: high byte of touch-key data
#define bTKD_CHG          0x80      // ReadOnly: indicate control changed, current data maybe invalid

/*  USB/Host/Device Registers  */
SFR(USB_C_CTRL,	USB_C_CTRL_ADDR);	// USB type-C control
#define bVBUS2_PD_EN      0x80      // USB VBUS2 10K pulldown resistance: 0=disable, 1=enable pullup
#define bUCC2_PD_EN       0x40      // USB CC2 5.1K pulldown resistance: 0=disable, 1=enable pulldown
#define bUCC2_PU1_EN      0x20      // USB CC2 pullup resistance control high bit
#define bUCC2_PU0_EN      0x10      // USB CC2 pullup resistance control low bit
#define bVBUS1_PD_EN      0x08      // USB VBUS1 10K pulldown resistance: 0=disable, 1=enable pullup
#define bUCC1_PD_EN       0x04      // USB CC1 5.1K pulldown resistance: 0=disable, 1=enable pulldown
#define bUCC1_PU1_EN      0x02      // USB CC1 pullup resistance control high bit
#define bUCC1_PU0_EN      0x01      // USB CC1 pullup resistance control low bit
// bUCC?_PU1_EN & bUCC?_PU0_EN: USB CC pullup resistance selection
//   00: disable pullup resistance
//   01: enable 56K pullup resistance for default USB power
//   10: enable 22K pullup resistance for 1.5A USB power
//   11: enable 10K pullup resistance for 3A USB power
SFR(UDEV_CTRL,	UDEV_CTRL_ADDR);	// USB device physical port control
#define bUD_PD_DIS        0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define bUD_DP_PIN        0x20      // ReadOnly: indicate current UDP pin level
#define bUD_DM_PIN        0x10      // ReadOnly: indicate current UDM pin level
#define bUD_LOW_SPEED     0x04      // enable USB physical port low speed: 0=full speed, 1=low speed
#define bUD_GP_BIT        0x02      // general purpose bit
#define bUD_PORT_EN       0x01      // enable USB physical port I/O: 0=disable, 1=enable
//sfr UHOST_CTRL      = 0xD1;         // USB host physical port control
#define UHOST_CTRL        UDEV_CTRL
#define bUH_PD_DIS        0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define bUH_DP_PIN        0x20      // ReadOnly: indicate current UDP pin level
#define bUH_DM_PIN        0x10      // ReadOnly: indicate current UDM pin level
#define bUH_LOW_SPEED     0x04      // enable USB port low speed: 0=full speed, 1=low speed
#define bUH_BUS_RESET     0x02      // control USB bus reset: 0=normal, 1=force bus reset
#define bUH_PORT_EN       0x01      // enable USB port: 0=disable, 1=enable port, automatic disabled if USB device detached
SFR(UEP1_CTRL,	UEP1_CTRL_ADDR);	// endpoint 1 control
#define bUEP_R_TOG        0x80      // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
#define bUEP_T_TOG        0x40      // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
#define bUEP_AUTO_TOG     0x10      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
#define bUEP_R_RES1       0x08      // handshake response type high bit for USB endpoint X receiving (OUT)
#define bUEP_R_RES0       0x04      // handshake response type low bit for USB endpoint X receiving (OUT)
#define MASK_UEP_R_RES    0x0C      // bit mask of handshake response type for USB endpoint X receiving (OUT)
#define UEP_R_RES_ACK     0x00
#define UEP_R_RES_TOUT    0x04
#define UEP_R_RES_NAK     0x08
#define UEP_R_RES_STALL   0x0C
// bUEP_R_RES1 & bUEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
//   00: ACK (ready)
//   01: no response, time out to host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
#define bUEP_T_RES1       0x02      // handshake response type high bit for USB endpoint X transmittal (IN)
#define bUEP_T_RES0       0x01      // handshake response type low bit for USB endpoint X transmittal (IN)
#define MASK_UEP_T_RES    0x03      // bit mask of handshake response type for USB endpoint X transmittal (IN)
#define UEP_T_RES_ACK     0x00
#define UEP_T_RES_TOUT    0x01
#define UEP_T_RES_NAK     0x02
#define UEP_T_RES_STALL   0x03
// bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
//   00: DATA0 or DATA1 then expecting ACK (ready)
//   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
SFR(UEP1_T_LEN,	UEP1_T_LEN_ADDR);	// endpoint 1 transmittal length
SFR(UEP2_CTRL,	UEP2_CTRL_ADDR);	// endpoint 2 control
SFR(UEP2_T_LEN,	UEP2_T_LEN_ADDR);	// endpoint 2 transmittal length
SFR(UEP3_CTRL,	UEP3_CTRL_ADDR);	// endpoint 3 control
SFR(UEP3_T_LEN,	UEP3_T_LEN_ADDR);	// endpoint 3 transmittal length
SFR(USB_INT_FG,	USB_INT_FG_ADDR);	// USB interrupt flag
   SBIT(U_IS_NAK,	    USB_INT_FG_ADDR, 7);	// ReadOnly: indicate current USB transfer is NAK received
   SBIT(U_TOG_OK,	    USB_INT_FG_ADDR, 6);	// ReadOnly: indicate current USB transfer toggle is OK
   SBIT(U_SIE_FREE,	    USB_INT_FG_ADDR, 5);	// ReadOnly: indicate USB SIE free status
   SBIT(UIF_FIFO_OV,	USB_INT_FG_ADDR, 4);	// FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
   SBIT(UIF_HST_SOF,	USB_INT_FG_ADDR, 3);	// host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
   SBIT(UIF_SUSPEND,	USB_INT_FG_ADDR, 2);	// USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
   SBIT(UIF_TRANSFER,	USB_INT_FG_ADDR, 1);	// USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
   SBIT(UIF_DETECT,	    USB_INT_FG_ADDR, 0);	// device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
   SBIT(UIF_BUS_RST,	USB_INT_FG_ADDR, 0);	// bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear
SFR(USB_INT_ST,	USB_INT_ST_ADDR);	// ReadOnly: USB interrupt status
#define bUIS_IS_NAK       0x80      // ReadOnly: indicate current USB transfer is NAK received for USB device mode
#define bUIS_TOG_OK       0x40      // ReadOnly: indicate current USB transfer toggle is OK
#define bUIS_TOKEN1       0x20      // ReadOnly: current token PID code bit 1 received for USB device mode
#define bUIS_TOKEN0       0x10      // ReadOnly: current token PID code bit 0 received for USB device mode
#define MASK_UIS_TOKEN    0x30      // ReadOnly: bit mask of current token PID code received for USB device mode
#define UIS_TOKEN_OUT     0x00
#define UIS_TOKEN_SOF     0x10
#define UIS_TOKEN_IN      0x20
#define UIS_TOKEN_SETUP   0x30
// bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode
//   00: OUT token PID received
//   01: SOF token PID received
//   10: IN token PID received
//   11: SETUP token PID received
#define MASK_UIS_ENDP     0x0F      // ReadOnly: bit mask of current transfer endpoint number for USB device mode
#define MASK_UIS_H_RES    0x0F      // ReadOnly: bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received
SFR(USB_MIS_ST,	USB_MIS_ST_ADDR);	// ReadOnly: USB miscellaneous status
#define bUMS_SOF_PRES     0x80      // ReadOnly: indicate host SOF timer presage status
#define bUMS_SOF_ACT      0x40      // ReadOnly: indicate host SOF timer action status for USB host
#define bUMS_SIE_FREE     0x20      // ReadOnly: indicate USB SIE free status
#define bUMS_R_FIFO_RDY   0x10      // ReadOnly: indicate USB receiving FIFO ready status (not empty)
#define bUMS_BUS_RESET    0x08      // ReadOnly: indicate USB bus reset status
#define bUMS_SUSPEND      0x04      // ReadOnly: indicate USB suspend status
#define bUMS_DM_LEVEL     0x02      // ReadOnly: indicate UDM level saved at device attached to USB host
#define bUMS_DEV_ATTACH   0x01      // ReadOnly: indicate device attached status on USB host
SFR(USB_RX_LEN,	USB_RX_LEN_ADDR);	// ReadOnly: USB receiving length
SFR(UEP0_CTRL,	UEP0_CTRL_ADDR);	// endpoint 0 control
SFR(UEP0_T_LEN,	UEP0_T_LEN_ADDR);	// endpoint 0 transmittal length
SFR(UEP4_CTRL,	UEP4_CTRL_ADDR);	// endpoint 4 control
SFR(UEP4_T_LEN,	UEP4_T_LEN_ADDR);	// endpoint 4 transmittal length
SFR(USB_INT_EN,	USB_INT_EN_ADDR);	// USB interrupt enable
#define bUIE_DEV_SOF      0x80      // enable interrupt for SOF received for USB device mode
#define bUIE_DEV_NAK      0x40      // enable interrupt for NAK responded for USB device mode
#define bUIE_FIFO_OV      0x10      // enable interrupt for FIFO overflow
#define bUIE_HST_SOF      0x08      // enable interrupt for host SOF timer action for USB host mode
#define bUIE_SUSPEND      0x04      // enable interrupt for USB suspend or resume event
#define bUIE_TRANSFER     0x02      // enable interrupt for USB transfer completion
#define bUIE_DETECT       0x01      // enable interrupt for USB device detected event for USB host mode
#define bUIE_BUS_RST      0x01      // enable interrupt for USB bus reset event for USB device mode
SFR(USB_CTRL,	USB_CTRL_ADDR);	// USB base control
#define bUC_HOST_MODE     0x80      // enable USB host mode: 0=device mode, 1=host mode
#define bUC_LOW_SPEED     0x40      // enable USB low speed: 0=full speed, 1=low speed
#define bUC_DEV_PU_EN     0x20      // USB device enable and internal pullup resistance enable
#define bUC_SYS_CTRL1     0x20      // USB system control high bit
#define bUC_SYS_CTRL0     0x10      // USB system control low bit
#define MASK_UC_SYS_CTRL  0x30      // bit mask of USB system control
// bUC_HOST_MODE & bUC_SYS_CTRL1 & bUC_SYS_CTRL0: USB system control
//   0 00: disable USB device and disable internal pullup resistance
//   0 01: enable USB device and disable internal pullup resistance, need external pullup resistance
//   0 1x: enable USB device and enable internal pullup resistance
//   1 00: enable USB host and normal status
//   1 01: enable USB host and force UDP/UDM output SE0 state
//   1 10: enable USB host and force UDP/UDM output J state
//   1 11: enable USB host and force UDP/UDM output resume or K state
#define bUC_INT_BUSY      0x08      // enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
#define bUC_RESET_SIE     0x04      // force reset USB SIE, need software clear
#define bUC_CLR_ALL       0x02      // force clear FIFO and count of USB
#define bUC_DMA_EN        0x01      // DMA enable and DMA interrupt enable for USB
SFR(USB_DEV_AD,	USB_DEV_AD_ADDR);	// USB device address, lower 7 bits for USB device address
#define bUDA_GP_BIT       0x80      // general purpose bit
#define MASK_USB_ADDR     0x7F      // bit mask for USB device address
SFR16(UEP2_DMA,	UEP2_DMA_L_ADDR);	// endpoint 2 buffer start address, little-endian
SFR(UEP2_DMA_L,	UEP2_DMA_L_ADDR);	// endpoint 2 buffer start address low byte
SFR(UEP2_DMA_H,	UEP2_DMA_H_ADDR);	// endpoint 2 buffer start address high byte
SFR16(UEP3_DMA,	UEP3_DMA_L_ADDR);	// endpoint 3 buffer start address, little-endian
SFR(UEP3_DMA_L,	UEP3_DMA_L_ADDR);	// endpoint 3 buffer start address low byte
SFR(UEP3_DMA_H,	UEP3_DMA_H_ADDR);	// endpoint 3 buffer start address high byte
SFR(UEP4_1_MOD,	UEP4_1_MOD_ADDR);	// endpoint 4/1 mode
#define bUEP1_RX_EN       0x80      // enable USB endpoint 1 receiving (OUT)
#define bUEP1_TX_EN       0x40      // enable USB endpoint 1 transmittal (IN)
#define bUEP1_BUF_MOD     0x10      // buffer mode of USB endpoint 1
// bUEPn_RX_EN & bUEPn_TX_EN & bUEPn_BUF_MOD: USB endpoint 1/2/3 buffer mode, buffer start address is UEPn_DMA
//   0 0 x:  disable endpoint and disable buffer
//   1 0 0:  64 bytes buffer for receiving (OUT endpoint)
//   1 0 1:  dual 64 bytes buffer by toggle bit bUEP_R_TOG selection for receiving (OUT endpoint), total=128bytes
//   0 1 0:  64 bytes buffer for transmittal (IN endpoint)
//   0 1 1:  dual 64 bytes buffer by toggle bit bUEP_T_TOG selection for transmittal (IN endpoint), total=128bytes
//   1 1 0:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes
//   1 1 1:  dual 64 bytes buffer by bUEP_R_TOG selection for receiving (OUT endpoint) + dual 64 bytes buffer by bUEP_T_TOG selection for transmittal (IN endpoint), total=256bytes
#define bUEP4_RX_EN       0x08      // enable USB endpoint 4 receiving (OUT)
#define bUEP4_TX_EN       0x04      // enable USB endpoint 4 transmittal (IN)
// bUEP4_RX_EN & bUEP4_TX_EN: USB endpoint 4 buffer mode, buffer start address is UEP0_DMA
//   0 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//   1 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 receiving (OUT endpoint), total=128bytes
//   0 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=128bytes
//   1 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//           + 64 bytes buffer for endpoint 4 receiving (OUT endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=192bytes
SFR(UEP2_3_MOD,	UEP2_3_MOD_ADDR);	// endpoint 2/3 mode
#define bUEP3_RX_EN       0x80      // enable USB endpoint 3 receiving (OUT)
#define bUEP3_TX_EN       0x40      // enable USB endpoint 3 transmittal (IN)
#define bUEP3_BUF_MOD     0x10      // buffer mode of USB endpoint 3
#define bUEP2_RX_EN       0x08      // enable USB endpoint 2 receiving (OUT)
#define bUEP2_TX_EN       0x04      // enable USB endpoint 2 transmittal (IN)
#define bUEP2_BUF_MOD     0x01      // buffer mode of USB endpoint 2
SFR16(UEP0_DMA,	UEP0_DMA_L_ADDR);	// endpoint 0 buffer start address, little-endian
SFR(UEP0_DMA_L,	UEP0_DMA_L_ADDR);	// endpoint 0 buffer start address low byte
SFR(UEP0_DMA_H,	UEP0_DMA_H_ADDR);	// endpoint 0 buffer start address high byte
SFR16(UEP1_DMA,	UEP1_DMA_L_ADDR);	// endpoint 1 buffer start address, little-endian
SFR(UEP1_DMA_L,	UEP1_DMA_L_ADDR);	// endpoint 1 buffer start address low byte
SFR(UEP1_DMA_H,	UEP1_DMA_H_ADDR);	// endpoint 1 buffer start address high byte
//sfr UH_SETUP        = 0xD2;         // host aux setup
#define UH_SETUP          UEP1_CTRL
#define bUH_PRE_PID_EN    0x80      // USB host PRE PID enable for low speed device via hub
#define bUH_SOF_EN        0x40      // USB host automatic SOF enable
//sfr UH_RX_CTRL      = 0xD4;         // host receiver endpoint control
#define UH_RX_CTRL        UEP2_CTRL
#define bUH_R_TOG         0x80      // expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
#define bUH_R_AUTO_TOG    0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define bUH_R_RES         0x04      // prepared handshake response type for host receiving (IN): 0=ACK (ready), 1=no response, time out to device, for isochronous transactions
//sfr UH_EP_PID       = 0xD5;         // host endpoint and token PID, lower 4 bits for endpoint number, upper 4 bits for token PID
#define UH_EP_PID         UEP2_T_LEN
#define MASK_UH_TOKEN     0xF0      // bit mask of token PID for USB host transfer
#define MASK_UH_ENDP      0x0F      // bit mask of endpoint number for USB host transfer
//sfr UH_TX_CTRL      = 0xD6;         // host transmittal endpoint control
#define UH_TX_CTRL        UEP3_CTRL
#define bUH_T_TOG         0x40      // prepared data toggle flag of host transmittal (SETUP/OUT): 0=DATA0, 1=DATA1
#define bUH_T_AUTO_TOG    0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define bUH_T_RES         0x01      // expected handshake response type for host transmittal (SETUP/OUT): 0=ACK (ready), 1=no response, time out from device, for isochronous transactions
//sfr UH_TX_LEN       = 0xD7;         // host transmittal endpoint transmittal length
#define UH_TX_LEN         UEP3_T_LEN
//sfr UH_EP_MOD       = 0xEB;         // host endpoint mode
#define UH_EP_MOD         UEP2_3_MOD
#define bUH_EP_TX_EN      0x40      // enable USB host OUT endpoint transmittal
#define bUH_EP_TBUF_MOD   0x10      // buffer mode of USB host OUT endpoint
// bUH_EP_TX_EN & bUH_EP_TBUF_MOD: USB host OUT endpoint buffer mode, buffer start address is UH_TX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for transmittal (OUT endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_T_TOG selection for transmittal (OUT endpoint), total=128bytes
#define bUH_EP_RX_EN      0x08      // enable USB host IN endpoint receiving
#define bUH_EP_RBUF_MOD   0x01      // buffer mode of USB host IN endpoint
// bUH_EP_RX_EN & bUH_EP_RBUF_MOD: USB host IN endpoint buffer mode, buffer start address is UH_RX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for receiving (IN endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_R_TOG selection for receiving (IN endpoint), total=128bytes
//sfr16 UH_RX_DMA     = 0xE4;         // host rx endpoint buffer start address, little-endian
#define UH_RX_DMA         UEP2_DMA
//sfr UH_RX_DMA_L     = 0xE4;         // host rx endpoint buffer start address low byte
#define UH_RX_DMA_L       UEP2_DMA_L
//sfr UH_RX_DMA_H     = 0xE5;         // host rx endpoint buffer start address high byte
#define UH_RX_DMA_H       UEP2_DMA_H
//sfr16 UH_TX_DMA     = 0xE6;         // host tx endpoint buffer start address, little-endian
#define UH_TX_DMA         UEP3_DMA
//sfr UH_TX_DMA_L     = 0xE6;         // host tx endpoint buffer start address low byte
#define UH_TX_DMA_L       UEP3_DMA_L
//sfr UH_TX_DMA_H     = 0xE7;         // host tx endpoint buffer start address high byte
#define UH_TX_DMA_H       UEP3_DMA_H

/*----- XDATA: xRAM ------------------------------------------*/

#define XDATA_RAM_SIZE    0x0400    // size of expanded xRAM, xdata SRAM embedded chip

/*----- Reference Information --------------------------------------------*/
#define ID_CH554          0x54      // chip ID

/* Interrupt routine address and interrupt number */
#define INT_ADDR_INT0     0x0003    // interrupt vector address for INT0
#define INT_ADDR_TMR0     0x000B    // interrupt vector address for timer0
#define INT_ADDR_INT1     0x0013    // interrupt vector address for INT1
#define INT_ADDR_TMR1     0x001B    // interrupt vector address for timer1
#define INT_ADDR_UART0    0x0023    // interrupt vector address for UART0
#define INT_ADDR_TMR2     0x002B    // interrupt vector address for timer2
#define INT_ADDR_SPI0     0x0033    // interrupt vector address for SPI0
#define INT_ADDR_TKEY     0x003B    // interrupt vector address for touch-key timer
#define INT_ADDR_USB      0x0043    // interrupt vector address for USB
#define INT_ADDR_ADC      0x004B    // interrupt vector address for ADC
#define INT_ADDR_UART1    0x0053    // interrupt vector address for UART1
#define INT_ADDR_PWMX     0x005B    // interrupt vector address for PWM1/2
#define INT_ADDR_GPIO     0x0063    // interrupt vector address for GPIO
#define INT_ADDR_WDOG     0x006B    // interrupt vector address for watch-dog timer
#define INT_NO_INT0       0         // interrupt number for INT0
#define INT_NO_TMR0       1         // interrupt number for timer0
#define INT_NO_INT1       2         // interrupt number for INT1
#define INT_NO_TMR1       3         // interrupt number for timer1
#define INT_NO_UART0      4         // interrupt number for UART0
#define INT_NO_TMR2       5         // interrupt number for timer2
#define INT_NO_SPI0       6         // interrupt number for SPI0
#define INT_NO_TKEY       7         // interrupt number for touch-key timer
#define INT_NO_USB        8         // interrupt number for USB
#define INT_NO_ADC        9         // interrupt number for ADC
#define INT_NO_UART1      10        // interrupt number for UART1
#define INT_NO_PWMX       11        // interrupt number for PWM1/2
#define INT_NO_GPIO       12        // interrupt number for GPIO
#define INT_NO_WDOG       13        // interrupt number for watch-dog timer

/* Special Program Space */
#define DATA_FLASH_ADDR   0xC000    // start address of Data-Flash
#define BOOT_LOAD_ADDR    0x3800    // start address of boot loader program
#define ROM_CFG_ADDR      0x3FF8    // chip configuration information address
#define ROM_CHIP_ID_HX    0x3FFA    // chip ID number highest byte (only low byte valid)
#define ROM_CHIP_ID_LO    0x3FFC    // chip ID number low word
#define ROM_CHIP_ID_HI    0x3FFE    // chip ID number high word

#endif  // __CH554_H__
