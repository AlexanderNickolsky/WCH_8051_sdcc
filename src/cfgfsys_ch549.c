
#include <stdint.h>
#include "ch549.h"


/*******************************************************************************
* Function Name  : CfgFsys( )
* Description  : CH554 clock selection and configuration function, Fsys 6MHz is used by default, FREQ_SYS can be passed
                 CLOCK_CFG configuration, the formula is as follows:
                 Fsys = (Fosc * 4 / (CLOCK_CFG & MASK_SYS_CK_SEL); the specific clock needs to be configured by yourself
*******************************************************************************/
void	CfgFsys( ) // CH549 version
{
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
//     CLOCK_CFG |= bOSC_EN_XT;                          // Enable external crystal
//     CLOCK_CFG & = ~ bOSC_EN_INT;                      // Turn off the internal crystal
#if FREQ_SYS == 48000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x07;  // 32MHz
#elif FREQ_SYS == 32000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x06;  // 32MHz
#elif FREQ_SYS == 24000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x05;  // 24MHz
#elif FREQ_SYS == 16000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x04;  // 16MHz
#elif FREQ_SYS == 12000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x03;  // 12MHz
#elif FREQ_SYS == 3000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x02;  // 3MHz
#elif FREQ_SYS == 750000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x01;  // 750KHz
#elif FREQ_SYS == 187500
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x00;  // 187.5MHz
#else
	#warning FREQ_SYS invalid or not set
#endif

	SAFE_MOD = 0x00;
}

