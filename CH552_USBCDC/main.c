#include <stdio.h>
#include <ch552.h>
#include <USB_Serial.h>
#include <delay.h>

// the following line is needed
USB_INTERRUPT

void CfgFsys(void);

void main(void)
{
    uint8_t value = 0;
    CfgFsys();
    USBCDCInit();

    while(1){
        printf_tiny("Hello, world %d\n",value++);
        mDelaymS(100);
    }


}
