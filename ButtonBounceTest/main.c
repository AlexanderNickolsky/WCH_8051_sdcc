#include <stdio.h>
#include <ch552.h>
#include <ch552_gpio.h>
#include <USB_Serial.h>
#include <delay.h>

/*
This program shows how much is the bouncing of your button.
Attach the button to pin 1.5 of the CH552 and ground.
Start serial monitoring program (Putty or something else) at 57600 baud rate.
Press the button several times and see how many times it actually bounced.
*/

USB_INTERRUPT

PINCHANGE_INTERRUPT(OnPinChange)

void CfgFsys(void);

volatile uint16_t counter = 0;

void OnPinChange(void) {
    ++counter;
}

void main(void)
{
    CfgFsys();
    USBCDCInit();

    P1SetModeOD_PULLUP(_BV(5));
    EnablePinChangeIntr(P1_5);
    EnableGPIOInterrupt();
    EA = 1;

    while(1){
        if(counter != 0){
            printf("%d\n",counter);
            mDelaymS(100);
            counter = 0;
        }
    }


}
