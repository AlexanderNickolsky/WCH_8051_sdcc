#ifndef USB_SERIAL_H_
#define USB_SERIAL_H_
#include <stdint.h>

// configuration
// #define REPORT_BAUD_RATE  // define REPORT_BAUD_RATE to make the device to respond to GET_LINE_CONFIG message
// #define NEED_READ         // define NEED_READ if you want to read data from the CDC.


// add line USB_INTERRUPT to the main file of the project (typically main.c)
// outside of any function

#define USB_INTERRUPT void DeviceInterrupt(void) __interrupt (INT_NO_USB) \
{ USBInterrupt( ); }

void USBCDCInit(void);
void SendData(uint8_t* SendBuf, uint8_t SendLEN);
void USBInterrupt( void );

#endif
