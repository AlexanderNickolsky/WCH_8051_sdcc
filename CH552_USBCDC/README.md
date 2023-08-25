# Printing with USB

This is the minimal implementation of USB serial connection for CH552.
It takes less than 6 kb of code and allows debugging with printf().
You need to add USB_Serial_ch552.c cfgfsys_ch552.c delay_ch552.c to the
project and add **include** folder to compiler's search path.

Baud rate is 57600 and it emulates a CH340 chip, so no special driver
is needed for Windows operating system. The system should install the
driver automatically.

Tested on Linux and Windows 10.

