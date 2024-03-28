#include <ch552.h>
#include <string.h>
#include <ch552_usb.h>
#include "USB_Serial.h"

#define  THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define  SET_LINE_CODING                0X20            // Configures DTE rate, stop-bits, parity, and number-of-character
#define  GET_LINE_CODING                0X21            // This request allows the host to find out the currently configured line coding.
#define  SET_CONTROL_LINE_STATE         0X22            // This request generates RS-232/V.24 style control signals.


static __code uint8_t DevDesc[] = {
    0x12,        // bLength
    0x01,        // bDescriptorType (Device)
    0x10, 0x01,  // bcdUSB 1.10
    0x02,        // bDeviceClass
    0x00,        // bDeviceSubClass
    0x02,        // bDeviceProtocol
    0x08,        // bMaxPacketSize0 8
    0x86, 0x1A,  // idVendor 0x1A86
    0x23, 0x75,  // idProduct 0x7523
    0x04, 0x03,  // bcdDevice 6.04
    0x00,        // iManufacturer (String Index)
    0x00,        // iProduct (String Index)
    0x00,        // iSerialNumber (String Index)
    0x01         // bNumConfigurations 1
// 18 bytes
};

static __code uint8_t CfgDesc[] = {
    0x09,        // bLength
    0x02,        // bDescriptorType (Configuration)
    0x27, 0x00,  // wTotalLength 39
    0x01,        // bNumInterfaces 1
    0x01,        // bConfigurationValue
    0x00,        // iConfiguration (String Index)
    0x80,        // bmAttributes
    0xF0,        // bMaxPower 480mA

    0x09,        // bLength
    0x04,        // bDescriptorType (Interface)
    0x00,        // bInterfaceNumber 0
    0x00,        // bAlternateSetting
    0x03,        // bNumEndpoints 3
    0x02,        // bInterfaceClass
    0x01,        // bInterfaceSubClass
    0x02,        // bInterfaceProtocol
    0x00,        // iInterface (String Index)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x82,        // bEndpointAddress (IN/D2H)
    0x02,        // bmAttributes (Bulk)
    0x20, 0x00,  // wMaxPacketSize 32
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x02,        // bEndpointAddress (OUT/H2D)
    0x02,        // bmAttributes (Bulk)
    0x20, 0x00,  // wMaxPacketSize 32
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x81,        // bEndpointAddress (IN/D2H)
    0x03,        // bmAttributes (Interrupt)
    0x08, 0x00,  // wMaxPacketSize 8
    0x01,         // bInterval 1 (unit depends on device speed)

// 39 bytes
};


// xdata

#define EP2ADDR (XDATA_RAM_SIZE - 2*MAX_PACKET_SIZE)
#define EP1ADDR (EP2ADDR - 64)
#define EP0ADDR (EP1ADDR - 64)

//__xdata __at (0x0000) uint8_t  Ep0Buffer[DEFAULT_ENDP0_SIZE];      // Endpoint 0 OUT & IN buffer, must be an even address
//__xdata __at (0x0040) uint8_t  Ep1Buffer[DEFAULT_ENDP1_SIZE];       //Endpoint 1 upload buffer
//__xdata __at (0x0080) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];        //Endpoint 2 IN & OUT buffer, must be an even address

static __xdata __at (EP0ADDR) uint8_t  Ep0Buffer[DEFAULT_ENDP0_SIZE];       // Endpoint 0 OUT & IN buffer, must be an even address
static __xdata __at (EP1ADDR) uint8_t  Ep1Buffer[DEFAULT_ENDP1_SIZE];       //Endpoint 1 upload buffer
static __xdata __at (EP2ADDR) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];        //Endpoint 2 IN & OUT buffer, must be an even address

#ifdef NEED_READ
__xdata uint8_t  RecBuf[64];
#endif // NEED_READ

static uint8_t num = 0;
static uint8_t ptr_printf_buffer = 0;
static uint8_t SetReqtp, SetupReq,Count,UsbConfig;
volatile uint8_t Flag;
static uint16_t SetupLen;
static const uint8_t * pDescr;                                                       //USB configuration flag

USB_SETUP_REQ   SetupReqBuf;

static uint8_t __code DataBuf[] = {0x30,0x00,0xc3,0x00,0xff,0xec,0x9f,0xec,0xff,0xec,0xdf,0xec,
                            0xdf,0xec,0xdf,0xec,0x9f,0xec,0x9f,0xec,0x9f,0xec,0x9f,0xec,
                            0xff,0xec
                           };


void USBDeviceCfg(void);
void USBDeviceIntCfg(void);
void USBDeviceEndPointCfg(void);

/*
int putchar(int c) {
    printf_buffer[ptr_printf_buffer++] = c;
    if(ptr_printf_buffer == PRINTF_BUFFER_SIZE - 1 || c == '\n' ) {
        SendData(printf_buffer, ptr_printf_buffer);
        ptr_printf_buffer = 0;
    }
    return c;
}
*/

void	mDelayuS( uint16_t n );

int putchar(int c) {
    Ep2Buffer[MAX_PACKET_SIZE] = c;
    UEP2_T_LEN = 1;
    UEP2_CTRL &= ~(bUEP_T_RES1 | bUEP_T_RES0);
    while(( UEP2_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_ACK );
    Flag = 0;
    return c;
}


/*******************************************************************************
* Function Name  : USBCDCInit()
* Description    : USB CDC initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void USBCDCInit(void) {
    USBDeviceCfg();
    USBDeviceEndPointCfg();
    USBDeviceIntCfg();
    UEP0_T_LEN = 0;
    UEP1_T_LEN = 0;
    UEP2_T_LEN = 0;
}

/*******************************************************************************
* Function Name  : USBDeviceCfg()
* Description    : USB device mode configuration
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceCfg(void) {
    USB_CTRL = 0x00;                                                           //Clear USB control register
    //USB_CTRL &= ~bUC_HOST_MODE;                                              //This bit selects the device mode
    USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                    //USB device and internal pull-up enable, automatically return to NAK before interrupt flag is cleared
    USB_DEV_AD = 0x00;                                                         //Device address initialization
    //     USB_CTRL |= bUC_LOW_SPEED;
    //     UDEV_CTRL |= bUD_LOW_SPEED;                                         //Select low speed 1.5M mode
    USB_CTRL &= ~bUC_LOW_SPEED;
    UDEV_CTRL &= ~bUD_LOW_SPEED;                                               //Select full speed 12M mode, the default mode
    UDEV_CTRL = bUD_PD_DIS;  // Disable DP / DM pull-down resistor
    UDEV_CTRL |= bUD_PORT_EN;                                                  //Enable physical port
}


/*******************************************************************************
* Function Name  : USBDeviceIntCfg()
* Description    : USB device mode interrupt initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceIntCfg(void) {
    USB_INT_EN |= bUIE_SUSPEND;                 //Enable device suspend interrupt
    USB_INT_EN |= bUIE_TRANSFER;                //Enable USB transfer completion interrupt
    USB_INT_EN |= bUIE_BUS_RST;                 //Enable device mode USB bus reset interrupt
    USB_INT_FG |= 0x1F;                         //Clear interrupt flag
    IE_USB = 1;                                 //Enable USB interrupt
    EA = 1;                                     //Allow microcontroller interrupt
}

/*******************************************************************************
* Function Name  : USBDeviceEndPointCfg()
* Description    : USB device mode endpoint configuration, simulation compatible HID device, in addition to endpoint 0 control transmission, also includes endpoint 2 batch upload
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceEndPointCfg(void) {
    // TODO: Is casting the right thing here? What about endianness?
    UEP1_DMA = (uint16_t) Ep1Buffer;                                          //Endpoint 1 sends data transfer address
    UEP2_DMA = (uint16_t) Ep2Buffer;                                          //Endpoint 2 IN data transfer address
    UEP2_3_MOD = 0xCC;                                                         //Endpoint 2/3 single buffer transceiver enable
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;                 //Endpoint 2 automatically flips the synchronization flag, IN transaction returns NAK, OUT returns ACK

    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //Endpoint 1 automatically flips the synchronization flag, IN transaction returns NAK
    UEP0_DMA = (uint16_t) Ep0Buffer;                                          //Endpoint 0 data transfer address
    UEP4_1_MOD = 0X40;                                                         //Endpoint 1 upload buffer; endpoint 0 single 64-byte send and receive buffer
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //Manual flip, OUT transaction returns ACK, IN transaction returns NAK
}

void SendData(uint8_t* SendBuf, uint8_t SendLEN) {
    while(SendLEN > 32) {
        memcpy(&Ep2Buffer[MAX_PACKET_SIZE],SendBuf,32);
        UEP2_T_LEN = 32;
        UEP2_CTRL &= ~(bUEP_T_RES1 | bUEP_T_RES0);								// means reply ACK or ready
        while(( UEP2_CTRL & MASK_UEP_T_RES ) == UEP_T_RES_ACK);  //
        SendLEN -= 32;
    }
    memcpy(&Ep2Buffer[MAX_PACKET_SIZE],SendBuf,SendLEN);
    UEP2_T_LEN = SendLEN;
    UEP2_CTRL &= ~(bUEP_T_RES1 | bUEP_T_RES0);
    Flag = 0;
}

#ifdef NEED_READ
void ReceiveData(void) {
    uint8_t len = USB_RX_LEN;
    memcpy(RecBuf,Ep2Buffer,len);
    RecBuf[len] = 0;
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK; //Default response ACK
    Flag = 1;
}
#else
void ReceiveData(void){}
#endif


#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

void USBInterrupt( void ) {
    uint16_t len;
    if(UIF_TRANSFER){
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP)) {
        case UIS_TOKEN_OUT | 2:
            len = USB_RX_LEN;
            ReceiveData();
            break;
        case UIS_TOKEN_IN | 2:
            UEP2_T_LEN = 0;
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
            break;
        case UIS_TOKEN_SETUP | 0:
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ))) {
                SetReqtp = UsbSetupBuf->bRequestType;
                SetupLen = UsbSetupBuf->wLengthL;
                SetupReq = UsbSetupBuf->bRequest;
                len = 0;

                if(SetReqtp == 0xc0) {
                    Ep0Buffer[0] = DataBuf[num];
                    Ep0Buffer[1] = DataBuf[num+1];
                    len = 2;
                    if(num<24) {
                        num += 2;
                    } else {
                        num = 24;
                    }
                } else if(SetReqtp == 0x40) {
                    len = 9;
                } else {
                    switch(SetupReq) {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH) {
                        case 1:
                            pDescr = DevDesc;
                            len = sizeof(DevDesc);
                            break;
                        case 2:
                            pDescr = CfgDesc;
                            len = sizeof(CfgDesc);
                            break;
                        default:
                            len = 0xff;
                            break;
                        }
                        if ( SetupLen > len ) SetupLen = len;
                        len = SetupLen >= 8 ? 8 : SetupLen;
                        memcpy(Ep0Buffer,pDescr,len);
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if ( SetupLen >= 1 ) len = 1;
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    default:
                        len = 0xff;
                        break;
                    }
                }
            } else {
                len = 0xff;
            }
            if(len == 0xff) {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;  //STALL
            } else if(len <= 8) {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
            } else {
                UEP0_T_LEN = 0;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
            }
            break;
        case UIS_TOKEN_IN | 0:                                                         //endpoint0 IN
            switch(SetupReq) {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen;
                memcpy( Ep0Buffer, pDescr, len );
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0:                                                 // endpoint0 OUT
            len = USB_RX_LEN;
            UEP0_T_LEN = 0;
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;
    }
    if(UIF_BUS_RST) {
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;
    }
    if (UIF_SUSPEND) {
        UIF_SUSPEND = 0;
            //if ( USB_MIS_ST & bUMS_SUSPEND ) {
            //SAFE_MOD = 0x55;
            //SAFE_MOD = 0xAA;
            //WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;
            //PCON |= PD;
            //SAFE_MOD = 0x55;
            //SAFE_MOD = 0xAA;
            //WAKE_CTRL = 0x00;
        //}
    } else {
        USB_INT_FG = 0x00;
    }
}




