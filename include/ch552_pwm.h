#pragma once

// CH552 PWM functions

#define SetPWMClk(CK_SE) (PWM_CK_SE = CK_SE)                                  //Frequency division, default clock Fsys

#define SetPWM1Dat(dat)  (PWM_DATA1 = dat)
#define SetPWM2Dat(dat)  (PWM_DATA2 = dat)

#define GetPWM1Dat()     (PWM_DATA1)
#define GetPWM2Dat()     (PWM_DATA2)

#define PWM1PinAlter( )  (PIN_FUNC |= bPWM1_PIN_X)                            // PWM mapping pin 3.1
#define PWM2PinAlter( )  (PIN_FUNC |= bPWM2_PIN_X)                            // PWM mapping pin 3.0

#define ForceClearPWMFIFO( )  (PWM_CTRL |= bPWM_CLR_ALL)                      // Empty PWM count and FIFO
#define CancelClearPWMFIFO( ) (PWM_CTRL &= ~bPWM_CLR_ALL)                     // Stop clearing PWM count and FIFO

#define PWM1OutEnable()  (PWM_CTRL  |=  bPWM1_OUT_EN)                         // PWM1 output enable
#define PWM2OutEnable()  (PWM_CTRL  |=  bPWM2_OUT_EN)                         // PWM2 output enable

#define PWM1OutDisable() (PWM_CTRL &=  ~bPWM1_OUT_EN)                         // PWM1 output disable
#define PWM2OutDisable() (PWM_CTRL &= ~bPWM2_OUT_EN)                         // PWM2 output disable


#define PWM1OutPolarHighAct() (PWM_CTRL &= ~bPWM1_POLAR)                       //PWM0 Low level by default, while active high.
#define PWM2OutPolarHighAct() (PWM_CTRL &= ~bPWM2_POLAR)                       //PWM1 Low level by default, while active high.
#define PWM1OutPolarLowAct()  (PWM_CTRL |= bPWM1_POLAR)                        //PWM0 High level by default while active low
#define PWM2OutPolarLowAct()  (PWM_CTRL |= bPWM2_POLAR)                        //PWM1 High level by default while active low

//PWM Interrupt
#define PWMInterruptEnable()  (PWM_CTRL |= bPWM_IF_END | bPWM_IE_END, IE_PWMX = 1)
#define PWMInterruptDisable() (IE_PWMX = 0)
