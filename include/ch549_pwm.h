#pragma once

// CH549 PWM functions

// #define package type before including this file to enable some checks for port/pin existence
// #define CH549L or CH548L or default - LQFP48
// #define CH549F - QFN28
// #define CH549G or CH548G - SOP16
// #define CH548N - SOP8

#if PWM_INTERRUPT
extern void  PWMInterruptEnable();                                            //PWM interrupt enable
#endif

#define SetPWMClk(CK_SE) (PWM_CK_SE = CK_SE)                                  //Frequency division, default clock Fsys

#define SetPWM0Dat(dat)  (PWM_DATA0 = dat)                                    //Set PWM output duty cycle
#define SetPWM1Dat(dat)  (PWM_DATA1 = dat)
#define SetPWM2Dat(dat)  (PWM_DATA2 = dat)
#define SetPWM3Dat(dat)  (PWM_DATA3 = dat)
#define SetPWM4Dat(dat)  (PWM_DATA4 = dat)
#define SetPWM5Dat(dat)  (PWM_DATA5 = dat)
#define SetPWM6Dat(dat)  (PWM_DATA6 = dat)
#define SetPWM7Dat(dat)  (PWM_DATA7 = dat)

#define GetPWM1Dat()     (PWM_DATA1)
#define GetPWM2Dat()     (PWM_DATA2)
#define GetPWM3Dat()     (PWM_DATA3)

#if defined(CH549G) || defined(CH548G)
  #undef SetPWM2Dat
  #undef SetPWM4Dat
  #undef SetPWM5Dat
  #undef SetPWM6Dat
  #undef SetPWM7Dat
#endif // defined

#if defined(CH548N)
  #undef SetPWM0Dat
  #undef SetPWM2Dat
  #undef SetPWM3Dat
  #undef SetPWM4Dat
  #undef SetPWM5Dat
  #undef SetPWM6Dat
  #undef SetPWM7Dat
#endif // defined

#if defined(CH549F)
  #undef SetPWM2Dat
  #undef SetPWM4Dat
  #undef SetPWM5Dat
#endif // defined

#define PWM0PinAlter( )  (PIN_FUNC |= bPWM0_PIN_X)                            // PWM mapping pin P1.5

#define ForceClearPWMFIFO( )  (PWM_CTRL |= bPWM_CLR_ALL)                      // Empty PWM count and FIFO
#define CancelClearPWMFIFO( ) (PWM_CTRL &= ~bPWM_CLR_ALL)                     // Stop clearing PWM count and FIFO

#define PWM0OutEnable()  (PWM_CTRL  |=  bPWM0_OUT_EN)                         // PWM0 output enable
#define PWM1OutEnable()  (PWM_CTRL  |=  bPWM1_OUT_EN)                         // PWM1 output enable
#define PWM2OutEnable()  (PWM_CTRL2 |=  bPWM2_OUT_EN)                         // PWM2 output enable
#define PWM3OutEnable()  (PWM_CTRL2 |=  bPWM3_OUT_EN)                         // PWM3 output enable
#define PWM4OutEnable()  (PWM_CTRL2 |=  bPWM4_OUT_EN)                         // PWM4 output enable
#define PWM5OutEnable()  (PWM_CTRL2 |=  bPWM5_OUT_EN)                         // PWM5 output enable
#define PWM6OutEnable()  (PWM_CTRL2 |=  bPWM6_OUT_EN)                         // PWM6 output enable
#define PWM7OutEnable()  (PWM_CTRL2 |=  bPWM7_OUT_EN)                         // PWM7 output enable

#if defined(CH549G) || defined(CH548G)
  #undef PWM2OutEnable
  #undef PWM4OutEnable
  #undef PWM5OutEnable
  #undef PWM6OutEnable
  #undef PWM7OutEnable
#endif // defined

#if defined(CH549N)
  #undef PWM0OutEnable
  #undef PWM2OutEnable
  #undef PWM3OutEnable
  #undef PWM4OutEnable
  #undef PWM5OutEnable
  #undef PWM6OutEnable
  #undef PWM7OutEnable
#endif // defined

#if defined(CH549F)
  #undef PWM2OutEnable
  #undef PWM4OutEnable
  #undef PWM5OutEnable
#endif // defined


#define PWM0OutDisable() (PWM_CTRL &=  ~bPWM1_OUT_EN)                         // PWM0 output disable
#define PWM1OutDisable() (PWM_CTRL &=  ~bPWM1_OUT_EN)                         // PWM1 output disable
#define PWM2OutDisable() (PWM_CTRL2 &= ~bPWM2_OUT_EN)                         // PWM2 output disable
#define PWM3OutDisable() (PWM_CTRL2 &= ~bPWM3_OUT_EN)                         // PWM3 output disable
#define PWM4OutDisable() (PWM_CTRL2 &= ~bPWM4_OUT_EN)                         // PWM4 output disable
#define PWM5OutDisable() (PWM_CTRL2 &= ~bPWM5_OUT_EN)                         // PWM5 output disable
#define PWM6OutDisable() (PWM_CTRL2 &= ~bPWM6_OUT_EN)                         // PWM6 output disable
#define PWM7OutDisable() (PWM_CTRL2 &= ~bPWM7_OUT_EN)                         // PWM7 output disable

#if defined(CH549G) || defined(CH548G)
  #undef PWM2OutDisable
  #undef PWM4OutDisable
  #undef PWM5OutDisable
  #undef PWM6OutDisable
  #undef PWM7OutDisable
#endif // defined

#if defined(CH548N)
  #undef PWM0OutDisable
  #undef PWM2OutDisable
  #undef PWM3OutDisable
  #undef PWM4OutDisable
  #undef PWM5OutDisable
  #undef PWM6OutDisable
  #undef PWM7OutDisable
#endif // defined

#if defined(CH549F)
  #undef PWM2OutDisable
  #undef PWM4OutDisable
  #undef PWM5OutDisable
#endif // defined

#define PWMMode6Bit()    (PWM_CTRL  |=  bPWM_MOD_6BIT)                         // PWM 6-bit data, PWM cycle is 64
#define PWMMode8Bit()    (PWM_CTRL  &= ~bPWM_MOD_6BIT)                         // PWM 8-bit data, and PWM cycle is 256 (default)


#define PWM0OutPolarHighAct() (PWM_CTRL &= ~bPWM0_POLAR)                       //PWM0 Low level by default, while active high.
#define PWM1OutPolarHighAct() (PWM_CTRL &= ~bPWM1_POLAR)                       //PWM1 Low level by default, while active high.
#define PWM0OutPolarLowAct()  (PWM_CTRL |= bPWM0_POLAR)                        //PWM0 High level by default while active low
#define PWM1OutPolarLowAct()  (PWM_CTRL |= bPWM1_POLAR)                        //PWM1 High level by default while active low

//PWM Interrupt
#define PWMInterruptEnable()  (PWM_CTRL |= bPWM_IF_END | bPWM_IE_END, IE_PWMX = 1)
#define PWMInterruptDisable() (IE_PWMX = 0)
