#pragma once

// CH549 GPIO functions

// #define package type before including this file to enable some checks for port/pin existence
// #define CH549L or CH548L or default - LQFP48
// #define CH549F - QFN28
// #define CH549G or CH548G - SOP16
// #define CH548N - SOP8

// Mode:
// PP = Push-Pull
// OD = Open Drain
// Direction:
// if Mode == PP:
//    0: Input, 1: Output
// if Mode == OD:
//    0: Pull-up resistor disabled, 1: Pull-up resistor enabled

// Available modes

// MODE_INPUT        0 // High impedance input mode, mask has no pull-up resistor
// MODE_PP_OUTPUT    1 // Push-pull output mode, has symmetrical drive capability which can output or absorb large current
// MODE_OD_NO_PULLUP 2 // Open-drain output, support high impedance input, mask has no pull-up resistor
// MODE_OD_PULLUP    3 // Quasi-bidirectional mode (standard 8051), open-drain output, support input, mask has pull-up resistor

// Port 0
#define P0SetModeINPUT(mask)         ((P0_MOD_OC &=  ~(mask)),  (P0_DIR_PU &=   ~(mask)))
#define P0SetModePP_OUTPUT(mask)     ((P0_MOD_OC &=  ~(mask)),  (P0_DIR_PU |=    (mask)))
#define P0SetModeOD_NO_PULLUP(mask)  ((P0_MOD_OC |=   (mask)),  (P0_DIR_PU &=  ~ (mask)))
#define P0SetModeOD_PULLUP(mask)     ((P0_MOD_OC |=   (mask)),  (P0_DIR_PU |=    (mask)))

#if defined(CH549G) || defined(CH548G) || defined(CH548N)
#undef P0SetModeINPUT
#undef P0SetModePP_OUTPUT
#undef P0SetModeOD_NO_PULLUP
#undef P0SetModeOD_PULLUP
#endif // defined

// Port 1
#define P1SetModeINPUT(mask)         ((P1_MOD_OC &=  ~(mask)),  (P1_DIR_PU &=   ~(mask)))
#define P1SetModePP_OUTPUT(mask)     ((P1_MOD_OC &=  ~(mask)),  (P1_DIR_PU |=    (mask)))
#define P1SetModeOD_NO_PULLUP(mask)  ((P1_MOD_OC |=   (mask)),  (P1_DIR_PU &=  ~ (mask)))
#define P1SetModeOD_PULLUP(mask)     ((P1_MOD_OC |=   (mask)),  (P1_DIR_PU |=    (mask)))

// Port 2
#define P2SetModeINPUT(mask)         ((P2_MOD_OC &=  ~(mask)),  (P2_DIR_PU &=   ~(mask)))
#define P2SetModePP_OUTPUT(mask)     ((P2_MOD_OC &=  ~(mask)),  (P2_DIR_PU |=    (mask)))
#define P2SetModeOD_NO_PULLUP(mask)  ((P2_MOD_OC |=   (mask)),  (P2_DIR_PU &=  ~ (mask)))
#define P2SetModeOD_PULLUP(mask)     ((P2_MOD_OC |=   (mask)),  (P2_DIR_PU |=    (mask)))

// Port 3
#define P3SetModeINPUT(mask)         ((P3_MOD_OC &=  ~(mask)),  (P3_DIR_PU &=   ~(mask)))
#define P3SetModePP_OUTPUT(mask)     ((P3_MOD_OC &=  ~(mask)),  (P3_DIR_PU |=    (mask)))
#define P3SetModeOD_NO_PULLUP(mask)  ((P3_MOD_OC |=   (mask)),  (P3_DIR_PU &=  ~ (mask)))
#define P3SetModeOD_PULLUP(mask)     ((P3_MOD_OC |=   (mask)),  (P3_DIR_PU |=    (mask)))

#if defined(CH548N)
#undef P3SetModeINPUT
#undef P3SetModePP_OUTPUT
#undef P3SetModeOD_NO_PULLUP
#undef P3SetModeOD_PULLUP
#endif // defined


// Port 4
#define P4SetModeINPUT(mask)         ((P4_MOD_OC &=  ~(mask)),  (P4_DIR_PU &=   ~(mask)))
#define P4SetModePP_OUTPUT(mask)     ((P4_MOD_OC &=  ~(mask)),  (P4_DIR_PU |=    (mask)))
#define P4SetModeOD_NO_PULLUP(mask)  ((P4_MOD_OC |=   (mask)),  (P4_DIR_PU &=  ~ (mask)))
#define P4SetModeOD_PULLUP(mask)     ((P4_MOD_OC |=   (mask)),  (P4_DIR_PU |=    (mask)))

#if defined(CH549G) || defined(CH548G) || defined(CH548N)
#undef P4SetModeINPUT
#undef P4SetModePP_OUTPUT
#undef P4SetModeOD_NO_PULLUP
#undef P4SetModeOD_PULLUP
#endif // defined

// Port 5
#define P5SetBits(mask)   (P5 |=  (mask))
#define P5SetBit(mask)     (P5 |=  _BV(mask))
#define P5ResetBits(mask) (P5 &=  ~(mask))
#define P5ResetBit (mask)  (P5 &=  ~ _BV(mask))

