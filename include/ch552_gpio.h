#pragma once

// CH552 GPIO functions

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

// Port 1
#define P1SetModeINPUT(mask)         ((P1_MOD_OC &=  ~(mask)),  (P1_DIR_PU &=   ~(mask)))
#define P1SetModePP_OUTPUT(mask)     ((P1_MOD_OC &=  ~(mask)),  (P1_DIR_PU |=    (mask)))
#define P1SetModeOD_NO_PULLUP(mask)  ((P1_MOD_OC |=   (mask)),  (P1_DIR_PU &=  ~ (mask)))
#define P1SetModeOD_PULLUP(mask)     ((P1_MOD_OC |=   (mask)),  (P1_DIR_PU |=    (mask)))

// Port 3
#define P3SetModeINPUT(mask)         ((P3_MOD_OC &=  ~(mask)),  (P3_DIR_PU &=   ~(mask)))
#define P3SetModePP_OUTPUT(mask)     ((P3_MOD_OC &=  ~(mask)),  (P3_DIR_PU |=    (mask)))
#define P3SetModeOD_NO_PULLUP(mask)  ((P3_MOD_OC |=   (mask)),  (P3_DIR_PU &=  ~ (mask)))
#define P3SetModeOD_PULLUP(mask)     ((P3_MOD_OC |=   (mask)),  (P3_DIR_PU |=    (mask)))

