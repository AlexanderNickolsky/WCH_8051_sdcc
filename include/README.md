## Conventions

8051 chips from WCH are all very similar, but they are in fact different.
So the best practice here is to have different sets of files for all those parts (as we have no C++ compiler).
They are named according to parts: ch552.h, ch552_pwm, ch552_gpio, ch549.h, ch549_gpio, etc.
For parts variations such as different packages there can be some features present or
absent. 

#define your package type before #including these files to get compiler errors when you are trying
to use non-existent features of the chips. E.g:

```
#define CH549G

#include <ch549.h>
#include <ch549_gpio.h>
#include <ch549_debug.h>
#include <ch549_pwm.h>

// here both CH549 and CH549G are #define-d. 

...

    PWM20OutEnable(); // error - CH549G has no PWM2 output


