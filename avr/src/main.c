#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "config.h"

void hardware_init(void)
{
    //Initialise hardware
    sei();
}

int main (void)
{
    hardware_init();
    
    for (;;) {
        //Mainloop
    }
}
