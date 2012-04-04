#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include "motor.h"
#include "init.h"
#include "estop.h"
#include "controlpanel.h"


int main(void)
{
	init();
	_delay_ms(1000);
    while(1)
    {
		controlpanel();
    }
}