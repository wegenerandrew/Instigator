#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include "hardware/motor.h"
#include "init.h"
#include "estop.h"
#include "controlpanel.h"
#include "debug.h"


int main(void)
{
	init();
    while (true) {
		controlpanel();
    }
}
