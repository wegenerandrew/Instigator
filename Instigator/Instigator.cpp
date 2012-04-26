#include "debug/controlpanel.h"
#include "debug/debug.h"
#include "hardware/estop.h"
#include "hardware/motor.h"
#include "init.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>

int main(void)
{
	init();
    while (true) {
		controlpanel();
    }
}
