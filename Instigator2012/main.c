/*
 * AVRGCC1.c
 *
 * Created: 4/2/2012 6:24:44 PM
 *  Author: Andrew
 */ 

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include "motor.h"
#include "init.h"
#include "estop.h"


int main(void)
{
	init();
	init_all();
	_delay_ms(1000);
    while(1)
    {
    }
}