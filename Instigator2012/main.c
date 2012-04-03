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

static const int boardled_mask = _BV(1);


void blink_init(){
	PORTR.DIRSET = boardled_mask;
}
int main(void)
{
	init();
	estop_init();
	blink_init();
	motor_init();
    while(1)
    {
		
        PORTR.OUTSET= boardled_mask;
		_delay_ms(1000);
		PORTR.OUTCLR= boardled_mask;
		_delay_ms(1000);
    }
}