#include "init.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "motor.h"
#include "estop.h"
#include "debug.h"

void init(){
	PMIC.CTRL = PMIC_HILVLEN_bm;	//Enable High level interrupts
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;	//Enable Medium level interrupts
	PMIC.CTRL |= PMIC_LOLVLEN_bm;	//Enable Low level interrupts
	
	sei();							//Clears global mask
}

void init_all() {
	estop_init();
	debug_init();
	motor_init();
}