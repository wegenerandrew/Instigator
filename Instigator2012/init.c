#include "init.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void init(){
	PMIC.CTRL = PMIC_HILVLEN_bm;	//Enable High level interrupts
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;	//Enable Medium level interrupts
	PMIC.CTRL |= PMIC_LOLVLEN_bm;	//Enable Low level interrupts
	
	sei();							//Clears global mask

}