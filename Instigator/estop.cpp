
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "hardware/motor.h"
#include "debug.h"

#define SIGINT0VECT PORTF_INT0_vect

static volatile bool estop = false;
// TODO: redo estop
void estop_init(){
	PORTF.DIRSET  &= _BV(7);		 //Set pin 7 as input leave pwm pins alone
	PORTF.INTCTRL  = TC_OVFINTLVL_HI_gc;		// EStop set to High Priority
	PORTF.PIN7CTRL = PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc;		//Set pin 7 to be pulled up and interupt to occur on the falling edge
	PORTF.INT0MASK = _BV(7);  //Set pin 7 in port F to be part of an interrupt
}

_Bool estop_check() {
	return estop;
}

void estop_killall() {
	estop = true;
	motor_estop();
	_delay_ms(10);
	debug_setErrorLED();
	cli();				// Disable all interrupts
//	PMIC.CTRL = 0x00;	// Disable High, Medium, and Low level interrupts
//	CPU_CCP = CCP_IOREG_gc;		// give change protection signature
//	RST.CTRL = RST_SWRST_bm;	// software reset processor
	while(true) { }
}

ISR(SIGINT0VECT){
	PORTF.INTFLAGS= 0x01; //Clear the flag by writing a one to it 
	estop_killall();
}


