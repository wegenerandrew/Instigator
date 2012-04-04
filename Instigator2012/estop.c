
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "motor.h"
#include "debug.h"

#define SIGINT0VECT PORTF_INT0_vect

static volatile bool estop = false;

void estop_init(){
	PORTF.DIRSET &=  0x0F;		 //Set pin 7 as input leave pwm pins alone
	PORTF.INTCTRL	=0x03;
	PORTF.PIN7CTRL = PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc;		//Set pin 7 to be pulled up and interupt to occur on the falling edge
	PORTF.INT0MASK = 0x80;  //Set pin 7 in port F to be part of an interrupt 	
}

_Bool estop_check() {
	return estop;
}

void estop_killall() {
	motor_estop();
	debug_setErrorLED();
}

ISR(SIGINT0VECT){
	estop = true;
	PORTF.INTFLAGS= 0x01; //Clear the flag by writing a one to it 
	estop_killall();
}