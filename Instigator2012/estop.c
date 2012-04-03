
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define SIGINT0VECT PORTF_INT0_vect

static const int boardled_mask = _BV(1);

void estop_init(){
	PORTF.DIRSET &=  0x0F;		 //Set pin 7 as input leave pwm pins alone
	PORTF.INTCTRL	=0x03;
	PORTF.PIN7CTRL = PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc;		//Set pin 7 to be pulled up and interupt to occur on the falling edge
	PORTF.INT0MASK = 0x80;  //Set pin 7 in port F to be part of an interrupt 
	
	
	
}

ISR(SIGINT0VECT){
	PORTF.INTFLAGS= 0x01; //Clear the flag by writing a one to it 
	
	PORTR.OUTCLR=boardled_mask; //Turn on the debug led, low true
	_delay_ms(10000);
	
}