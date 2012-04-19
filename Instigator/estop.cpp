
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "hardware/motor.h"
#include "debug.h"
#include "estop.h"
#include "control/solenoidcontrol.h"

#define SIGINT0VECT PORTF_INT0_vect

static volatile bool estop = false;

void estop_init(){
	PORTF.DIRSET  &= _BV(7);		 //Set pin 7 as input leave pwm pins alone
	PORTF.INTCTRL  = TC_OVFINTLVL_HI_gc;		// EStop set to High Priority
	PORTF.PIN7CTRL = PORT_ISC_RISING_gc | PORT_OPC_PULLUP_gc;		//Set pin 7 to be pulled up and interupt to occur on the falling edge
	PORTF.INT0MASK = _BV(7);  //Set pin 7 in port F to be part of an interrupt
}

void estop_initCheck() {
	if (estop_checkPin()) {
		estop_killall();
	}
}

_Bool estop_check() {
	return estop;
}

void estop_killall() {
	estop = true;
	motor_estop();
	solenoidcontrol_kill();
	_delay_ms(10);
	debug_setErrorLED();
	cli();				// Disable all interrupts
//	PMIC.CTRL = 0x00;	// Disable High, Medium, and Low level interrupts
//	CPU_CCP = CCP_IOREG_gc;		// give change protection signature
//	RST.CTRL = RST_SWRST_bm;	// software reset processor
	while (true) {
		if (!estop_checkPin()) {
			estop_reboot();
		}
	}
}

void estop_reboot() {
	estop = true;
	motor_estop();
	solenoidcontrol_kill();
	_delay_ms(10);
	debug_setErrorLED();
	cli();				// Disable all interrupts
	CPU_CCP = CCP_IOREG_gc;		// give change protection signature
	RST.CTRL = RST_SWRST_bm;	// software reset processor
}

bool estop_checkPin() {
	uint8_t estop_check = PORTF.IN;
	estop_check = estop_check >> 7;
	if (estop_check == 1) {
		return true;
	} else {
		return false;
	}
}

ISR(SIGINT0VECT){
	PORTF.INTFLAGS= 0x01; //Clear the flag by writing a one to it
	if (estop_checkPin()) {
		estop_killall();
	}
}


