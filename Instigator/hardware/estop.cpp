#include "debug/debug.h"
#include "hardware/motor.h"
#include "hardware/estop.h"
#include "hardware/solenoid.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define SIGINT0VECT PORTH_INT0_vect

static PORT_t &estop_port = PORTH;
static const int estop_mask = _BV(1);

static volatile bool estop = false;

void estop_init() {
	estop_port.DIRSET  &= estop_mask;		 //Set pin 7 as input leave pwm pins alone
	estop_port.INTCTRL  = TC_OVFINTLVL_HI_gc;		// EStop set to High Priority
	estop_port.PIN7CTRL = PORT_ISC_RISING_gc | PORT_OPC_PULLUP_gc;		//Set pin 7 to be pulled up and interupt to occur on the falling edge
	estop_port.INT0MASK = estop_mask;  //Set pin 7 in port F to be part of an interrupt
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
	solenoid_kill();
	_delay_ms(10);
	debug_setLED(ESTOP_LED, true);
	cli();				// Disable all interrupts
	while (true) {
		if (!estop_checkPin()) {
			estop_reboot();
		}
	}
}

void estop_reboot() {
	estop = true;
	motor_estop();
	solenoid_kill();
	_delay_ms(10);
	debug_setLED(ESTOP_LED, true);
	cli();				// Disable all interrupts
	CPU_CCP = CCP_IOREG_gc;		// give change protection signature
	RST.CTRL = RST_SWRST_bm;	// software reset processor
}

bool estop_checkPin() {
	uint8_t estop_check = estop_port.IN;
	estop_check = estop_check >> 7;
	if (estop_check == estop_mask) {	// TODO May break estop!!
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


