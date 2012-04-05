#include "tick.h"
#include "control/motor.h"
#include "estop.h"
#include <avr/interrupt.h>

#define TIMOVFVEC TCF1_OVF_vect

void tick_init() {
	TCF1.CTRLA = TC_CLKSEL_DIV8_gc;	// 32 MHz clock / 8 = 4 MHz timer
	TCF1.PER = TICK_TIMMAX;			// TICK_TIMHZ / (TICK_TIMHZ / TICK_HZ) = TICK_HZ timer
	tick_resume();
}

void tick_resume() {
	TCF1.INTCTRLA = TC_OVFINTLVL_LO_gc; // overflow interrupt enabled at low priority, for running the ticks
}

void tick_suspend() {
	TCF1.INTCTRLA = 0;
}

ISR(TIMOVFVEC) {
	if (estop_check()) {		// If estopped, run estops
		motor_estop();
	} else {					// Else, run normal ticks
		motor_tick();
	}	
}
