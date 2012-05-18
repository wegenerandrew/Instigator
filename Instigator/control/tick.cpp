#include "control/motorcontrol.h"
#include "control/magfollow.h"
#include "control/tick.h"
#include "control/odometry.h"
#include "debug/debug.h"
#include "hardware/estop.h"
#include "hardware/gps.h"
#include "hardware/motor.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

#define TIMOVFVEC TCF1_OVF_vect

static TC1_t &tim = TCF1;
#define TIMCCAVEC TCF1_CCA_vect

static volatile uint32_t tickcount;
static volatile uint16_t ticklength;


void tick_init() {
	tim.CTRLA = TC_CLKSEL_DIV64_gc; 	// 32Mhz / 64 = .5 Mhz timer (TICK_TIMHZ == 5E5)
	tim.CTRLB = TC0_CCAEN_bm; 			// enable capture compare A
	tim.PER = TICK_TIMMAX; 				// TICK_TIMHZ / (TICK_TIMHZ / TICK_HZ) = TICK_HZ timer
	tim.CCABUF = 200; 					// 200 / .5Mhz = 400us CCA (for linesensor)
	tick_resume(); 						// enable interrupts
}


void tick_wait() {
	uint32_t t = tickcount;
	while (t == tickcount) { }
}

void tick_suspend() {
	tim.INTCTRLA = 0;
	tim.INTCTRLB = 0;
}
void tick_resume() {
	tim.INTCTRLB = TC_CCAINTLVL_HI_gc; // capture compare A interrupt enabled at high priority, because the tick may not be completed before this goes off
	tim.INTCTRLA = TC_OVFINTLVL_LO_gc; // overflow interrupt enabled at low priority, for running the ticks
}


uint16_t tick_getTimer() {
	return tim.CNT;
}

uint32_t tick_getCount() {
	return tickcount;
}

uint16_t tick_getLength() { // in uS
	return (uint16_t)((uint32_t)ticklength * TICK_US / TICK_TIMMAX);
}

ISR(TIMOVFVEC) {
	uint16_t start = tim.CNT;
	debug_setLED(TICK_LED, true);
	tickcount++;

	magfollow_tick();
	motor_tick();
	motorcontrol_tick();
	gps_tick();
	debug_tick();
	odometry_tick();

	debug_setLED(TICK_LED, false);
	ticklength = tim.CNT - start;
}

ISR(TIMCCAVEC) {
}

