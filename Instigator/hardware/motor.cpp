#include "control/tick.h"
#include "debug/debug.h"
#include "hardware/motor.h"

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdarg.h>
#include <string.h>
#include <avr/pgmspace.h>

static PORT_t &motor_port = PORTF;
static const int PWMpins_mask = _BV(0) | _BV(1) | _BV(2);

static PORT_t &motordir_port = PORTH;
static const int motordir_mask = _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7);

static const uint8_t port[3] = {0, 1, 2}; // RLW

static uint32_t tick_timer = 0;
static int16_t desPWML = 0;
static int16_t currPWML = 0;
static int16_t desPWMR = 0;
static int16_t currPWMR = 0;
static int16_t desPWMW = 0;
static int16_t currPWMW = 0;
static bool lramp = false;
static bool rramp = false;
static bool wramp = false;

void motor_init() {
	motor_port.DIRSET = PWMpins_mask;

	motordir_port.DIRSET = motordir_mask;

	TCF0.CTRLA = TC_CLKSEL_DIV1_gc; // no divider means timer runs at 32Mhz
	TCF0.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SS_gc; // enable all capture compares, single slope PWM
	TCF0.PER = 1023; // 32Mhz / ~1024 = 31.25 khz PWM freq
}

void motor_setPWM(uint8_t mot, int16_t PWM) {
	if (mot == MOTOR_LEFT) {
		currPWML = PWM;
	} else if (mot == MOTOR_RIGHT) {
		currPWMR = PWM;
	} else if (mot == MOTOR_WEEDWHACKER) {
		currPWMW = PWM;
	}

	if (PWM == 0) {
		(&TCF0.CCABUF)[port[mot]] = 0;
	} else if (PWM > 0) {
		(&TCF0.CCABUF)[port[mot]] = PWM;
		if (mot == MOTOR_LEFT || mot == MOTOR_RIGHT) {
			motordir_port.OUTCLR = _BV((port[mot]*2)+2);		// Set directional ports for forwards
			motordir_port.OUTSET = _BV((port[mot]*2)+3);
		}
	} else {
		(&TCF0.CCABUF)[port[mot]] = -PWM;
		if (mot == MOTOR_LEFT || mot == MOTOR_RIGHT) {
			motordir_port.OUTSET = _BV((port[mot]*2)+2);		// Set directional ports for backwards
			motordir_port.OUTCLR = _BV((port[mot]*2)+3);
		}
	}
}

void motor_allOff() {
	lramp = false;
	rramp = false;
	wramp = false;
	currPWML = 0;
	currPWMR = 0;
	currPWMW = 0;
	for (int i=0; i<motor_count; i++) {
		(&TCF0.CCABUF)[i] = 0;
	}
}

void motor_killMotor(uint8_t mot) {
	if (mot == MOTOR_LEFT) {
		lramp = false;
		currPWML = 0;
	} else if (mot == MOTOR_RIGHT) {
		rramp = false;
		currPWMR = 0;
	} else if (mot == MOTOR_WEEDWHACKER) {
		wramp = false;
		currPWMW = 0;
	}
	(&TCF0.CCABUF)[mot] = 0;
}

int16_t motor_getPWM(uint8_t mot) {
	int16_t PWM = (&TCF0.CCA)[port[mot]]; // CCx registers are also adjacent

	uint8_t dirpin_mask = _BV(2*port[mot] + 2);		// TODO: Test!!!
	if (!(PORTK.IN & dirpin_mask))
		PWM = -PWM;

	return PWM;
}

void motor_estop() {
	motor_allOff();
}

void motor_ramp(int16_t newPWML, int16_t newPWMR) {
	desPWML = newPWML;
	desPWMR = newPWMR;
	lramp = true;
	rramp = true;
}

void motor_startWhacker(int16_t newPWMW) {
	desPWMW = newPWMW;
	wramp = true;
}

void motor_goPWM(int16_t newPWML, int16_t newPWMR) {	// Set an RPM to lock hard on
	lramp = false;
	rramp = false;
	desPWML = newPWML;
	desPWMR = newPWMR;
	motor_setPWM(MOTOR_LEFT, desPWML);
	motor_setPWM(MOTOR_RIGHT, desPWMR);
}

void motor_off(uint8_t mot) {
	motor_setPWM(mot, 0);
}

void motor_tick() {
	if (lramp) {
		if (currPWML < desPWML) {
			motor_setPWM(MOTOR_LEFT, currPWML + 50);
		} else if (currPWML > desPWML) {
			motor_setPWM(MOTOR_LEFT, currPWML - 50);
		} else {
			lramp = false;
		}
	}
	if (rramp) {
		if (currPWMR < desPWMR) {
			motor_setPWM(MOTOR_RIGHT, currPWMR + 50);
		} else if (currPWMR > desPWMR) {
			motor_setPWM(MOTOR_RIGHT, currPWMR - 50);
		} else {
			rramp = false;
		}
	}
	if (wramp) {
		if (currPWMW < desPWMW) {
			motor_setPWM(MOTOR_WEEDWHACKER, currPWMW + 50);
		} else if (currPWMR > desPWMR) {
			motor_setPWM(MOTOR_WEEDWHACKER, currPWMW - 50);
		} else {
			wramp = false;
		}
	}
	tick_timer = tick_timer + 1;
}
