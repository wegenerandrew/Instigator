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
static const int PWMpins_mask = _BV(0) | _BV(1);

static PORT_t &motordir_port = PORTH;
static const int motordir_mask = _BV(2) | _BV(3) | _BV(4) | _BV(5);

static const uint8_t port[2] = {0, 1}; // RL

static uint32_t tick_timer = 0;
static int16_t desPWML = 0;
static int16_t currPWML = 0;
static int16_t desPWMR = 0;
static int16_t currPWMR = 0;
static bool lramp = false;
static bool rramp = false;
static const int ramping_factor = 10;

void motor_init() {
	motor_port.DIRSET = PWMpins_mask;

	motordir_port.DIRSET = motordir_mask;

	TCF0.CTRLA = TC_CLKSEL_DIV1_gc; // no divider means timer runs at 32Mhz
	TCF0.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SS_gc; // enable all capture compares, single slope PWM
	TCF0.PER = 1023; // 32Mhz / ~1024 = 31.25 khz PWM freq
}

void motor_setPWM(Motor mot, int16_t PWM) {
	if (mot == MOTOR_LEFT) {
		currPWML = PWM;
	} else if (mot == MOTOR_RIGHT) {
		currPWMR = PWM;
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

void motor_goPWM(int16_t newPWML, int16_t newPWMR) {	// Set an RPM to lock hard on
	lramp = false;
	rramp = false;
	desPWML = newPWML;
	desPWMR = newPWMR;
	motor_setPWM(MOTOR_LEFT, desPWML);
	motor_setPWM(MOTOR_RIGHT, desPWMR);
}

int16_t motor_getPWM(Motor mot) {
	int16_t PWM = (&TCF0.CCA)[port[mot]]; // CCx registers are also adjacent

	uint8_t dirpin_mask = _BV(2*port[mot] + 2);		// TODO: Test!!!
	if (((PORTK.IN << (4 - 2*port[mot])) >> 6) == 0x01) //xx01xx is backwards
		PWM = -PWM;

	return PWM;
}

void motor_ramp(int16_t newPWML, int16_t newPWMR) {
	desPWML = newPWML;
	desPWMR = newPWMR;
	lramp = true;
	rramp = true;
}

void motor_rampMotor(Motor mot, int16_t newPWM) {
	if (mot == MOTOR_LEFT) {
		desPWML = newPWM;
		lramp = true;
	} else if (mot == MOTOR_RIGHT) {
		desPWMR = newPWM;
		rramp = true;
	}
}

void motor_off(Motor mot) {		// Sets specified Motor to 0 PWM
	motor_setPWM(mot, 0);
}

void motor_allOff() {			// Disables all motors
	lramp = false;
	rramp = false;
	currPWML = 0;
	currPWMR = 0;
	for (int i=0; i<motor_count; i++) {
		(&TCF0.CCABUF)[i] = 0;		// TODO: name
	}
}

void motor_killMotor(Motor mot) {
	if (mot == MOTOR_LEFT) {
		lramp = false;
		currPWML = 0;
	} else if (mot == MOTOR_RIGHT) {
		rramp = false;
		currPWMR = 0;
	}
	(&TCF0.CCABUF)[mot] = 0;
}

void motor_estop() {				// Estops all motors
	motor_allOff();
}

void motor_tick() {
	if (lramp) {
		if (currPWML < desPWML) {
			motor_setPWM(MOTOR_LEFT, currPWML + ramping_factor);
		} else if (currPWML > desPWML) {
			motor_setPWM(MOTOR_LEFT, currPWML - ramping_factor);
		} else {
			lramp = false;
		}
	}
	if (rramp) {
		if (currPWMR < desPWMR) {
			motor_setPWM(MOTOR_RIGHT, currPWMR + ramping_factor);
		} else if (currPWMR > desPWMR) {
			motor_setPWM(MOTOR_RIGHT, currPWMR - ramping_factor);
		} else {
			rramp = false;
		}
	}
	tick_timer = tick_timer + 1;
}
