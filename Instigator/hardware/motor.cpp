#include "hardware/motor.h"
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdarg.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "debug.h"
#include "tick.h"


static const int ctrlpins_mask = 0xFF;


static const int PWMpins_mask = 0x0F;

static PORT_t &motordir_port = PORTD;
static const int motordir_mask = _BV(0) | _BV(1);

static const uint8_t port[4] = {0, 1, 2, 3}; // LRDF

static uint32_t tick_timer = 0;
static int16_t desPWML = 0;
static int16_t currPWML = 0;
static int16_t desPWMR = 0;
static int16_t currPWMR = 0;
static bool lramp = false;
static bool rramp = false;

void motor_init() {
	PORTK.DIRSET = ctrlpins_mask;
	PORTF.DIRSET = PWMpins_mask;

	motordir_port.DIRSET = motordir_mask;

	TCF0.CTRLA = TC_CLKSEL_DIV1_gc; // no divider means timer runs at 32Mhz
	TCF0.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SS_gc; // enable all capture compares, single slope PWM
	TCF0.PER = 1023; // 32Mhz / ~1024 = 31.25 khz PWM freq
}

void motor_setPWM(uint8_t mot, int16_t PWM) {
	uint8_t in1pin_mask = _BV(2*port[mot]);
	uint8_t in2pin_mask = in1pin_mask << 1;
	
	if (mot == MOTOR_LEFT) {
		currPWML = PWM;
	} else if (mot == MOTOR_RIGHT) {
		currPWMR = PWM;
	}

	if (PWM == 0) {
		PORTK.OUTCLR = in1pin_mask | in2pin_mask;
		(&TCF0.CCABUF)[port[mot]] = 0;
	} else if (PWM > 0) {
		PORTK.OUTCLR = in2pin_mask;
		PORTK.OUTSET = in1pin_mask;
		(&TCF0.CCABUF)[port[mot]] = PWM;

		motordir_port.OUTCLR = _BV(mot-2);		// Set directional port forwards
	} else {
		PORTK.OUTCLR = in1pin_mask;
		PORTK.OUTSET = in2pin_mask;
		(&TCF0.CCABUF)[port[mot]] = -PWM;

		motordir_port.OUTSET = _BV(mot-2);		// Set directional port backwards
	}
}

void motor_allOff() {
	lramp = false;
	rramp = false;
	currPWML = 0;
	currPWMR = 0;
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
	}
	(&TCF0.CCABUF)[mot] = 0;
}

int16_t motor_getPWM(uint8_t mot) {
	int16_t PWM = (&TCF0.CCA)[port[mot]]; // CCx registers are also adjacent

	uint8_t in1pin_mask = _BV(2*port[mot]);
	if (!(PORTK.IN & in1pin_mask))
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
			motor_setPWM(MOTOR_LEFT, currPWML + 100);
		} else if (currPWML > desPWML) {
			motor_setPWM(MOTOR_LEFT, currPWML - 100);
		} else {
			lramp = false;
		}
	}
	if (rramp) {
		if (currPWMR < desPWMR) {
			motor_setPWM(MOTOR_RIGHT, currPWMR + 100);
		} else if (currPWMR > desPWMR) {
			motor_setPWM(MOTOR_RIGHT, currPWMR - 100);
		} else {
			rramp = false;
		}
	}
	tick_timer = tick_timer + 1;
}
