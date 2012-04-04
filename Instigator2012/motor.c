#include "motor.h"
#include <avr/io.h>
#include <stdio.h>


static const int ctrlpins_mask = 0xFF;


static const int PWMpins_mask = 0x0F;



static const uint8_t port[4] = {2, 3, 1, 0}; // LRDF

void motor_init() {
	PORTK.DIRSET = ctrlpins_mask;
	PORTF.DIRSET = PWMpins_mask;

	TCF0.CTRLA = TC_CLKSEL_DIV1_gc; // no divider means timer runs at 32Mhz
	TCF0.CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_SS_gc; // enable all capture compares, single slope PWM
	TCF0.PER = 1023; // 32Mhz / ~1024 = 31.25 khz PWM freq

	//motor_setPWM(MOTOR_FAN, motor_maxPWM);
	motor_setPWM(MOTOR_RIGHT, 512);
	motor_setPWM(MOTOR_LEFT, 512);
}

void motor_setPWM(uint8_t mot, int16_t PWM) {
	uint8_t in1pin_mask = _BV(2*port[mot]);
	uint8_t in2pin_mask = in1pin_mask << 1;
	


	if (PWM == 0) {
		PORTK.OUTCLR = in1pin_mask | in2pin_mask;
		(&TCF0.CCABUF)[port[mot]] = 0;
	} else if (PWM > 0) {
		PORTK.OUTCLR = in2pin_mask;
		PORTK.OUTSET = in1pin_mask;
		(&TCF0.CCABUF)[port[mot]] = PWM;
	} else {
		PORTK.OUTCLR = in1pin_mask;
		PORTK.OUTSET = in2pin_mask;
		(&TCF0.CCABUF)[port[mot]] = -PWM;
	}
}

void motor_allOff() {
	for (int i=0; i<motor_count; i++) {
		(&TCF0.CCABUF)[i] = 0;
	}
}

int16_t motor_getPWM(uint8_t mot) {
	int16_t PWM = (&TCF0.CCA)[port[mot]]; // CCx registers are also adjacent

	uint8_t in1pin_mask = _BV(2*port[mot]);
	if (!(PORTK.IN & in1pin_mask))
		PWM = -PWM;

	return PWM;
}

void motor_estop() {
	motor_setPWM(MOTOR_RIGHT, 0);
	motor_setPWM(MOTOR_LEFT, 0);
}

void motor_tick() {
	if (estop_check()) {	// If we have an ESTOP
		motor_setPWM(MOTOR_RIGHT, 0);
		motor_setPWM(MOTOR_LEFT, 0);
	} else {				// Normal Operations
		motor_setPWM(MOTOR_LEFT, 1000);
		motor_setPWM(MOTOR_RIGHT, 1000);
	}
}