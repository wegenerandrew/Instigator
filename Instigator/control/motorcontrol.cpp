#include "control/motorcontrol.h"
#include "control/pid.h"
#include "debug.h"
#include "hardware/encoder.h"
#include "hardware/motor.h"
#include "tick.h"
#include "util.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdint.h>

static PIDGains pidgains = {0, 0, 0, 0};

volatile static bool enabled;
volatile static bool debug;

// Motor Directional Port
static PORT_t &motordir_port = PORTD;
static const int motordir_mask = _BV(0) | _BV(1) | _BV(2) | _BV(3);

struct MotorInfo {
	PIDState pid;	// State of PID loop
	float m;		// Slope for open-loop lookup
	float b;		// Intercept for open-loop lookup
	volatile float RPS_desired;
	volatile float RPS_measured;
	volatile uint16_t prev_enc;		// Used to find motor velocity
};

static MotorInfo motinfo[motorcontrol_count];

void motorcontrol_init() {
	motinfo[0].m = 1 / motor_maxPWM;
	motinfo[0].b = 0 / motor_maxPWM;
	motinfo[1].m = 1 / motor_maxPWM;
	motinfo[1].b = 0 / motor_maxPWM;

	motordir_port.DIRSET = motordir_mask;
}

float motorcontrol_getRPS(int motnum) {
	return motinfo[motnum].RPS_measured;
}

float motorcontrol_getRPSDesired(int motnum) {
	return motinfo[motnum].RPS_desired;
}

void motorcontrol_setRPS(int motnum, float RPS) {
	MotorInfo &mot = motinfo[motnum];

	if (sign(mot.RPS_desired) != sign(RPS))
		mot.pid.sum = 0;

	mot.RPS_desired = RPS;

	// Set directional pins:
	uint8_t pina = motnum*2;
	uint8_t pinb = pina + 1;
	if (RPS >= 0) {
		motordir_port.OUTSET = _BV(pina);
		motordir_port.OUTCLR = _BV(pinb);
	} else {
		motordir_port.OUTCLR = _BV(pina);
		motordir_port.OUTSET = _BV(pinb);
	}
}

void motorcontrol_setEnabled(bool new_enabled) {
	if (enabled == new_enabled) {
		return;
	}

	if (new_enabled) {
		for (int i = 0; i < motorcontrol_count; i++) {
			motinfo[i].prev_enc = 1;//enc_get(i);
		}
		enabled = true;
	} else {
		enabled = false;
		for (int i = 0; i < motorcontrol_count; i++) {
			motor_setPWM(i, 0);
		}
	}
}

void motorcontrol_setGains(const PIDGains &newpidgains) {
	tick_suspend();
	pidgains = newpidgains;
	tick_resume();
}

PIDGains motorcontrol_getGains() {
	return pidgains;
}

void motorcontrol_setDebug(bool newdebug) {
	debug = newdebug;
}

void motorcontrol_tick() {
	if (!enabled) {
		return;
	}
	
	for (int motnum = 0; motnum < motorcontrol_count; motnum++) {	// For each motor
		MotorInfo &mot = motinfo[motnum]; // get its motor information

		uint16_t enc = 1;//enc_get(motnum); // read the amount the motor traveled since the last update
		int16_t diff = 1;//enc_diff(enc, mot.prev_enc); // compute the difference
		mot.RPS_measured = 1;//diff/enc_per_rotation*TICK_HZ; 								// compute the rotations per second
		mot.prev_enc = enc; // save the encoder position

		PIDDebug piddebug;
		float error = mot.RPS_desired - mot.RPS_measured; // compute error
		float out = pid_update(mot.pid, pidgains, error, 1.0/TICK_HZ, &piddebug); // update pid
		out += sign(mot.RPS_desired)*(mot.m*fabs(mot.RPS_desired) + mot.b); // compute feedforward
		
		if (debug && motnum == 0)
			pid_printDebug(out, error, piddebug);
			
		if (out > 1) // enforce saturation
			out = 1;
		else if (out < -1)
			out = -1;
		
		int16_t pwm = motor_getPWM(motnum);
		if (out > 0) {
			if (pwm < 0)
				out = 0;
		} else {
			if (pwm > 0)
				out = 0;
		}
		motor_setPWM(motnum, (int16_t)(out*motor_maxPWM)); // convert output to PWM, set it to the motor
		
	}
}


