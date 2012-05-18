#include "control/motorcontrol.h"
#include "control/pid.h"
#include "control/tick.h"
#include "debug/debug.h"
#include "hardware/encoder.h"
#include "hardware/motor.h"
#include "util.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/pgmspace.h>

static PIDGains pidgains = {1.1, 0, .01, 0};

volatile static bool enabled;
volatile static bool debug;

struct MotorInfo {
	PIDState pid;					// State of PID loop
	float m;						// Slope for open-loop lookup
	float b;						// Intercept for open-loop lookup
	volatile float RPS_desired;
	volatile float RPS_measured;
	volatile float RPS_final;
	volatile bool ramp;
	volatile uint8_t ramp_count;
	volatile uint16_t prev_encoder;		// Used to find motor velocity
};

static MotorInfo motinfo[motorcontrol_count];

static int cnt = 0;

void motorcontrol_init() {
	motinfo[0].m = 13.282 / motor_maxPWM;
	motinfo[0].b = 45.070 / motor_maxPWM;
	motinfo[1].m = 13.282 / motor_maxPWM;
	motinfo[1].b = 49.524 / motor_maxPWM;
	motinfo[0].ramp = false;
	motinfo[1].ramp = false;
}

float motorcontrol_getRPS(Motor motnum) {
	return motinfo[motnum].RPS_measured;
}

float motorcontrol_getRPSDesired(Motor motnum) {
	return motinfo[motnum].RPS_desired;
}

void motorcontrol_adjustRPS(Motor motnum, float RPS) {		// Run by motorcontrol_tick() to ramp RPS
	MotorInfo &mot = motinfo[motnum];

	if (sign(mot.RPS_desired) != sign(RPS)) {
		mot.pid.sum = 0;
	}
	mot.RPS_desired = RPS;
}

void motorcontrol_setRPS(Motor motnum, float RPS) {			// Run to start ramping RPS towards desired
/*	MotorInfo &mot = motinfo[motnum];

	mot.RPS_final = RPS;
	mot.RPS_desired = motorcontrol_getRPS(motnum);
	mot.ramp = true;*/
	motorcontrol_adjustRPS(motnum, RPS);
}

void motorcontrol_setEnabled(bool new_enabled) {
	if (enabled == new_enabled) {
		return;
	}

	if (new_enabled) {
		for (int i = 0; i < motorcontrol_count; i++) {
			motinfo[i].prev_encoder = encoder_get((Encoder)i);
		}
		enabled = true;
	} else {
		enabled = false;
		for (int i = 0; i < motorcontrol_count; i++) {
			motor_rampMotor((Motor)i, 0);
			//^^ was motor_setPWM();
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

/*	for (int motnum = 0; motnum < motorcontrol_count; motnum++) {
		MotorInfo &mot = motinfo[motnum];
		if (mot.ramp_count == 5) {
			mot.ramp_count = 0;
			if (mot.ramp) {										// If motor is ramping
				if ((mot.RPS_final - mot.RPS_desired) > .1) {
					motorcontrol_adjustRPS((Motor)motnum, mot.RPS_desired + .1);
				} else if ((mot.RPS_desired - mot.RPS_final) > .1) {
					motorcontrol_adjustRPS((Motor)motnum, mot.RPS_desired - .1);
				} else {										// Close enough to final (after ramping)
					motorcontrol_adjustRPS((Motor)motnum, mot.RPS_final);			// Set RPS_desired to what we wanted to ramp to
					mot.ramp = false;							// Done ramping
				}
			}
		}
		mot.ramp_count++;
	}*/

	for (int motnum = 0; motnum < motorcontrol_count; motnum++) {	// For each motor
		MotorInfo &mot = motinfo[motnum]; 							// get its motor information

		uint16_t encoder = encoder_get((Encoder)motnum);			// read the amount the motor traveled since the last update
		int16_t diff = encoder_diff(encoder, mot.prev_encoder); 	// compute the difference
		mot.RPS_measured = (diff/ticks_per_rotation)*TICK_HZ; 		// compute the rotations per second
		mot.prev_encoder = encoder; 								// save the encoder position
/*
		if (cnt > 5) {
			if (motnum == 0) {
				printf_P(PSTR("L: Desired: %8f, Measured: %8f"), mot.RPS_desired, mot.RPS_measured);
			} else {
				cnt == 0;
				printf_P(PSTR("R: Desired: %8f, Measured: %8f\n"), mot.RPS_desired, mot.RPS_measured);
			}
		}
		cnt++;*/

/*		if (motnum == 0) {
			printf_P(PSTR("%5f "), mot.RPS_measured);
		} else {
			printf_P(PSTR("%5f\n"), mot.RPS_measured);
		}*/

		PIDDebug piddebug;
		float error = mot.RPS_desired - mot.RPS_measured; 							// compute error
/*		if (motnum == 0) {
			printf_P(PSTR("Right: %f "), error);
		} else {
			printf_P(PSTR("Left: %f\n"), error);
		}*/
		float out = pid_update(mot.pid, pidgains, error, 1.0/TICK_HZ, &piddebug);	// update pid
		if (motnum == 0) {
//			printf_P(PSTR("PID out: %f\n"), out);
		}
//		out += mot.RPS_desired;		// instead of in line below, added it in here
		out += sign(mot.RPS_desired)*(mot.m*fabs(mot.RPS_desired) + mot.b); 		// compute feedforward
		// TODO: Got rid of feed forward term for now, unless needed in future!

		if (debug && motnum == 0) {
			pid_printDebug(out, error, piddebug);
		}

		if (out > 1) { 			// enforce saturation
			out = 1;
		} else if (out < -1) {
			out = -1;
		}

		motor_rampMotor((Motor)motnum, (int16_t)(out*motor_maxPWM)); 		// convert output to PWM, set it to the motor
		// ^^ was motor_setPWM();
	/*	if (motnum == 0) {
			printf_P(PSTR("Driving PWM: %f\n"), out*motor_maxPWM);
		}*/
	}
}
