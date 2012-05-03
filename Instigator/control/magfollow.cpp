#include "control/magfollow.h"
#include "control/pid.h"
#include "control/drive.h"
#include "control/motorcontrol.h"
#include "control/tick.h"
#include "debug/debug.h"
#include "hardware/mag.h"
#include "util.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

static volatile bool enabled = false;
static volatile float heading;						// heading in radians
static volatile float vel;
static volatile float error_filter;
static volatile bool debug;
static PIDState pidstate;
static PIDGains pidturngains = {100, 0, 5, 0};		// TODO: Integral??
static PIDGains pidgains = {100, 0, 5, 0};
static float heading_offset;						// heading offset value in radians
static MagCal magcal = {-97.5, -81, 0.89606};		// x_offset, y_offset, y_scale

void magfollow_start(float new_vel, float new_heading) {		// heading in radians
	pid_initState(pidstate);
	heading = new_heading;
	vel = new_vel;
	enabled = true;
}

void magfollow_stop() {
	enabled = false;
	drive_stop();
}

bool magfollow_enabled() {
	return enabled;
}

void magfollow_turn(float new_vel, float new_heading) {		// heading in radians
	pid_initState(pidstate);
	heading = new_heading;
	vel = new_vel;
	float error = magfollow_getHeading() - heading;
	error = anglewrap(error);
	PIDDebug piddebug;
	while (sign(error)*error > 0.15) {	// stop when within this many radians of desired heading
		_delay_ms(10);
		error = magfollow_getHeading() - heading;
		error = anglewrap(error);
		float out = pid_update(pidstate, pidturngains, error, 0.01, &piddebug);
		if (debug) {
			pid_printDebug(out, error_filter, piddebug);
		}
//		printf("error: %f, pid: %f\n", sign(error)*error, out);	//debugging
		drive(vel*(out/200), -vel*(out/200));		// if it doesnt reach ends of turns, decrease divisor
//		printf("lvel: %f, rvel: %f, curr: %f, des: %f\n", vel*(out/300), -vel*(out/300), magfollow_getHeading(), heading);
	}
}

float magfollow_getHeading() {
//	cli();
	MagReading reading = mag_getReading();
	float x = reading.x - magcal.x_offset;
	float y = (reading.y - magcal.y_offset)*magcal.y_scale;
//	float value = anglewrap(atan(y/x) + heading_offset);
	float value = anglewrap(atan2(y, x) + heading_offset);
//	sei();
	return value;
}

void magfollow_setHeading(float desired_heading) {
	heading_offset += desired_heading - magfollow_getHeading();
	heading_offset = anglewrap(heading_offset);
}

void magfollow_tick() {
	if (!enabled) {
		return;
	}
	
	float error = magfollow_getHeading() - heading;
	error = anglewrap(error);
	
	error_filter = .85f*error_filter + .15f*error;

	PIDDebug piddebug;
	float out = pid_update(pidstate, pidgains, error_filter, TICK_DT, &piddebug);

	if (debug)
		pid_printDebug(out, error_filter, piddebug);
	
	drive_steer(10*out, vel);

}

void magfollow_setDebug(bool new_debug) {
	debug = new_debug;
}

void magfollow_setCal(const MagCal &new_magcal) {
	magcal = new_magcal;
}

const MagCal &magfollow_getCal() {
	return magcal;
}

PIDGains magfollow_getGains() {
	return pidgains;
}

void magfollow_setGains(const PIDGains &newpidgains) {
	pidgains = newpidgains;
}

