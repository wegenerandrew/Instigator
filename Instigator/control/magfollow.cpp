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

static volatile bool follow_enabled = false;
static volatile bool turn_enabled = false;
static volatile float heading;						// heading in radians
static volatile float vel;
static volatile float error_filter;
static volatile bool debug;
static PIDState pidstate;
static PIDGains pidturngains = {150, 0, 45, 0};		// TODO: Integral??
static PIDGains pidgains = {100, 0, 120, 0};
static float heading_offset;						// heading offset value in radians
static MagCal magcal = {-24, -13.5, 0.94566};		// x_offset, y_offset, y_scale
static int ctr = 0;

void magfollow_start(float new_vel, float new_heading) {		// heading in radians
	pid_initState(pidstate);
	heading = anglewrap(new_heading);
	vel = new_vel;
	turn_enabled = false;
	follow_enabled = true;
}

void magfollow_stop() {
	follow_enabled = false;
	turn_enabled = false;
	drive_stop();
}

bool magfollow_enabled() {
	return follow_enabled;
}

bool magfollow_getTurnEnabled() {
	return turn_enabled;
}

void magfollow_turn(float new_vel, float new_heading) {		// heading in radians
	pid_initState(pidstate);
	heading = new_heading;
	vel = new_vel;
	follow_enabled = false;
	turn_enabled = true;
}

float magfollow_getHeading() {				// Returns heading in radians
	MagReading reading = mag_getReading();
	float x = reading.x - magcal.x_offset;
	float y = (reading.y - magcal.y_offset)*magcal.y_scale;
	float value = anglewrap(atan2(y, x) + M_PI/2 + heading_offset);
	return value;
}

float magfollow_getRawHeading() {
	MagReading reading = mag_getReading();
	float x = reading.x - magcal.x_offset;
	float y = (reading.y - magcal.y_offset)*magcal.y_scale;
	float value = anglewrap(atan2(y, x) + M_PI/2);
	return value;
}

float magfollow_getOffset() {
	return heading_offset;
}

void magfollow_setOffset(float offset) {
	heading_offset = offset;
}

void magfollow_setHeading(float desired_heading) {
	heading_offset += desired_heading - magfollow_getHeading();
	heading_offset = anglewrap(heading_offset);
}

void magfollow_tick() {
	if (!follow_enabled && ! turn_enabled) {
		return;
	}

	if (follow_enabled) {
		float error = magfollow_getHeading() - heading;
		error = anglewrap(error);

		error_filter = .85f*error_filter + .15f*error;

		PIDDebug piddebug;
		float out = pid_update(pidstate, pidgains, error_filter, TICK_DT, &piddebug);

		if (debug)
			pid_printDebug(out, error_filter, piddebug);

		drive_steer(10*out, vel);
	} else if (turn_enabled) {
		float error = magfollow_getHeading() - heading;
		error = anglewrap(error);
		PIDDebug piddebug;
		if (sign(error)*error < degtorad(1)) {	// stop when within this many degrees of desired heading
			ctr++;
		} else {
			ctr = 0;
		}
		float out = pid_update(pidstate, pidturngains, error, 0.01, &piddebug);
		if (debug) {
			pid_printDebug(out, error_filter, piddebug);
		}
		drive(vel*(out/200), -vel*(out/200));		// if it doesnt reach ends of turns, decrease divisor
		if (ctr > 20) {		// could need to be increased after being moved into tick
			ctr = 0;
			magfollow_stop();
		}
	}
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

PIDGains magfollow_getTurnGains() {
	return pidturngains;
}

void magfollow_setTurnGains(const PIDGains &newpidturngains) {
	pidturngains = newpidturngains;
}

