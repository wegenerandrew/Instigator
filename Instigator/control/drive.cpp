#include "control/drive.h"
#include "control/motorcontrol.h"
#include "hardware/encoder.h"
#include "util.h"

static const float wheel_circumference	= 1;		// In centimeters. TODO: measure!
static const float wheelbase_radius	= 1;		// In centimeters. TODO: measure!

void drive(float lvel, float rvel) {		// Velocities in centimeters/second.
	motorcontrol_setRPS(MOTOR_LEFT, lvel/wheel_circumference);
	motorcontrol_setRPS(MOTOR_RIGHT, rvel/wheel_circumference);
	motorcontrol_setEnabled(true);
}

void drive_dist(float lvel, float rvel, float dist) {
	Motor mot;
	if (fabs(lvel) > fabs(rvel)) {
		mot = MOTOR_LEFT;
	} else {
		mot = MOTOR_RIGHT;
	}
	
	int16_t enc_ticks = (int16_t)((dist / wheel_circumference) * enc_per_rotation);
	uint16_t enc_start = enc_get(mot);

	drive(lvel, rvel);
	if (dist > 0) {
		while (enc_diff(enc_get(mot), enc_start) < enc_ticks) { }
	} else {
		while (enc_diff(enc_get(mot), enc_start) > enc_ticks) { }
	}
	drive_stop();
}

void drive_stop() {
	drive(0, 0);
}

void drive_off() {
	motorcontrol_setEnabled(false);
}

void drive_fd(float vel) {
	drive(vel, vel);
}

void drive_fdDist(float vel, float dist) {
	drive_dist(vel, vel, dist);
}

void drive_bk(float vel) {
	drive(-vel, -vel);
}

void drive_bkDist(float vel, float dist) {
	drive_dist(-vel, -vel, -dist);
}

void drive_turn(float vel, float deg, bool right) {
	if (right) {
		drive_rturnDeg(vel, deg);
	} else {
		drive_lturnDeg(vel, deg);
	}
}

void drive_lturn(float vel) {
	drive(-vel, vel);
}

void drive_lturnDeg(float vel, float deg) {
	float dist = degtorad(deg) * wheelbase_radius;
	drive_dist(-vel, vel, dist);
}

void drive_rturn(float vel) {
	drive(vel, -vel);
}

void drive_rturnDeg(float vel, float deg) {
	float dist = degtorad(deg) * wheelbase_radius;
	drive_dist(vel, -vel, dist);
}

void drive_steer(float steer, float vel) {
	drive(vel + steer, vel - steer);
}
