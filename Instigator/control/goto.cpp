#include "control/goto.h"
#include "control/drive.h"
#include "control/odometry.h"
#include "control/magfollow.h"
#include "util.h"

#include <stdint.h>
#include <math.h>

/*************************
 ** Allows non-blocking **
 ** navigation towards  **
 **  desired location   **
 *************************/

static bool enabled = false;
static int ctr = 100;
static const float pos_cutoff = 50;	// in centimeters, range before desired point to consider done
static bool turning = false;
static float desired_heading;

GotoData data;

static float goto_findHeading() {
	float x_diff = data.x_desired - data.x_current;
	float y_diff = data.y_desired - data.y_current;
	return anglewrap(atan2(y_diff, x_diff));
}

void goto_pos(float x_pos, float y_pos, float new_vel) {		// CENTIMETERS!!
	data.x_desired = x_pos;
	data.y_desired = y_pos;
	OdomData odom = odometry_getPos();
	data.x_current = odom.x_pos;
	data.y_current = odom.y_pos;
	data.x_original = odom.x_pos;
	data.y_original = odom.y_pos;
	if ((data.x_desired - data.x_original) > 0) {
		data.x_dir = 1;
	} else {
		data.x_dir = -1;
	}
	if ((data.y_desired - data.y_original) > 0) {
		data.y_dir = 1;
	} else {
		data.y_dir = -1;
	}
	data.vel = new_vel;
	desired_heading = goto_findHeading();		// Do primary aiming so we drive relatively straight
	if (desired_heading > .1) {		// If we need to turn first
		turning = true;
		magfollow_turn(data.vel, desired_heading);
	} else {
		turning = false;
	}
	goto_setEnabled(true);
}

void goto_setEnabled(bool new_enabled) {
	enabled = new_enabled;
}

void goto_stop() {
	goto_setEnabled(false);
	magfollow_stop();
}

bool goto_getEnabled() {
	return enabled;
}

GotoData goto_getData() {
	return data;
}

void goto_tick() {
	if (!enabled) {
		return;
	}

	if (turning) {		// Angle desired is such that we need to turn first
		if (magfollow_getTurnEnabled()) {	// for all following ticks we do nothing until magturn is done
			// Do nothing
			return;
		} else {	// magturn is done
			turning = false;
		}
	} else {		// since magturn is done, do magfollow

		// Update current data
		OdomData odom = odometry_getPos();
		data.x_current = odom.x_pos;
		data.y_current = odom.y_pos;
		data.heading = odom.heading;

		// Check if we're already there
		//if (abs(data.x_desired - data.x_current) < pos_cutoff && abs(data.y_desired - data.y_current) < pos_cutoff) {		// If we're within 20 cm on x and y of desired
		if (((data.x_dir*(data.x_desired - data.x_current) - pos_cutoff) < 0) && ((data.y_dir*(data.y_desired - data.y_current) - pos_cutoff) < 0)) {
			goto_stop();		// Then stop following
			return;
		}

	// If not there, continue going there
		// Drive at necessary heading with limiter for magfollow's PIDs
		if (ctr >= 100) {
			magfollow_start(data.vel, goto_findHeading());
			ctr = 0;
		} else {
			ctr++;
		}
	}
}
