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

GotoData data;

static float goto_findHeading() {
	float x_diff = data.x_desired - data.x_current;
	float y_diff = data.y_desired - data.y_current;
	return anglewrap(atan2(y_diff, x_diff));
}

void goto_pos(float x_pos, float y_pos, float new_vel) {
	data.x_desired = x_pos;
	data.y_desired = y_pos;
	data.vel = new_vel;
	float desired_heading = goto_findHeading();		// Do primary aiming so we drive relatively straight
	if (desired_heading > .1) {
		magfollow_turn(data.vel, desired_heading);
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

	// Update current data
	OdomData odom = odometry_getPos();
	data.x_current = odom.x_pos;
	data.y_current = odom.y_pos;
	data.heading = odom.heading;

	// Check if we're already there
	if (abs(data.x_desired - data.x_current) < 10 && abs(data.y_desired - data.y_current) < 10) {		// If we're within 100 cm on x and y of desired
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
