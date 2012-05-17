#include "control/odometry.h"
#include "control/magfollow.h"
#include "control/drive.h"
#include "hardware/encoder.h"

#include <stdint.h>
#include <math.h>

static OdomData odom;

static int lprevious = 0;
static int rprevious = 0;

void odometry_setPos(float newx, float newy) {
	odom.x_pos = newx;
	odom.y_pos = newy;
}

void odometry_tick() {
	int ltick = encoder_get(LEFT_ENCODER);
	int rtick = encoder_get(RIGHT_ENCODER);
	float ldist = (encoder_diff(ltick, lprevious)/ticks_per_rotation)*wheel_circumference;
	float rdist = (encoder_diff(rtick, rprevious)/ticks_per_rotation)*wheel_circumference;
	lprevious = ltick;
	rprevious = rtick;

	odom.heading = magfollow_getHeading();
	float dist = (ldist + rdist)/2;
	odom.x_pos += dist*cos(odom.heading);
	odom.y_pos += dist*sin(odom.heading);
}
