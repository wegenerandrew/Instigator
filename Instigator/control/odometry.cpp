#include "control/odometry.h"
#include "control/magfollow.h"

#include <stdint.h>

static float x_pos = 0;
static float y_pos = 0;
static float heading = 0;

void odometry_setPos(float newx, float newy) {
	x_pos = newx;
	y_pos = newy;
}

void odometry_update(int16_t ldist, int16_t rdist) {
	heading = magfollow_getHeading();
	int dist = (ldist + rdist)/2;
//	int newx = dist*cos(heading);
//	int newy = dist*sin(heading);
	
}


