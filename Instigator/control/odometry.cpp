#include "control/odometry.h"

static float x_pos = 0;
static float y_pos = 0;

void odometry_setPos(float newx, float newy) {
	x_pos = newx;
	y_pos = newy;
}

void odometry_tick() {
	
}
