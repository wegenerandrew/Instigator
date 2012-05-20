#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdint.h>

struct OdomData {		// All in centimeters, heading in radians
	float x_pos;
	float y_pos;
	float heading;
};

void odometry_setPos(float newx, float newy);
OdomData odometry_getPos();

void odometry_tick();

#endif
