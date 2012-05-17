#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdint.h>

struct OdomData {
	float x_pos;
	float y_pos;
	float heading;
};

void odometry_setPos(float newx, float newy);
void odometry_tick();

#endif
