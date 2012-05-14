#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdint.h>

// make odom thing for data x and y and heading

void odometry_setPos(float newx, float newy);
void odometry_update(int16_t ldist, int16_t rdist);

#endif
