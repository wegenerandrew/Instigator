#include "competition/overlap.h"
#include "control/goto.h"
#include "hardware/sonar.h"
#include "util.h"

#include <stdio.h>
#include <avr/pgmspace.h>

static const float incrementer = 0.5;		// what base amount should we move over by for next path? (meters)
static const float edge_buffer = 1;

static float aInc = 1;		// Multipliers for portion of incrementer to use on next pass
static float bInc = 1;
static float cInc = 1;
static float dInc = 1;
static float new_xi, new_yi;
static float new_xj, new_yj;
static float new_xk, new_yk;
static float new_xl, new_yl;
static float dist = 0;

static float overlap_wait() {		// wait for goto to finish, while finding closest sonar reading of trashcan
	float min = 0;
	float dist = 0;
	while (goto_getEnabled()) {
		dist = sonar_getDist(LEFT_SONAR);
		if (dist < min) {
			min = dist;
		}
	}
	return min;
}

void overlap_run(float xi, float yi, float xj, float yj, float xk, float yk, float xl, float yl, float vel) {
	float diffx = 0;
	float diffy = 0;
	float diff = 0;
	new_xi = xi;
	new_yi = yi;
	new_xj = xj;
	new_yj = yj;
	new_xk = xk;
	new_yk = yk;
	new_xl = xl;
	new_yl = yl;
	float heading = anglewrap(atan2((yj - yi), (xj - xi)));		// Get angle of baseline so we can rotate all our shifts by that then add them in.
	if (fabs(xj - xi) > fabs(xk - xl)) {
		diffx = fabs(xj - xi);
	} else {
		diffx = fabs(xk - xl);
	}
	if (fabs(yl - yi) > fabs(yk - yj)) {
		diffy = fabs(yl - yi);
	} else {
		diffy = fabs(yk - yj);
	}
	if (diffx < diffy) {
		diff = diffx;
	} else {
		diff = diffy;
	}
	Vector vec;
	vec.x = edge_buffer;
	vec.y = edge_buffer;
	vec = rotate(vec, heading);
	new_xi = new_xi + vec.x;
	new_yi = new_yi + vec.y;
	goto_pos(mtocm(new_xi), mtocm(new_yi), vel);
	printf_P(PSTR("Going to 0, 0 with offset\n"));
	overlap_wait();
	while (true) {		// in meters!
		vec.x = -bInc;		// Side A, plotting j
		vec.y = aInc;
		vec = rotate(vec, heading);
		new_xj = new_xj + vec.x;
		new_yj = new_yj + vec.y;
		printf_P(PSTR("aInc: %f, x: %f, y: %f (meters). last dist: %f\n"), aInc, new_xj, new_yj, dist);
		goto_pos(mtocm(new_xj), mtocm(new_yj), vel);
		dist = overlap_wait();
		aInc = (dist/sonar_max)*incrementer;

		vec.x = -bInc;		// Side B, plotting k
		vec.y = -cInc;
		vec = rotate(vec, heading);
		new_xk = new_xk + vec.x;
		new_yk = new_yk + vec.y;
		printf_P(PSTR("bInc: %f, x: %f, y: %f (meters). last dist: %f\n"), bInc, new_xk, new_yk, dist);
		goto_pos(mtocm(new_xk), mtocm(new_yk), vel);
		dist = overlap_wait();
		bInc = (dist/sonar_max)*incrementer;

		vec.x = dInc;		// Side C, plotting l
		vec.y = -cInc;
		vec = rotate(vec, heading);
		new_xl = new_xl + vec.x;
		new_yl = new_yl + vec.y;
		printf_P(PSTR("cInc: %f, x: %f, y: %f (meters). last dist: %f\n"), cInc, new_xl, new_yl, dist);
		goto_pos(mtocm(new_xl), mtocm(new_yl), vel);
		dist = overlap_wait();
		cInc = (dist/sonar_max)*incrementer;

		vec.x = dInc;	// Side D, plotting i
		vec.y = aInc;
		vec = rotate(vec, heading);
		new_xi = new_xi + vec.x;
		new_yi = new_yi + vec.y;
		printf_P(PSTR("dInc: %f, x: %f, y: %f (meters). last dist: %f\n"), dInc, new_xi, new_yi, dist);
		goto_pos(mtocm(new_xi), mtocm(new_yi), vel);
		dist = overlap_wait();
		dInc = (dist/sonar_max)*incrementer;
	}
}
