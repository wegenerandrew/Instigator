#include "competition/spiralIn.h"
#include "control/goto.h"
#include "control/odometry.h"
#include "util.h"

#include <stdio.h>
#include <avr/pgmspace.h>

static const float incrementer = 0.5;
static const float edge_buffer = 1;

static float new_xi, new_yi;
static float new_xj, new_yj;
static float new_xk, new_yk;
static float new_xl, new_yl;

static void spiralIn_wait() {
	while (goto_getEnabled()) {
		// Do nothing
	}
}

void spiralIn_run(float xi, float yi, float xj, float yj, float xk, float yk, float xl, float yl, float vel) {// Meters!
	float diffx = 0;
	float diffy = 0;
	float diff = 0;
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
	new_xi = xi + vec.x;
	new_yi = yi + vec.y;
	goto_pos(mtocm(new_xi), mtocm(new_yi), vel);
	printf_P(PSTR("Going to 0, 0 with offset\n"));
	spiralIn_wait();
	for (float i = edge_buffer; i < (diff)/2; i = i + incrementer) {		// in meters!
		vec.x = -i;
		vec.y = i;
		vec = rotate(vec, heading);
		new_xj = xj + vec.x;
		new_yj = yj + vec.y;
		printf_P(PSTR("I: %f, x: %f, y: %f (meters).\n"), i, new_xj, new_yj);
		goto_pos(mtocm(new_xj), mtocm(new_yj), vel);
		spiralIn_wait();
		vec.x = -i;
		vec.y = -i;
		vec = rotate(vec, heading);
		new_xk = xk + vec.x;
		new_yk = yk + vec.y;
		printf_P(PSTR("I: %f, x: %f, y: %f (meters).\n"), i, new_xk, new_yk);
		goto_pos(mtocm(new_xk), mtocm(new_yk), vel);
		spiralIn_wait();
		vec.x = i;
		vec.y = -i;
		vec = rotate(vec, heading);
		new_xl = xl + vec.x;
		new_yl = yl + vec.y;
		printf_P(PSTR("I: %f, x: %f, y: %f (meters).\n"), i, new_xl, new_yl);
		goto_pos(mtocm(new_xl), mtocm(new_yl), vel);
		spiralIn_wait();
		vec.x = i + incrementer;
		vec.y = i;
		vec = rotate(vec, heading);
		new_xi = xi + vec.x;
		new_yi = yi + vec.y;
		printf_P(PSTR("I: %f, x: %f, y: %f (meters).\n"), i, new_xi, new_yi);
		goto_pos(mtocm(new_xi), mtocm(new_yi), vel);
		spiralIn_wait();
	}
}
