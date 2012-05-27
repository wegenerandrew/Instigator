#include "competition/spiralIn.h"
#include "control/goto.h"
#include "control/odometry.h"
#include "util.h"

#include <stdio.h>
#include <avr/pgmspace.h>

static const float incrementer = 0.5;
static const float edge_buffer = 0.5;

static void spiralIn_wait() {
	while (goto_getEnabled()) {
		// Do nothing
	}
}

void spiralIn_run(float xi, float yi, float xj, float yj, float xk, float yk, float xl, float yl, float vel) {// Meters!
	float diffx = 0;
	float diffy = 0;
	float diff = 0;
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
	goto_pos(mtocm(xi + edge_buffer), mtocm(yi + edge_buffer), vel);
	printf_P(PSTR("Going to 0, 0 with offset\n"));
	spiralIn_wait();
	for (float i = edge_buffer; i < (diff)/2; i = i + incrementer) {		// in meters!
		printf_P(PSTR("I: %f, x: %f, y: %f.\n"), i, xj - i, yj + i);
		goto_pos(mtocm(xj - i), mtocm(yj + i), vel);
		spiralIn_wait();
		printf_P(PSTR("I: %f, x: %f, y: %f.\n"), i, xk - i, yk - i);
		goto_pos(mtocm(xk - i), mtocm(yk - i), vel);
		spiralIn_wait();
		printf_P(PSTR("I: %f, x: %f, y: %f.\n"), i, xl + i, yl - i);
		goto_pos(mtocm(xl + i), mtocm(yl - i), vel);
		spiralIn_wait();
		printf_P(PSTR("I: %f, x: %f, y: %f.\n"), i, xi + i, yi + i + incrementer);
		goto_pos(mtocm(xi + i), mtocm(yi + i + incrementer), vel);
		spiralIn_wait();
	}
}
