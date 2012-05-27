#include "competition/overlap.h"
#include "control/goto.h"
#include "util.h"

#include <stdio.h>
#include <avr/pgmspace.h>

static const float incrementer = 0.5;

static void overlap_wait() {
	while (goto_getEnabled()) {
		// Do nothing
	}
}

void overlap_run(float x_min, float x_max, float y_min, float y_max, float vel) {
	float diff = 0;
	if ((x_max - x_min) < (y_max - y_min)) {
		diff = x_max - x_min;
	} else {
		diff = y_max - y_min;
	}
	for (float i = 0; i < (diff)/2; i = i + incrementer) {
		printf_P(PSTR("I: %f, xmax: %f, ymin: %f.\n"), i, x_max - i, y_min + i);
		goto_pos(mtocm(x_max - i), mtocm(y_min + i), vel);
		overlap_wait();
		printf_P(PSTR("I: %f, xmax: %f, ymax: %f.\n"), i, x_max - i, y_max - i);
		goto_pos(mtocm(x_max - i), mtocm(y_max - i), vel);
		overlap_wait();
		printf_P(PSTR("I: %f, xmin: %f, ymax: %f.\n"), i, x_min + i, y_max - i);
		goto_pos(mtocm(x_min + i), mtocm(y_max - i), vel);
		overlap_wait();
		printf_P(PSTR("I: %f, xmin: %f, ymin: %f.\n"), i, x_min + i, y_min + i + incrementer);
		goto_pos(mtocm(x_min + i), mtocm(y_min + i + incrementer), vel);
		overlap_wait();
	}
}
