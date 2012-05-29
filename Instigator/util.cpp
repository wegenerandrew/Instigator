#include "util.h"

#include <stdio.h>
#include <util/delay.h>

void msleep(unsigned long ms) {
	while (ms >= 10) {
		_delay_ms(10);
		ms -= 10;
	}

	while (ms-- > 0)
		_delay_ms(1);
}

int vsscanf(const char *s, const char *fmt, va_list ap) {
	FILE f;
	f.flags = __SRD | __SSTR;
	f.buf = (char *)s;
	return vfscanf(&f, fmt, ap);
}

float anglewrap(float rad) {
	if (rad > M_PI) {
		return rad - 2*M_PI;
	} else if (rad < -M_PI) {
		return rad + 2*M_PI;
	} else {
		return rad;
	}
}

Vector rotate(Vector vec, float angle) {
	Vector ans;
	ans.x = (float)(vec.x * cos(angle) - vec.y * sin(angle));		// Transform x
	ans.y = (float)(vec.x * sin(angle) + vec.y * cos(angle));		// Transform y
	return ans;
}
