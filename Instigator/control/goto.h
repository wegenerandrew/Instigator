#ifndef GOTO_H_
#define GOTO_H_

#include <stdint.h>

struct GotoData {
	float x_desired;
	float y_desired;
	float x_current;
	float y_current;
	float x_original;
	float y_original;
	int x_dir;
	int y_dir;
	float heading;
	float vel;
};

void goto_pos(float x_pos, float y_pos, float new_vel);

void goto_setEnabled(bool new_enabled);
void goto_stop();
bool goto_getEnabled();
GotoData goto_getData();

void goto_tick();

#endif
