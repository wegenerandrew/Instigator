#include "control/obstacleAvoidance.h"
#include "control/goto.h"
#include "control/drive.h"
#include "control/magfollow.h"
#include "hardware/sonar.h"
#include "util.h"

#include <stdint.h>
#include <math.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/pgmspace.h>

struct Obstacle {
	float left_dist, right_dist, front_dist;
	bool left, right, front;
	uint8_t left_filter, right_filter, front_filter;
};

static float x1, y1;
static float x2, y2;
static float x3, y3;
static float prev_x, prev_y, vel;
static int stage = 0;
static int obstacle_hits = 6;
static float x_dir, y_dir;
static float rotation;
static float x_offset, y_offset;
static float heading;

Obstacle obstacle;
static bool avoiding = false;
static bool enabled = false;
static int obstacle_threshold = 160;
static int obstacle_surround_x = 300;
static int obstacle_surround_y = 100;

void obstacleAvoidance_setEnabled(bool new_enabled) {
	if (enabled && avoiding && !new_enabled) {		// If we were in the middle of avoiding something, and we turn off avoidance
		stage = 0;
		goto_pos(prev_x, prev_y, vel);		// drop back to traveling to last desired position
		avoiding = false;
	}
	enabled = new_enabled;
}

bool obstacleAvoidance_getEnabled() {
	return enabled;
}

static void obstacleAvoidance_updateObstacles() {
	obstacle.left_dist = sonar_getDist(LEFT_SONAR);
	if (obstacle.left_dist < obstacle_threshold && obstacle.left_filter > obstacle_hits) {
		obstacle.left = true;
		obstacle.left_filter = obstacle_hits;
	} else if (obstacle.left_dist > obstacle_threshold && obstacle.left_filter < 0) {
		obstacle.left = false;
		obstacle.left_filter = 0;
	} else if (obstacle.left_dist < obstacle_threshold) {
		obstacle.left_filter++;
	} else {
		obstacle.left_filter--;
	}
	obstacle.front_dist = sonar_getDist(FRONT_SONAR);		// TODO FIX!!!
	if (obstacle.front_dist < obstacle_threshold && obstacle.front_filter > obstacle_hits) {
		obstacle.front = true;
		obstacle.front_filter = obstacle_hits;
	} else if (obstacle.front_dist > obstacle_threshold && obstacle.front_filter < 0) {
		obstacle.front = false;
		obstacle.front_filter = 0;
	} else if (obstacle.front_dist < obstacle_threshold) {
		obstacle.front_filter++;
	} else {
		obstacle.front_filter--;
	}
}

static float obstacleAvoidance_getSteer() {		// if we decide to start using a steering function
	return 1;
}

void obstacleAvoidance_tick() {
	if (!enabled) {
		return;
	}

	obstacleAvoidance_updateObstacles();

	if (!avoiding) {		// We weren't avoiding anything yet
		// Save where we were headed, set avoiding true, disable everything else, plan four points, go to each while inbetween waiting for them to finish
		if (/*!obstacle.left && !obstacle.left && */!obstacle.front) {
			return;
		}

		goto_stop();		// Disabling everything
		magfollow_stop();

		GotoData data = goto_getData();		// Save previous data (where we were headed)
		prev_x = data.x_desired;
		prev_y = data.y_desired;
		vel = data.vel;
		heading = data.heading;

		avoiding = true;					// Set avoiding true;

		x1 = 0;
		y1 = obstacle_surround_y;

		x2 = obstacle_surround_x;
		y2 = obstacle_surround_y;

		x3 = obstacle_surround_x;
		y3 = 0;

		x_dir = data.x_desired - data.x_original;
		y_dir = data.y_desired - data.y_original;
		rotation = anglewrap(atan2(y_dir, x_dir));
		x_offset = data.x_current;
		y_offset = data.y_current;
		x1 = ((float)(x1 * cos(rotation) - y1 * sin(rotation))) + x_offset;		// Transform x
		y1 = ((float)(x1 * sin(rotation) + y1 * cos(rotation))) + y_offset;		// Transform y

		x2 = ((float)(x2 * cos(rotation) - y2 * sin(rotation))) + x_offset;		// Transform x
		y2 = ((float)(x2 * sin(rotation) + y2 * cos(rotation))) + y_offset;		// Transform y

		x3 = ((float)(x3 * cos(rotation) - y3 * sin(rotation))) + x_offset;		// Transform x
		y3 = ((float)(x3 * sin(rotation) + y3 * cos(rotation))) + y_offset;		// Transform y

		goto_stop();
		magfollow_stop();
		printf_P(PSTR("Avoiding Obstacle!! %d, %d, %d\n"), obstacle.left, obstacle.front, obstacle.right);
		stage = 1;
		printf_P(PSTR("Going to X: %f, Y: %f\n"), x1, y1);
		goto_pos(x1, y1, vel);
	} else {				// Otherwise we already were avoiding
		// check if we're at point yet, if so, goto next, if not, wait
		// when done, stop avoiding, return to going to prev pos desired
		if (goto_getEnabled()) {
			return;
		}
		if (magfollow_getTurnEnabled()) {
			return;
		}

		if (stage == 1) {
			stage = 2;
			goto_stop();
			magfollow_stop();
			printf_P(PSTR("Going to X: %f, Y: %f\n"), x2, y2);
			goto_pos(x2, y2, vel);
		} else if (stage == 2) {
			stage = 3;
			goto_stop();
			magfollow_stop();
			printf_P(PSTR("Going to X: %f, Y: %f\n"), x3, y3);
			goto_pos(x3, y3, vel);
		} else if (stage == 3) {
			stage = 4;
			goto_stop();
			magfollow_stop();
			printf_P(PSTR("Going to heading: %f\n"), heading);
			magfollow_turn(vel, heading);
		} else if (stage == 4) {
			stage = 0;
			goto_stop();
			magfollow_stop();
			printf_P(PSTR("Going to previous X: %f, Y: %f\n"), prev_x, prev_y);
			goto_pos(prev_x, prev_y, vel);
			obstacle.left = false;
			obstacle.front = false;
			obstacle.right = false;
			avoiding = false;
		} else {
			stage = 0;
			avoiding = false;
		}
	}
}
