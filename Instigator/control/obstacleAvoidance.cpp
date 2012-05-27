#include "control/obstacleAvoidance.h"
#include "control/goto.h"
#include "control/drive.h"
#include "control/magfollow.h"
#include "hardware/sonar.h"

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
static int obstacle_hits = 1;

Obstacle obstacle;
static bool avoiding = false;
static bool enabled = false;
static int obstacle_threshold = 460;
static int obstacle_surround_square = 300;

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
/*	obstacle.left_dist = sonar_getDist(LEFT_SONAR);
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
	}*/
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
/*	obstacle.right_dist = sonar_getDist(RIGHT_SONAR);
	if (obstacle.right_dist < obstacle_threshold && obstacle.right_filter > obstacle_hits) {
		obstacle.right = true;
		obstacle.right_filter = obstacle_hits;
	} else if (obstacle.right_dist > obstacle_threshold && obstacle.right_filter < 0) {
		obstacle.right = false;
		obstacle.right_filter = 0;
	} else if (obstacle.right_dist < obstacle_threshold) {
		obstacle.right_filter++;
	} else {
		obstacle.right_filter--;
	}*/
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

		avoiding = true;					// Set avoiding true;

		if (fabs(data.x_desired - data.x_current) < fabs(data.y_desired - data.y_current)) {		// Then we're driving on the y axis
			if ((data.y_desired - data.y_current) > 0) {	// Going + on y
				x1 = data.x_current - obstacle_surround_square;
				y1 = data.y_current;

				x2 = data.x_current - obstacle_surround_square;
				y2 = data.y_current + obstacle_surround_square;

				x3 = data.x_current;
				y3 = data.y_current + obstacle_surround_square;
			} else {		// Going - on y
				x1 = data.x_current + obstacle_surround_square;
				y1 = data.y_current;

				x2 = data.x_current + obstacle_surround_square;
				y2 = data.y_current - obstacle_surround_square;

				x3 = data.x_current;
				y3 = data.y_current - obstacle_surround_square;
			}
		} else {		// Otherwise we're on the x axis
			if ((data.x_desired - data.x_current) > 0) {	// Going + on x
				x1 = data.x_current;
				y1 = data.y_current + obstacle_surround_square;

				x2 = data.x_current + obstacle_surround_square;
				y2 = data.y_current + obstacle_surround_square;

				x3 = data.x_current + obstacle_surround_square;
				y3 = data.y_current;
			} else {		// Going - on x
				x1 = data.x_current;
				y1 = data.y_current - obstacle_surround_square;

				x2 = data.x_current - obstacle_surround_square;
				y2 = data.y_current - obstacle_surround_square;

				x3 = data.x_current - obstacle_surround_square;
				y3 = data.y_current;
			}
		}
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

		if (stage == 1) {
			stage = 2;
			goto_stop();
			magfollow_stop();
			_delay_ms(2000);
			printf_P(PSTR("Going to X: %f, Y: %f\n"), x2, y2);
			goto_pos(x2, y2, vel);
		} else if (stage == 2) {
			stage = 3;
			goto_stop();
			magfollow_stop();
			_delay_ms(2000);
			printf_P(PSTR("Going to X: %f, Y: %f\n"), x3, y3);
			goto_pos(x3, y3, vel);
		} else if (stage == 3) {
			stage = 0;
			goto_stop();
			magfollow_stop();
			_delay_ms(2000);
			printf_P(PSTR("Going to X: %f, Y: %f\n"), prev_x, prev_y);
			goto_pos(prev_x, prev_y, vel);
			avoiding = false;
		} else {
			stage = 0;
			avoiding = false;
		}
	}
}
