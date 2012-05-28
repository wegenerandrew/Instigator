#include "competition/calibrate.h"
#include "control/magfollow.h"
#include "control/odometry.h"
#include "control/drive.h"
#include "debug/controlpanel.h"
#include "hardware/gps.h"
#include "util.h"

#include <stdio.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

struct Coord {
	float x;
	float y;
};

static float x1, y1;
static float x2, y2;
static float x3, y3;
static float x4, y4;
static float x5, y5;
static float x6, y6;
static float x7, y7;
static float x8, y8;

static float ma, mb, mc, md;

Calibration calibration;
Field field;

Field calibrate_competition() {
	gps_setOffset(0, 0);		// clear out all possible previously done calibrations so we only work in global
	magfollow_setOffset(0);
	calibrate_field();
	return field;
}

Calibration calibrate_stationary() {
	calibrate_mag();
	calibrate_gps();
	calibrate_odom();
	return calibration;
}

bool calibrate_mag() {
	printf_P(PSTR("Magfollow Heading Calibration: "));
	magfollow_setHeading(0);		// heading set in radians
	calibration.mag = true;
	printf_P(PSTR("Success!\n"));
	return calibration.mag;
}

bool calibrate_gps() {
	printf_P(PSTR("GPS Offset Calibration: "));
	GPSHealth gps = gps_getHealth();
	if (gps.Lat_Std_Dev < gps_std_dev_cutoff && gps.Lon_Std_Dev < gps_std_dev_cutoff && gps.solStatus == 0 && gps.rt20Status == 0 && gps.fixStatus == 2) {
		GPSPos pos = gps_getPos();
		float heading = anglewrap(magfollow_getRawHeading());
		gps_setOffset(pos.X_Raw - gps_base_offset*cos(heading), pos.Y_Raw - gps_base_offset*sin(heading));
		gps_setEnabled(true);
		calibration.gps = true;
		printf_P(PSTR("Success!\n"));
	} else {
		gps_setEnabled(false);
		calibration.gps = false;
		printf_P(PSTR("Failed! Invalid position.\n"));
	}
	return calibration.gps;
}

bool calibrate_odom() {
	printf_P(PSTR("Odometry Zeroing Calibration: "));
	odometry_setPos(0, 0);
	calibration.odom = true;
	printf_P(PSTR("Success!\n"));
	return calibration.odom;
}

static Coord calibrate_avgGPS() {
	GPSPos gps;
	Coord coord;
	gps = gps_getPos();
	coord.x = gps.X_Raw;
	coord.y = gps.Y_Raw;
	printf_P(PSTR("X: %f, Y: %f\n"), gps.X_Raw, gps.Y_Raw);
	return coord;
}

static void wait_key() {
	float speed = 0;
	float steer_incrementer = 5;
	float speed_incrementer = 10;
	int steer = 0;
	while (true) {
		int key = controlpanel_promptChar("Press any key to average and accept position or w a s d 'space' to drive.");
		switch (key) {
			case ' ':
				drive_stop();
				speed = 0;
				steer = 0;
				break;
			case 'w':
				steer = 0;
				speed += speed_incrementer;
				printf_P(PSTR("Speed: %f\n"), speed);
				drive_fd(speed);
				break;
			case 'a':
				steer -= steer_incrementer;
				drive_steer(steer, speed);
				break;
			case 's':
				steer = 0;
				speed -= speed_incrementer;
				printf_P(PSTR("Speed: %f\n"), speed);
				drive_fd(speed);		// speed will be negative!!
				break;
			case 'd':
				steer += steer_incrementer;
				drive_steer(steer, speed);
				break;
			default:
				return;
		}
	}
}

bool calibrate_field() {
	printf_P(PSTR("Field Calibration:\n"));
	Coord coord;
	printf_P(PSTR("Move robot GPS antenna in-line with and below left edge of field.\n"));
	wait_key();
	coord = calibrate_avgGPS();
	x1 = coord.x;
	y1 = coord.y;
	printf_P(PSTR("Move robot GPS antenna in-line with and below right edge of field.\n"));
	wait_key();
	coord = calibrate_avgGPS();
	x2 = coord.x;
	y2 = coord.y;
	printf_P(PSTR("Move robot GPS antenna in-line with and to right of bottom edge of field.\n"));
	wait_key();
	coord = calibrate_avgGPS();
	x3 = coord.x;
	y3 = coord.y;
	printf_P(PSTR("Move robot GPS antenna in-line with and to right of top edge of field.\n"));
	wait_key();
	coord = calibrate_avgGPS();
	x4 = coord.x;
	y4 = coord.y;
	printf_P(PSTR("Move robot GPS antenna in-line with and above right edge of field.\n"));
	wait_key();
	coord = calibrate_avgGPS();
	x5 = coord.x;
	y5 = coord.y;
	printf_P(PSTR("Move robot GPS antenna in-line with and above left edge of field.\n"));
	wait_key();
	coord = calibrate_avgGPS();
	x6 = coord.x;
	y6 = coord.y;
	printf_P(PSTR("Move robot GPS antenna in-line with and to left of top edge of field.\n"));
	wait_key();
	coord = calibrate_avgGPS();
	x7 = coord.x;
	y7 = coord.y;
	printf_P(PSTR("Move robot GPS antenna in-line with and to left of bottom edge of field.\n"));
	wait_key();
	coord = calibrate_avgGPS();
	x8 = coord.x;
	y8 = coord.y;
	// Have all points on lines, now find slopes of lines a, b, c, and d
	ma = (y3 - y8)/(x3 - x8);
	mb = (y5 - y2)/(x5 - x2);
	mc = (y4 - y7)/(x4 - x7);
	md = (y6 - y1)/(x6 - x1);
	// Have all slopes, now plug in to find coordinates of corners i, j, k, and l
	field.xi = (y8 - y1 + (x1*md) - (x8*ma))/(md - ma);
	field.yi = (md*(field.xi - x1) + y1);
	printf_P(PSTR("Xi: %f, Yi: %f\n"), field.xi, field.yi);
	field.xj = (y2 - y3 + (x3*ma) - (x2*mb))/(ma - mb);
	field.yj = (ma*(field.xj - x3) + y3);
	printf_P(PSTR("Xj: %f, Yj: %f\n"), field.xj, field.yj);
	field.xk = (y4 - y5 + (x5*mb) - (x4*mc))/(mb - mc);
	field.yk = (mb*(field.xk - x5) + y5);
	printf_P(PSTR("Xk: %f, Yk: %f\n"), field.xk, field.yk);
	field.xl = (y6 - y7 + (x7*mc) - (x6*md))/(mc - md);
	field.yl = (mc*(field.xl - x7) + y7);
	printf_P(PSTR("Xl: %f, Yl: %f\n"), field.xl, field.yl);
	gps_setEnabled(true);
	calibration.field = true;
	
	return calibration.field;
}
