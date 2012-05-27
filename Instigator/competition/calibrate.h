#ifndef CALIBRATE_H_
#define CALIBRATE_H_

struct Calibration {
	bool mag;
	bool odom;
	bool gps;
	bool field;
	float xi, yi;
	float xj, yj;
	float xk, yk;
	float xl, yl;
};

struct Field {
	float xi, yi;
	float xj, yj;
	float xk, yk;
	float xl, yl;
};

Field calibrate_competition();
Calibration calibrate_stationary();

bool calibrate_mag();
bool calibrate_gps();
bool calibrate_odom();
bool calibrate_field();

#endif
