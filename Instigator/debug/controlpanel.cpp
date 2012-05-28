#include "competition/spiralIn.h"
#include "competition/calibrate.h"
#include "control/drive.h"
#include "control/magfollow.h"
#include "control/motorcontrol.h"
#include "control/tick.h"
#include "control/odometry.h"
#include "control/goto.h"
#include "control/obstacleAvoidance.h"
#include "hardware/gps.h"
#include "debug/debug.h"
#include "debug/tests.h"
#include "debug/controlpanel.h"
#include "hardware/motor.h"
#include "hardware/mag.h"
#include "hardware/encoder.h"
#include "hardware/sonar.h"
#include "hardware/adc.h"
#include "hardware/weedwhacker.h"
#include "util.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static const char unknown_str[] PROGMEM = "Unknown. ? for help.";
static const float turn_diff = 100;

static float speed = 0;
static float setSpeed = 100;
static float speed_incrementer = 10;
static float large_speed_incrementer = 100;
static float steer_incrementer = 5;
static float turn_reducer = 50;

void controlpanel_init() {
	printf_P(PSTR("Starting up\n"));
}

bool controlpanel() {
	while (true) {
		switch (controlpanel_promptChar("Main")) {
			case 'd':
				controlpanel_drive();
				break;
			case 's':
				controlpanel_sensor();
				break;
			case 'l':
				controlpanel_debug();
				break;
			case 'a':
				controlpanel_autonomy();
				break;
			case 'o':
				controlpanel_odometry();
				break;
			case 'c':
				controlpanel_calibrate();
				break;
			default:
				puts_P(unknown_str);
				break;
			case '?':
				static const char msg[] PROGMEM =
					"Control Panels:\n"
					"  d - Drive\n"
					"  s - Sensors\n"
					"  l - debug\n"
					"  a - Autonomy\n"
					"  o - Odometry\n"
					"  c - Calibrate\n"
					"  q - Quit";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_autonomy() {
	float follow = 0;
	char input = ' ';
	OdomData odom;
	while (true) {
		char ch = controlpanel_promptChar("Autonomy");
		switch (ch) {
			case 'G': {
				float x_des, y_des, vel;
				odom = odometry_getPos();
				printf_P(PSTR("Current Position, X (meters): %f, Y (meters): %f, Heading (deg): %f\n"), cmtom(odom.x_pos), cmtom(odom.y_pos), radtodeg(odom.heading));
				controlpanel_prompt("Goto X (meters): ", "%f", &x_des);
				controlpanel_prompt("Goto Y (meters): ", "%f", &y_des);
				controlpanel_prompt("At, Vel (cm/s): ", "%f", &vel);
				obstacleAvoidance_setEnabled(true);
				goto_pos(mtocm(x_des), mtocm(y_des), vel);
				break;
			}
			case 'g': {
				float x_des, y_des, vel;
				odom = odometry_getPos();
				printf_P(PSTR("Current Position, X (meters): %f, Y (meters): %f, Heading (deg): %f\n"), cmtom(odom.x_pos), cmtom(odom.y_pos), radtodeg(odom.heading));
				controlpanel_prompt("Goto X (meters): ", "%f", &x_des);
				controlpanel_prompt("Goto Y (meters): ", "%f", &y_des);
				controlpanel_prompt("At, Vel (cm/s): ", "%f", &vel);
				goto_pos(mtocm(x_des), mtocm(y_des), vel);
				break;
			}
			case 'e':
				if (goto_getEnabled()) {
					printf_P(PSTR("Goto: enabled, "));
				} else {
					printf_P(PSTR("Goto: disabled, "));
				}
				if (magfollow_enabled()) {
					printf_P(PSTR("Magfollow: enabled, "));
				} else {
					printf_P(PSTR("Magfollow: disabled, "));
				}
				if (obstacleAvoidance_getEnabled()) {
					printf_P(PSTR("Obstacle Avoidance: enabled.\n"));
				} else {
					printf_P(PSTR("Obstacle Avoidance: disabled.\n"));
				}
				break;
			case 's': {			// Sets current heading of robot to prompted heading from user
				float newheading;
				controlpanel_prompt("Heading (deg)", "%f", &newheading);
				magfollow_setHeading(degtorad(newheading));
				break;
			}
			case 'h': {			// Prints out magnetometer calibrated heading
				float heading = magfollow_getHeading();
				heading = radtodeg(heading);
				printf_P(PSTR("Mag Heading (deg): %f\n"), heading);
				break;
			}
			case 'w': {
				printf_P(PSTR("Currently Facing (deg): %f\n"), radtodeg(magfollow_getHeading()));
				controlpanel_prompt("Follow at Heading (deg)", "%f", &follow);
				magfollow_start(setSpeed, anglewrap(degtorad(follow)));
				printf_P(PSTR("Following at (deg): %f\n"), follow);
				break;
			}
			case 'a':
				follow = follow + 5;
				if (magfollow_enabled()) {
					magfollow_start(setSpeed, anglewrap(degtorad(follow)));
					printf_P(PSTR("Following at (deg): %f\n"), follow);
				} else {
					printf_P(PSTR("Not following, but heading set to (deg): %f\n"), follow);
				}
				break;
			case 'd':
				follow = follow - 5;
				if (magfollow_enabled()) {
					magfollow_start(setSpeed, anglewrap(degtorad(follow)));
					printf_P(PSTR("Following at (deg): %f\n"), follow);
				} else {
					printf_P(PSTR("Not following, but heading set to (deg): %f\n"), follow);
				}
				break;
			case 't':
				printf_P(PSTR("Currently Facing (deg): %f\n"), radtodeg(magfollow_getHeading()));
				controlpanel_prompt("Turn to Heading (deg)", "%f", &follow);
				follow = degtorad(follow);
				printf_P(PSTR("Currently at (deg): %f, Turning to (deg): %f\n"), radtodeg(magfollow_getHeading()), radtodeg(follow));
				magfollow_turn(setSpeed, anglewrap(follow));
				break;
			case 'o':
				obstacleAvoidance_setEnabled(false);
				printf_P(PSTR("Obstacle Avoidance Disabled.\n"));
				break;
			case 'O':
				obstacleAvoidance_setEnabled(true);
				printf_P(PSTR("Obstacle Avoidance Enabled!\n"));
				break;
			case 'c': {
				printf_P(PSTR("Beginning auto-cal!\nTurn robot to face 0 Degrees in field.\n"));
				input = controlpanel_promptChar("Press 'Enter' to begin or any other key to cancel.");
				if (input == 10) {
					printf_P(PSTR("Calibrating...\n"));
					calibrate_stationary();
				} else {
					printf_P(PSTR("Auto-cal Cancelled.\n"));
				}
				break;
			}
			case 'C': {
				printf_P(PSTR("Beginning Competition Routine!\n"));
				input = controlpanel_promptChar("Press 'Enter' to begin or any other key to cancel.");
				if (input == 10) {
					printf_P(PSTR("Calibrating...\n"));
					Field field = calibrate_competition();
					float vel;
					controlpanel_prompt("Velocity (cm/s): ", "%f", &vel);
					printf_P(PSTR("Running Spiral-In Course!\n"));
					spiralIn_run(field.xi, field.yi, field.xj, field.yj, field.xk, field.yk, field.xl, field.yl, vel);
				} else {
					printf_P(PSTR("Auto-cal Cancelled.\n"));
				}
				break;
			}
			case 'i': {
				printf_P(PSTR("Beginning competition auto-cal!\nTurn robot to face 0 Degrees in field.\n"));
				input = controlpanel_promptChar("Press 'Enter' to begin or any other key to cancel.");
				if (input == 10) {
					printf_P(PSTR("Calibrating...\n"));
					magfollow_setOffset(0);
				} else {
					printf_P(PSTR("Auto-cal Cancelled.\n"));
				}
				float xi, yi;
				float xj, yj;
				float xk, yk;
				float xl, yl;
				float vel;
				controlpanel_prompt("Xi (meters): ", "%f", &xi);
				controlpanel_prompt("Yi (meters): ", "%f", &yi);
				controlpanel_prompt("Xj (meters): ", "%f", &xj);
				controlpanel_prompt("Yj (meters): ", "%f", &yj);
				controlpanel_prompt("Xk (meters): ", "%f", &xk);
				controlpanel_prompt("Yk (meters): ", "%f", &yk);
				controlpanel_prompt("Xl (meters): ", "%f", &xl);
				controlpanel_prompt("Yl (meters): ", "%f", &yl);
				controlpanel_prompt("Velocity (cm/s): ", "%f", &vel);
				spiralIn_run(xi, yi, xj, yj, xk, yk, xl, yl, vel);
				break;
			}
			case 'f':
				debug_halt("testing");
				break;
			case ' ':
				magfollow_stop();
				obstacleAvoidance_setEnabled(false);
				goto_setEnabled(false);
				break;
			case 'q':
				magfollow_stop();
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Control Panels:\n"
					"  G  - Goto Coordinate w/ Obstacle Avoidance\n"
					"  g  - Goto Coordinate\n"
					"  e  - Print states of enables\n"
					"  s  - Set Heading\n"
					"  h  - Current Heading\n"
					"  w  - Magfollow\n"
					"  a  - Shift following left\n"
					"  d  - Shift following right\n"
					"  t  - MagTurn\n"
					" O/o - Enable/Disable Obstacle Avoidance\n"
					"  c  - Auto-Calibration Routine\n"
					"  C  - Do Competition Routine\n"
					"  i  - Run Spiral-In competition\n"
					"  f  - Halt\n"
					" ' ' - Stop\n"
					"  q  - Quit";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_odometry() {
	OdomData odom;
	while(true) {
		char ch = controlpanel_promptChar("Odometry");
		switch (ch) {
			case 'p':
				odom = odometry_getPos();
				printf_P(PSTR("X: %f, Y: %f, Heading (deg): %f\n"), odom.x_pos, odom.y_pos, radtodeg(odom.heading));
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Control Panels:\n"
					"  p - Current Position Data\n"
					" ' '- Stop\n"
					"  q - Quit";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_drive() {
	int16_t steer = 0;
	while (true) {
		char ch=controlpanel_promptChar("Drive");
		switch (ch) {
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
			case 'W':
				steer = 0;
				speed += large_speed_incrementer;
				printf_P(PSTR("Speed: %f\n"), speed);
				drive_fd(speed);
				break;
			case 'A':
				drive_lturn(setSpeed - turn_reducer);
				break;
			case 'S':
				steer = 0;
				speed -= large_speed_incrementer;
				printf_P(PSTR("Speed: %f\n"), speed);
				drive_fd(speed);		// speed will be negative!!
				break;
			case 'D':
				drive_rturn(setSpeed - turn_reducer);
				break;
			case 'k':
				weedwhacker_power(false);
				break;
			case 'K':
				weedwhacker_power(true);
				break;
			case '=':
				setSpeed += 100;
				printf_P(PSTR("Speed: %f\n"), setSpeed);
				break;
			case '-':
				setSpeed -= 100;
				printf_P(PSTR("Speed: %f\n"), setSpeed);
				break;
			case '+':
				setSpeed += 10;
				printf_P(PSTR("Speed: %f\n"), setSpeed);
				break;
			case '_':
				setSpeed -= 10;
				printf_P(PSTR("Speed: %f\n"), setSpeed);
				break;

			case 'p':
				motorcontrol_setDebug(false);
				printf_P(PSTR("Debug disabled\n"));
				break;

			case 'c':
				motorcontrol_setEnabled(false);
				printf_P(PSTR("Motor control disabled\n"));
				break;

			case 'P':
				motorcontrol_setDebug(true);
				break;

			case 'q':	
			//	motorcontrol_setEnabled(false);
				return;

			case 'm': {
				/*float amax = drive_getTrajAmax();
				printf_P(PSTR("Current amax: %.2f\n"), amax);
				if (controlpanel_prompt("amax", "%f", &amax) != 1) {
					printf_P(PSTR("Cancelled.\n"));
					continue;
				}

				drive_setTrajAmax(amax);
				printf_P(PSTR("Amax set to: %.2f\n"), amax);*/
				break;
			}

			default:
				puts_P(unknown_str);
				drive_stop();
				break;
			case '?':
				static const char msg[] PROGMEM =
					"Drive commands:\n"
					"  wasd  - Control robot\n"
					"  space - Stop\n"
					"  k	 - Weedwhacker Kill\n"
					"  K	 - Weedwhacker Start\n"
					"  -=_+  - Adjust speed\n"
					"  WASD  - Full speed movements\n"
					"  c	 - Disable motor control\n"
					"  Pp	 - Enable/Disable motor control debug\n"
					"  q	 - Back";
				puts_P(msg);
				break;
		}
	}
}

void controlpanel_debug() {
	bool error_led = false;
	bool estop_led = false;
	bool tick_led = false;
	bool led2 = false;
	bool led3 = false;
	while (true) {
		char ch = controlpanel_promptChar("LED");
		switch (ch) {
			case 'e':
				debug_setLED(ERROR_LED, !error_led);
				error_led = !error_led;
				break;
			case 's':
				debug_setLED(ESTOP_LED, !estop_led);
				estop_led = !estop_led;
				break;
			case 't':
				debug_setLED(TICK_LED, !tick_led);
				tick_led = !tick_led;
				break;
			case '2':
				debug_setLED(OTHER2_LED, !led2);
				led2 = !led2;
				break;
			case '3':
				debug_setLED(OTHER3_LED, !led3);
				led3 = !led3;
				break;
			case 'b':
				debug_buzzerSolid(4);
				break;
			case 'B':
				debug_buzzerBeep(3);
				break;
			case 'q':
				debug_setLED(ERROR_LED, false);
				debug_setLED(ESTOP_LED, false);
				debug_setLED(TICK_LED, false);
				debug_setLED(OTHER2_LED, false);
				debug_setLED(OTHER3_LED, false);
				return;
			case '?':
				static const char msg[] PROGMEM =
					"LED commands:\n"
					"  e  - Error LED\n"
					"  s  - Estop LED\n"
					"  t  - Tick LED\n"
					"  2  - LED 2\n"
					"  3  - LED 3\n"
					"  b  - Solid Buzzer\n"
					"  B  - Beep Buzzer\n"
					"  q  - Back";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_sensor() {
	while (true) {
		char ch = controlpanel_promptChar("Sensors");
		switch (ch) {
			case 'e':
				controlpanel_encoder();
				break;
			case 'b':
				printf_P(PSTR("%f\n"), adc_sampleBattery());
				break;
			case 'm':
				controlpanel_magnetometer();
				break;
			case 's':
				controlpanel_sonar();
				break;
			case 'g':
				controlpanel_gps();
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Sensor menu:\n"
					"  e - Encoders\n"
					"  b - Battery Voltage\n"
					"  m - Magnetometer\n"
					"  s - Sonar\n"
					"  g - GPS\n"
					"  q - Back\n";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_gps() {
	char rawData;
	while (true) {
		char ch = controlpanel_promptChar("GPS");
		switch (ch) {
			case 'r':
				gps_printRaw();
				break;
			case 's':
				gps_printStatus();
				break;
			case 'p':
				gps_printPos();
				break;
			case 'l': {
				GPSPos pos = gps_getPos();
				printf_P(PSTR("Raw position: X: %f, Y: %f\n"), pos.X_Raw, pos.Y_Raw);
				break;
			}
			case 'o':
				gps_printOffset();
				break;
			case 'e':
				if (gps_getEnabled()) {
					gps_setEnabled(false);
					printf_P(PSTR("GPS has been disabled!\n"));
				} else {
					gps_setEnabled(true);
					printf_P(PSTR("GPS has been enabled!\n"));
				}
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Sensor menu:\n"
					"  r - Raw Data\n"
					"  s - GPS Status\n"
					"  p - Position\n"
					"  l - Global UTM Position\n"
					"  o - Global UTM Origin Location (calibrated)\n"
					"  e - Enable/Disable GPS Odom update\n"
					"  q - Back\n";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_encoder() {
	int ticks = 0;
	while (true) {
		char ch = controlpanel_promptChar("Encoders");
		switch (ch) {
			case 'p':
				ticks = encoder_get(LEFT_ENCODER);
				printf_P(PSTR("L Ticks: %d "), ticks);
				ticks = encoder_get(RIGHT_ENCODER);
				printf_P(PSTR("R Ticks: %d\n"), ticks);
				break;
			case 'r':
				encoder_reset(LEFT_ENCODER);
				encoder_reset(RIGHT_ENCODER);
				break;
			case 's':
				motorcontrol_setEnabled(true);
				printf_P(PSTR("L RPS: %f, R RPS: %f\n"), motorcontrol_getRPS(MOTOR_LEFT), motorcontrol_getRPS(MOTOR_RIGHT));
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Encoder menu:\n"
					"  p - position (ticks)\n"
					"  r - Reset counter\n"
					"  s - RPS of wheels\n"
					"  q - Back\n";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_magnetometer() {
	MagReading dir;
	float heading;
	float first_heading;
	while (true) {
		char ch = controlpanel_promptChar("Magnetometer");
		switch (ch) {
			case 'x':
				dir = mag_getReading();
				printf_P(PSTR("X: %d\n"), dir.x);
				break;
			case 'y':
				dir = mag_getReading();
				printf_P(PSTR("Y: %d\n"), dir.y);
				break;
			case 'z':
				dir = mag_getReading();
				printf_P(PSTR("Z: %d\n"), dir.z);
				break;
			case 'h':
				heading = magfollow_getHeading();
				printf_P(PSTR("Heading: %f\n"), heading);
				break;
			case 'j':
				printf_P(PSTR("Raw heading: %f\n"), magfollow_getRawHeading());
				break;
			case 'l':
				motorcontrol_setRPS(MOTOR_LEFT, -.1);
				motorcontrol_setRPS(MOTOR_RIGHT, .1);
				first_heading = magfollow_getHeading();
				heading = magfollow_getHeading();
				while (sign(heading - first_heading)*(heading - first_heading) < 1) {
					dir = mag_getReading();
					printf_P(PSTR("%d %d %d\n"), dir.x, dir.y, dir.z);
					heading = magfollow_getHeading();
				}
				while (heading != first_heading) {
					heading = magfollow_getHeading();
					dir = mag_getReading();
					printf_P(PSTR("%d %d %d\n"), dir.x, dir.y, dir.z);
				}
				while (sign(heading - first_heading)*(heading - first_heading) < 1) {
					dir = mag_getReading();
					printf_P(PSTR("%d %d %d\n"), dir.x, dir.y, dir.z);
					heading = magfollow_getHeading();
				}
				motorcontrol_setRPS(MOTOR_LEFT, 0);
				motorcontrol_setRPS(MOTOR_RIGHT, 0);
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Encoder menu:\n"
					"  x - x value\n"
					"  y - y value\n"
					"  z - z value\n"
					"  h - Heading\n"
					"  j - Raw Heading\n"
					"  l - Calibrate\n"
					"  q - Back\n";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_sonar() {
	while (true) {
		char ch = controlpanel_promptChar("Sonars");
		switch (ch) {
			case 'd':
				printf_P(PSTR("LDist: %f, FDist: %f\n"), sonar_getDist(LEFT_SONAR), sonar_getDist(FRONT_SONAR));
				break;
			case 't':
				printf_P(PSTR("Timer: %f\n"), sonar_getTimer());
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Encoder menu:\n"
					"  d - Distance (cm)\n"
					"  t - Timer\n"
					"  q - Back\n";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

void controlpanel_calibrate() {
	PIDGains newgains;
	while (true) {
		char ch = controlpanel_promptChar("Calibrate");
		switch (ch) {
			case 'm':
				if (controlpanel_promptGains("magfollow", magfollow_getGains(), newgains)) {
					magfollow_setGains(newgains);
					printf_P(PSTR("Gains set!\n"));
				} else {
					printf_P(PSTR("Cancelled.\n"));
				}
				break;
			case 't':
				if (controlpanel_promptGains("magturn", magfollow_getTurnGains(), newgains)) {
					magfollow_setTurnGains(newgains);
					printf_P(PSTR("Gains set!\n"));
				} else {
					printf_P(PSTR("Cancelled.\n"));
				}
				break;
			case 'd':
				if (controlpanel_promptGains("motorcontrol", motorcontrol_getGains(), newgains)) {
					motorcontrol_setGains(newgains);
					printf_P(PSTR("Gains set!\n"));
				} else {
					printf_P(PSTR("Cancelled.\n"));
				}
				break;
			case 'f':
				tests_feedforward();
				break;
			case 'q':
				return;
			case '?':
				static const char msg[] PROGMEM =
					"Calibrate Menu:\n"
					"  m - Magnetometer PID\n"
					"  t - Magturn PID\n"
					"  d - Motorcontrol PID\n"
					"  f - Feed-forward calibration !!MOVES MOTORS!!\n"
					"  q - Back\n";
				puts_P(msg);
				break;
			default:
				puts_P(unknown_str);
				break;
		}
	}
}

int controlpanel_prompt(const char *prompt, const char *fmt, ...) {
	va_list argp;
	va_start(argp, fmt);

	printf_P(PSTR("%s# "), prompt);

	char buf[32];
	fgets(buf, sizeof(buf), stdin);
	return vsscanf(buf, fmt, argp);
}

char controlpanel_promptChar(const char *prompt) {
	printf_P(PSTR("%s> "), prompt);

	char ch = getchar();
	putchar('\n');
	return ch;
}

bool controlpanel_promptGains(const char *name, const PIDGains &curgains, PIDGains &gains) {
	printf_P(PSTR("Setting gains for %s\n"), name);
	printf_P(PSTR("Current gains: P %.4f I %.4f D %.4f MaxI %.4f\n"), curgains.p, curgains.i, curgains.d, curgains.maxi);
	
	if (controlpanel_prompt("P", "%f", &gains.p) != 1)
		gains.p = curgains.p;
	if (controlpanel_prompt("I", "%f", &gains.i) != 1)
		gains.i = curgains.i;
	if (controlpanel_prompt("D", "%f", &gains.d) != 1)
		gains.d = curgains.d;
	if (controlpanel_prompt("MaxI", "%f", &gains.maxi) != 1)
		gains.maxi = curgains.maxi;
		
	printf_P(PSTR("New gains: P %.4f I %.4f D %.4f MaxI %.4f\n"), gains.p, gains.i, gains.d, gains.maxi);
	return controlpanel_promptChar("Ok? [y/n]") == 'y';
}
