#ifndef SONAR_H_
#define SONAR_H_

static const int sonar_count = 3;

enum Sonar {
	MIDDLE_SONAR,
	LEFT_SONAR,
	RIGHT_SONAR
};

void sonar_init();
float sonar_getDist(Sonar sonar);
float sonar_getTimer();
float sonar_getStart(Sonar sonar);

#endif