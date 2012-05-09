#include "control/gpsparse.h"
#include "hardware/gps.h"

#include <stdio.h>
#include <avr/pgmspace.h>

static char gpsparse_newData[] = "";
static char gpsparse_validData[] = "";

void gpsparse_printRawData() {
	int ptr = 0;
	while (gpsparse_validData[ptr]) {
		printf_P(PSTR("%c"), gpsparse_validData[ptr]);
		ptr++;
	}
	printf_P(PSTR("\n"));
}
