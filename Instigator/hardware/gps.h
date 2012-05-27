#ifndef GPS_H_
#define GPS_H_

struct GPSHealth {
	float Lat_Std_Dev;
	float Lon_Std_Dev;
	int satellites;
	int solStatus;
	int rt20Status;
	int fixStatus;
	bool valid;
};

struct GPSPos {
	float X_Actual;
	float Y_Actual;
	float X_Raw;
	float Y_Raw;
};

const float gps_base_offset = .6;
const float gps_std_dev_cutoff = .5;		// How small should our std dev be before we start using the data?

void gps_init();
void gps_setEnabled(bool new_enabled);
bool gps_getEnabled();
bool gps_put(char ch);
int gps_puts(const char *buf);
int gps_get();
void gps_putch(char ch);
void gps_tick();
GPSHealth gps_getHealth();
GPSPos gps_getPos();
void gps_setOffset(float x, float y);
void gps_printRaw();
void gps_printStatus();
void gps_printPos();
void gps_printOffset();

#endif
