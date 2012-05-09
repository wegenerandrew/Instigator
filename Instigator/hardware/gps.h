#ifndef GPS_H_
#define GPS_H_

void gps_init();
bool gps_put(char ch);
int gps_puts(const char *buf);
int gps_get();
void gps_putch(char ch);
void gps_tick();
void gps_printRaw();
void gps_printStatus();
void gps_printPos();

#endif
