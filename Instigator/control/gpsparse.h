#ifndef GPSPARSE_H_
#define GPSPARSE_H_

void gpsparse_update();
void gpsparse_printRawData();

void gpsparse_printHeader();
void gpsparse_addHeader(char ch);
void gpsparse_addWeek(char ch);
void gpsparse_addSeconds(char ch);
void gpsparse_addDiffLag(char ch);
void gpsparse_addSats(char ch);
void gpsparse_addLat(char ch);
void gpsparse_addLon(char ch);
void gpsparse_addHgt(char ch);
void gpsparse_addUndulation(char ch);
void gpsparse_addDatum(char ch);
void gpsparse_addLatStd(char ch);
void gpsparse_addLonStd(char ch);
void gpsparse_addSolStatus(char ch);
void gpsparse_addRt20Status(char ch);
void gpsparse_addFixSat(char ch);
void gpsparse_addIdle(char ch);
void gpsparse_addStnID(char ch);
void gpsparse_addChecksum(char ch);

#endif
