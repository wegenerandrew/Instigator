#ifndef TICK_H_
#define TICK_H_

#define TICK_HZ 200
#define TICK_TIMHZ 4E6
#define TICK_TIMMAX (TICK_TIMHZ/TICK_HZ)

void tick_init();
void tick_suspend();
void tick_resume();

#endif
