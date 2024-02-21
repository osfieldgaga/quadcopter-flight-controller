#ifndef GLOBALS
#define GLOBALS

#define X 0 // pitch
#define Y 1 // roll
#define Z 2 // yaw

extern unsigned long int loop_timer;
extern unsigned long int now; // Exists just to reduce the calls to micros()
extern int period;


#endif