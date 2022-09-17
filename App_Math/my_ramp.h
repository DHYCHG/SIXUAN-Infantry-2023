#ifndef _MY_RAMP_H
#define _MY_RAMP_H
#include "stm32f4xx_hal.h"
#define mode0to1 0
#define moden1top1 1
#define mode_sin 2

typedef struct
{
	float out_x;
	float out;
	float T;
	int mode;
}myrampGen_t;

void my_ramp_init(myrampGen_t *ramp,float setcale, int mode);
int ifoverformyramp(myrampGen_t *ramp);
float ramp_calc(myrampGen_t *ramp,float T);
void ramp_clear(myrampGen_t *ramp);
#endif

