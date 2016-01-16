#ifndef __ENCODER_H
#define __ENCODER_H		

#include "sys.h"

#define ENCODER_INIT_VALUE 32768 

extern s16 SpeedL, SpeedR;
 
void EncoderL_Init(void);
void EncoderR_Init(void);
void GetWheelSpeed(void);

#endif
