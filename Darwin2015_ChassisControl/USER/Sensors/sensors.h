#ifndef __SENSORS_H
#define __SENSORS_H

#include "stm32f10x.h"

#define US1_TRIG PEout(0)	
#define US1_ECHO PEin(1)
#define US2_TRIG PEout(2)	
#define US2_ECHO PEin(3)

#define BMP1 PEin(8)
#define BMP2 PEin(9)
#define BMP3 PEin(10)
#define BMP4 PEin(11)
#define BMP5 PEin(12)
#define BMP6 PEin(13)

#define GroundIR_R	PDin(6)
#define GroundIR_L 	PDin(7)

#define MINSPEED 30
#define ABS(x) (x>=0?x:-x)

typedef enum{FALSE=0,TRUE}BOOL;

extern u8 IRs;
extern float US1_Dis;
extern float US2_Dis;
extern u8 Bumpers;

void Sensors_Init(void);

void GetIRSensors(void);

void TrigUS1(void);
void TrigUS2(void);
void GetUS1Data(void);
void GetUS2Data(void);

void GetBumpers(void);

#endif
