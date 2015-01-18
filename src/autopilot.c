/*
 * autopilot.c
 *
 *  Created on: 17 sty 2015
 *      Author: zapol
 */

#include "sensors/drv_mpu6050.h"
#include "board.h"

int16_t angle[2];

void autopilotInit(void)
{
    printf("Initializing autopilot...\n");

    mpu6050Init();
//    mpu6050DmpInitialize();
    if(mpu6050Detect(0,0))
    {
    	printf("MPU6050 detected\n");
    }
    else
    {
    	printf("MPU6050 failed\n");
    }
}

uint8_t autopilotRun(uint16_t *inppm, uint16_t *outppm)
{
	for(int i=0;i!=PPM_CHANNELS;i++)outppm[i]=inppm[i];
//	mpu6050DmpLoop();
//	printf("Angles: %d, %d", angle[0], angle[1]);
	return false;
}

void readInertial(void)
{
}

