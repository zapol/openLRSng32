/*
 * autopilot.c
 *
 *  Created on: 17 sty 2015
 *      Author: zapol
 */

#include "sensors/drv_mpu6050.h"
#include "board.h"
#include "pid.h"

int16_t angle[2];

void autopilotInit(void)
{
    printf("Initializing autopilot...\n");

    mpu6050Init();
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
	struct mpu6050data *imuData;
	float error, output;
	// PID parameters: P, I, D, iran, ibuf, lasterr
	static struct pid elevator_pid = {1.0,1.0,1.0,100.0,0.0,0.0};
	static struct pid ailerons_pid = {1.0,1.0,1.0,100.0,0.0,0.0};

	imuData = mpu6050GetData();
	printf("Yaw: %4d, Pitch: %4d, Roll: %4d\n", (int)(imuData->yaw*360/M_PI), (int)(imuData->pitch*360/M_PI), (int)(imuData->roll*360/M_PI));

	if(inppm[AUTOPILOT_EN_CHANNEL] > 512)
	{
		printf("Autopilot ON\n");
		// Rudder - just copy
		outppm[AUTOPILOT_RUDDER_CHANNEL] = inppm[AUTOPILOT_RUDDER_CHANNEL];

		// Elevator - regulate
		error = (float)(inppm[AUTOPILOT_ELEVATOR_CHANNEL]-512.0)/4.0 - imuData->pitch/M_PI*360;
		output = pid(&elevator_pid, error);
		if(output>512)output = 512;
		if(output<-512)output = -512;
		outppm[AUTOPILOT_ELEVATOR_CHANNEL] = (uint16_t)output;

		// Throttle - just copy
		outppm[AUTOPILOT_THROTTLE_CHANNEL] = inppm[AUTOPILOT_THROTTLE_CHANNEL];

		// Ailerons - regulate
		error = (float)(inppm[AUTOPILOT_AILERONS_CHANNEL]-512.0)/4.0 - imuData->roll/M_PI*360;
		output = pid(&ailerons_pid, error);
		if(output>512)output = 512;
		if(output<-512)output = -512;
		outppm[AUTOPILOT_AILERONS_CHANNEL] = (uint16_t)output;

		// Flaps - just copy
		outppm[AUTOPILOT_FLAPS_CHANNEL] = inppm[AUTOPILOT_FLAPS_CHANNEL];
		return true;
	}
	else
	{
		printf("Autopilot OFF\n");
		for(int i=0;i!=PPM_CHANNELS;i++)outppm[i]=inppm[i];
		return false;
	}
}

void readInertial(void)
{
}

