/*
 * autopilot.h
 *
 *  Created on: 17 sty 2015
 *      Author: zapol
 */

#ifndef SRC_AUTOPILOT_H_
#define SRC_AUTOPILOT_H_

#define AUTONOMY_INTERVAL_US 5000
#define AUTOPILOT_RUDDER_CHANNEL 0
#define AUTOPILOT_ELEVATOR_CHANNEL 1
#define AUTOPILOT_THROTTLE_CHANNEL 2
#define AUTOPILOT_AILERONS_CHANNEL 3
#define AUTOPILOT_FLAPS_CHANNEL 4
#define AUTOPILOT_EN_CHANNEL 5

void autopilotInit(void);
uint8_t autopilotRun(uint16_t *inppm, uint16_t *outppm);
void readInertial(void);


#endif /* SRC_AUTOPILOT_H_ */
