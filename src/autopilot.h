/*
 * autopilot.h
 *
 *  Created on: 17 sty 2015
 *      Author: zapol
 */

#ifndef SRC_AUTOPILOT_H_
#define SRC_AUTOPILOT_H_

#define AUTONOMY_INTERVAL_US 10000

void autopilotInit(void);
uint8_t autopilotRun(uint16_t *inppm, uint16_t *outppm);
void readInertial(void);


#endif /* SRC_AUTOPILOT_H_ */
