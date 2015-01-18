/*
 * pid.h
 *
 *  Created on: 18 sty 2015
 *      Author: zapol
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

struct pid{
	float p;
	float i;
	float d;
	float irange;
	float ibuf;
	float last_err;
};

float pid(struct pid *params, float error);

#endif /* SRC_PID_H_ */
