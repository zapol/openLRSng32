/*
 * pid.c
 *
 *  Created on: 18 sty 2015
 *      Author: zapol
 */

#include "pid.h"

float pid(struct pid *params, float error)
{
	float output=0;
	output += params->p * error;

	output += (params->last_err) * params->d;
	params->last_err = error;

	params->ibuf += error;
	if(params->ibuf >= params->irange) params->ibuf = params->irange;
	if(params->ibuf <= -params->irange) params->ibuf = -params->irange;
	output += params->ibuf * params->i;
	return output;
}
