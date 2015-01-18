/*
 * mixer.h
 *
 *  Created on: 18 sty 2015
 *      Author: zapol
 */

#include <stdint.h>

#ifndef SRC_MIXER_H_
#define SRC_MIXER_H_

#define MIXER_CHANNELS 32
#define PPM_CHANNELS 16

#define MIXER_CHANNEL_EMPTY	0xff
#define MIXER_CHANNEL_ZERO	0xfe
#define MIXER_CHANNEL_ONE	0xfd

struct __attribute__((__packed__)) mixer_config {
	uint8_t srcChannel;
	uint8_t dstChannel;
	int16_t scaling;
	int16_t offset;
};

extern struct mixer_config mixer_config[MIXER_CHANNELS];

uint16_t mixerReadEeprom(void);
void mixerWriteEeprom(void);
void mixerInitDefaults(void);
void mixerExecute(uint16_t *inppm, uint16_t *outppm);

#endif /* SRC_MIXER_H_ */
