/*
 * mixer.c
 *
 *  Created on: 18 sty 2015
 *      Author: zapol
 */

#include "mixer.h"
#include "board.h"

struct mixer_config mixer_config[MIXER_CHANNELS];

uint16_t mixerReadEeprom(void)
{
	return 0;
}

void mixerWriteEeprom(void)
{

}

void mixerInitDefaults(void)
{
	for(int i=0;i!=MIXER_CHANNELS;i++)
	{
		if(i<PPM_CHANNELS)
		{
			mixer_config[i].srcChannel=i;
			mixer_config[i].dstChannel=i;
		}
		else
		{
			mixer_config[i].srcChannel=0xff;
			mixer_config[i].dstChannel=0xff;
		}
		mixer_config[i].scaling=100;
		mixer_config[i].offset=0;
	}

	mixer_config[0].srcChannel=2;
	mixer_config[0].dstChannel=0;
	mixer_config[0].scaling=100;
	mixer_config[0].offset=0;

	mixer_config[1].srcChannel=1;
	mixer_config[1].dstChannel=1;
	mixer_config[1].scaling=100;
	mixer_config[1].offset=-150;

	mixer_config[2].srcChannel=0;
	mixer_config[2].dstChannel=2;
	mixer_config[2].scaling=-100;
	mixer_config[2].offset=-100;

	mixer_config[3].srcChannel=3;
	mixer_config[3].dstChannel=3;
	mixer_config[3].scaling=-100;
	mixer_config[3].offset=50;

	mixer_config[4].srcChannel=3;
	mixer_config[4].dstChannel=4;
	mixer_config[4].scaling=-100;
	mixer_config[4].offset=-170;

	mixer_config[5].srcChannel=4;
	mixer_config[5].dstChannel=5;
	mixer_config[5].scaling=40;
	mixer_config[5].offset=-300;

	mixer_config[6].srcChannel=4;
	mixer_config[6].dstChannel=6;
	mixer_config[6].scaling=40;
	mixer_config[6].offset=-300;
}

void mixerExecute(uint16_t *inppm, uint16_t *outppm)
{
	struct mixer_config *cfg;
	int16_t tempPPM[PPM_CHANNELS];

	for(int i=0;i!=PPM_CHANNELS;i++) tempPPM[i]=0;

	for(int i=0;i!=MIXER_CHANNELS;i++)
	{
		cfg = &mixer_config[i];
		if(cfg->dstChannel<PPM_CHANNELS)
		{
			int src;
			if(cfg->srcChannel<PPM_CHANNELS)
				src = inppm[cfg->srcChannel]-512;
			else if(cfg->srcChannel==MIXER_CHANNEL_ONE)
				src = 512;
			else //if(cfg->srcChannel==MIXER_CHANNEL_ZERO)
				src = 0;

			tempPPM[cfg->dstChannel] += (src*cfg->scaling)/100 + cfg->offset;
		}
	}

	for(int i=0;i!=PPM_CHANNELS;i++)
	{
		if(tempPPM[i]<=-512) outppm[i]=0;
		else if(tempPPM[i]>=512) outppm[i]=1024;
		else outppm[i]=tempPPM[i]+512;
	}

}
