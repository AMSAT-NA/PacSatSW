/*
 * audio.h
 *
 *  Created on: Dec 19, 2012
 *      Author: fox
 */

#ifndef AUDIO_H_
#define AUDIO_H_

#ifdef LEGACY_GOLF
#include <pacsat.h>

typedef enum {
	MixerSilence=0,MixerInit,MixerData,MixerTone,MixerOneZero,MixerSquare,MixerSilenceAndSwitch
} Mixer_Source;
/*
 * Routine prototypes
 */

void TelemetryRadioTask(void *pvParameters);


void AudioTransponderOff(void);
void AudioTransponderOn(void);
void AudioCWToneOn(void);
void AudioCWToneOff(void);
void AudioSetMixerSource(Mixer_Source src);

/*
 * Telemetry fetching
 */

uint8_t GetPAPowerFlagCnt(void);
uint8_t GetDCTPowerFlagCnt(void);

#endif

#endif /* AUDIO_H_ */
