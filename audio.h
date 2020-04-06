#ifndef AUDIO_H
#define AUDIO_H

#include <stdlib.h>

uint16_t acoustic_init(void);
int32_t get_degree(void);

void processAudioData(int16_t *data, uint16_t num_samples);

#endif
