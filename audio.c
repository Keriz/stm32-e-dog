#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <audio/microphone.h>
#include <math.h>
#include "arm_math.h"
#include "acoustic_sl.h"

#include "audio.h"
/*
 * USER DEFINES
 *
 */

#define SAMPLING_FREQUENCY 	16000 //160 samples every 10ms
#define NUMBER_CHANNELS		4
#define NUMBER_CHANNEL_PER_STREAM 4

//units in decimals of a millimeter
#define DISTANCE_MIC_LEFT_RIGHT 620
#define DISTANCE_MIC_FRONT_BACK 609

#define RIGHT_MIC 0
#define LEFT_MIC 1
#define FRONT_MIC 2
#define BOTTOM_MIC 3
/*
 * USER VARIABLES
 *
 */

static AcousticSL_Handler_t   libSoundSourceLoc_Handler_Instance;
static AcousticSL_Config_t    libSoundSourceLoc_Config_Instance;

//pulse coded modulation mic buffer (right, left, front, bottom)
// divided by 1000 because the alrgotihm runs with 1ms of data
static uint16_t micBufferIN[4*SAMPLING_FREQUENCY/1000];



uint16_t acoustic_init(void){
	uint16_t error_value = 0;

	libSoundSourceLoc_Handler_Instance.channel_number = 4;
	libSoundSourceLoc_Handler_Instance.M12_distance = DISTANCE_MIC_LEFT_RIGHT;
	libSoundSourceLoc_Handler_Instance.M34_distance = DISTANCE_MIC_FRONT_BACK;
	libSoundSourceLoc_Handler_Instance.sampling_frequency = SAMPLING_FREQUENCY;
	libSoundSourceLoc_Handler_Instance.algorithm = ACOUSTIC_SL_ALGORITHM_GCCP;
	libSoundSourceLoc_Handler_Instance.ptr_M1_channels = NUMBER_CHANNEL_PER_STREAM;
	libSoundSourceLoc_Handler_Instance.ptr_M2_channels = NUMBER_CHANNEL_PER_STREAM;
	libSoundSourceLoc_Handler_Instance.ptr_M3_channels = NUMBER_CHANNEL_PER_STREAM;
	libSoundSourceLoc_Handler_Instance.ptr_M4_channels = NUMBER_CHANNEL_PER_STREAM;
	libSoundSourceLoc_Handler_Instance.samples_to_process = 512;
	AcousticSL_getMemorySize( &libSoundSourceLoc_Handler_Instance);
	libSoundSourceLoc_Handler_Instance.pInternalMemory=(uint32_t *)malloc(libSoundSourceLoc_Handler_Instance.internal_memory_size);

	if(libSoundSourceLoc_Handler_Instance.pInternalMemory == NULL)
	{
	while(1); /*Error Management*/
	}

	error_value = AcousticSL_Init( &libSoundSourceLoc_Handler_Instance);

	if(error_value != 0)
	{
	while(1); /*Error Management*/
	}

	/*Setup Source Localization dynamic parameters*/
	libSoundSourceLoc_Config_Instance.resolution = 10;
	libSoundSourceLoc_Config_Instance.threshold = 15000;
	error_value = AcousticSL_setConfig(&libSoundSourceLoc_Handler_Instance, &libSoundSourceLoc_Config_Instance);

	if(error_value != 0)
	{
	while(1); /*Error Management*/
	}

	return error_value;

}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	It computes the angle
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/

//static int32_t result[4];
void processAudioData(int16_t *data, uint16_t num_samples){

	int32_t result[4];

	for (uint16_t i = 0; i < 16; ++i){

		micBufferIN[i*4+RIGHT_MIC] 	= data[i*4+RIGHT_MIC];
		micBufferIN[i*4+LEFT_MIC] 	= data[i*4+LEFT_MIC];
		micBufferIN[i*4+FRONT_MIC] 	= data[i*4+FRONT_MIC];
		micBufferIN[i*4+BOTTOM_MIC] = data[i*4+BOTTOM_MIC];

		if(i >= (4*SAMPLING_FREQUENCY/1000) ){
					break;
				}
	}

	//for (uint16_t j = 0; j < (4*SAMPLING_FREQUENCY/1000); ++j){
	//	chprintf((BaseSequentialStream *)&SD3, "micbuffer= %d\n",micBufferIN[j]);
	//}


	if(AcousticSL_Data_Input((int16_t *)&micBufferIN[LEFT_MIC], (int16_t *)&micBufferIN[RIGHT_MIC],
			(int16_t *)&micBufferIN[BOTTOM_MIC], (int16_t *)&micBufferIN[FRONT_MIC], &libSoundSourceLoc_Handler_Instance)){

		AcousticSL_Process((int32_t *)result, &libSoundSourceLoc_Handler_Instance);

		chprintf((BaseSequentialStream *)&SD3, "result before =%d\n",result[0] );
		if(result[0]!=ACOUSTIC_SL_NO_AUDIO_DETECTED)
		{
		result[0]=(result[0] - 45);
		if(result[0]<0)
		  result[0]+=360;
		chprintf((BaseSequentialStream *)&SD3, "result=%d\n",result[0] );
		}

	}
}

//int32_t get_degree(void) {
//	return result[0];
//}


