#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <process_audio.h>
#include <communications.h>

#include <arm_math.h>
#include <arm_const_structs.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

static float micLeft_cmplx_input_temp[2 * FFT_SIZE];
static float micRight_cmplx_input_temp[2 * FFT_SIZE];
static float micBack_cmplx_input_temp[2 * FFT_SIZE];


#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing


static int16_t max_norm_index_right = 0;
static int16_t max_norm_index_left = 0;;
static int16_t max_norm_index_back=0;

void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}

int16_t highest_peak(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	return max_norm_index;
}


/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*/
void processAudioData(int16_t *data, uint16_t num_samples){
	static uint8_t mustSend = 0;
	static uint16_t nb_samples = 0;
	double phase_right =0;
	double phase_left =0;
	double reel_right=0;
	double imag_right=0;
	double reel_left=0;
	double imag_left=0;
	double dephasage=0;


	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){


		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		//stock mic Right and mic Left in temporaly buffer
		//after change i=minfreq and i<maxfreq
		for(uint16_t i = 0 ; i  < 2 * FFT_SIZE ; i++){
				//construct an array of complex numbers. Put 0 to the imaginary part
				micRight_cmplx_input_temp[i] =micRight_cmplx_input[i] ;
				micLeft_cmplx_input_temp[i] = micLeft_cmplx_input[i];
				micBack_cmplx_input_temp[i] = micBack_cmplx_input[i];
		}

		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		max_norm_index_right= highest_peak(micRight_output);
		max_norm_index_left = highest_peak(micLeft_output);
		max_norm_index_back = highest_peak(micBack_output);

		//chprintf((BaseSequentialStream *)&SD3,"max_index=%d_and=%d\n",max_norm_index_right,max_norm_index_left);

		if(max_norm_index_right == max_norm_index_left && max_norm_index_right != -1){
			//reel_right= micRight_cmplx_input_temp[max_norm_index_right*2];
			//imag_right= micRight_cmplx_input_temp[max_norm_index_right*2+1];
			//phase_right= atan2(imag_right,reel_right);
			phase_right= atan2(micRight_cmplx_input_temp[max_norm_index_right*2+1],micRight_cmplx_input_temp[max_norm_index_right*2]);


			//reel_left= micLeft_cmplx_input_temp[max_norm_index_left*2];
			//imag_left= micLeft_cmplx_input_temp[max_norm_index_left*2+1];
			//phase_left= atan2(imag_left,reel_left);
			phase_left= atan2(micLeft_cmplx_input_temp[max_norm_index_left*2+1],micLeft_cmplx_input_temp[max_norm_index_left*2]);
			dephasage= phase_right - phase_left;
			if(dephasage > -1 && dephasage < 1){
				chprintf((BaseSequentialStream *)&SD3,"angle=%f\n",dephasage*180/3.14);
				//chprintf((BaseSequentialStream *)&SD3,"max_index=%dand=%d\n",max_norm_index_right,max_norm_index_left);
				//chprintf((BaseSequentialStream *)&SD3,"rad=%f\n",dephasage);
			}

		}
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
