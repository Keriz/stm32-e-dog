#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <audio/microphone.h>
#include "audio.h"
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


//#define MIN_VALUE_THRESHOLD	10000
#define MIN_VALUE_THRESHOLD	5000
#define PHASE_RAD_THRESHOLD 1

#define COEFF_MOBILE_MEAN 0.07


#define MIN_FREQ		15	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

static int16_t nothing=0; //counter
static int16_t found=0; //counter
static bool no_move=0; //counter

static int16_t max_norm_index_right = 0;
static int16_t max_norm_index_left = 0;;
static int16_t max_norm_index_back=0;
static float phase_delay_X=0;
static float phase_delay_Y=0;

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

	static uint16_t nb_samples = 0;
	float phase_right =0;
	float phase_left =0;
	float phase_back =0;

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

		//calculate the magnitude and find the highest peak
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		max_norm_index_right= highest_peak(micRight_output);
		max_norm_index_left = highest_peak(micLeft_output);
		max_norm_index_back = highest_peak(micBack_output);

		//chprintf((BaseSequentialStream *)&SD3,"max_norm_index=%dand%d\n",max_norm_index_right,max_norm_index_left);
		// if we find an index for the same value of frequency calculate the phase
		if(max_norm_index_right == max_norm_index_left  && max_norm_index_right != -1 && max_norm_index_left != -1){
			phase_right	= atan2(micRight_cmplx_input_temp[max_norm_index_right*2+1],micRight_cmplx_input_temp[max_norm_index_right*2]);
			phase_left	= atan2(micLeft_cmplx_input_temp[max_norm_index_left*2+1],micLeft_cmplx_input_temp[max_norm_index_left*2]);
			phase_back	= atan2(micBack_cmplx_input_temp[max_norm_index_back*2+1],micBack_cmplx_input_temp[max_norm_index_back*2]);

			float new_phase_delay_X = phase_right - phase_left;
			if(new_phase_delay_X > -PHASE_RAD_THRESHOLD && new_phase_delay_X < PHASE_RAD_THRESHOLD){ //filter
				//chprintf((BaseSequentialStream *)&SD3,"angle=%f\n",phase_delay_X*180/PI);


				phase_delay_X = COEFF_MOBILE_MEAN*new_phase_delay_X + (1-COEFF_MOBILE_MEAN)*phase_delay_X;

				if(phase_delay_X > 0){ //determine if we are in y>0 or y<0
					phase_delay_Y= phase_right-phase_back;
					//chprintf((BaseSequentialStream *)&SD3,"angle=%f\n",phase_delay_Y*180/PI);
				}
				else if(phase_delay_X <0){
					phase_delay_Y= phase_left-phase_back;
					//chprintf((BaseSequentialStream *)&SD3,"x<0 angle=%f\n",phase_delay_Y*180/PI);
				}
			}
			else{
				phase_delay_Y=NOT_FOUND;
				phase_delay_X=NOT_FOUND;
			}
			found++;
			if(found==10){
				no_move=0;
				found=0;
			}
		}
		else{ //not found
			//chprintf((BaseSequentialStream *)&SD3,"max_norm_index=%dand%d\n",max_norm_index_right,max_norm_index_left);
			//if not found for .. ms NO_MOVE ==1
			nothing++;
			if(nothing==10){
				no_move = 1;
				nothing=0;
			}

			phase_delay_Y	= NOT_FOUND;
			phase_delay_X 	= NOT_FOUND;
		}



		nb_samples = 0;

	}
}

bool move_or_not(void){
	if(no_move)
		return false; //don't move
	else
		return true; // move
}


float get_phase_delay_X(void){
	return phase_delay_X;
}

float get_phase_delay_Y(void){
	return phase_delay_Y;
}
