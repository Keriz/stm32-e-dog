#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <chprintf.h>
#include <audio.h>
#include <audio/microphone.h>

#include <sensors/proximity.h>
#include <process_move.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size) 
{	
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}

void SendStartToComputer(BaseSequentialStream* out) 
{	
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
}

void ReceiveStartFromComputer(BaseSequentialStream* in){

	volatile uint8_t c1

	uint8_t state = 0;
	while(state != 5){

        c1 = chSequentialStreamGet(in);

        //State machine to detect the string EOF\0S in order synchronize
        //with the frame received
        switch(state){
        	case 0:
        		if(c1 == 's')
        			state = 1;
        		else
        			state = 0;
        	case 1:
        		if(c1 == 't')
        			state = 2;
        		else if(c1 == 's')
        			state = 1;
        		else
        			state = 0;
        	case 2:
        		if(c1 == 'a')
        			state = 3;
        		else if(c1 == 's')
        			state = 1;
        		else
        			state = 0;
        	case 3:
        		if(c1 == 'r')
        			state = 4;
        		else if(c1 == 's')
        			state = 1;
        		else
        			state = 0;
        	case 4:
        		if(c1 == 't')
        			state = 5;
        		else if(c1 == 's')
        			state = 1;
        		else
        			state = 0;
        }
        
	}
}

void Send_value2(void)
{
	chprintf((BaseSequentialStream *)&SD3, "prox1_cali_=%d\n", get_calibrated_prox(1));
	chprintf((BaseSequentialStream *)&SD3, "prox0_cali_=%d\n", get_calibrated_prox(0));


}
void Send_value3(void)
{
	chprintf((BaseSequentialStream *)&SD3,"pos_to_reach_right=%d\n",VL53L0X_get_dist_mm());
}

static void timer12_start(void){
    //General Purpose Timer configuration
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void)
{

	halInit();
	chSysInit();
	mpu_init();

	serial_start();
	usb_start();
	motors_init();

	mic_start(&processAudioData);

	//proximity_start();
	//process_move_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	///essagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	//messagebus_topic_t *phase_topic = messagebus_find_topic_blocking(&bus, "/phase");
	//proximity_msg_t proximity_values;
    /* Infinite loop. */

	ReceiveStartFromComputer((BaseSequentialStream *) &SD3);

    while (1) {
    		//Done();
			wait_send_to_computer();

			SendStartToComputer((BaseSequentialStream *) &SD3);

			//LEFT, RIGHT
			//each mic out 2*4*(2*FFT_SIZE + FFT_SIZE)) = 24 576
			//we have 2 mics so the data size to be sent is 24 576 bytes each time we call this function
			//baud rate is 115 200 that means this needs to be called max 115200 / 24576 = 4 times per second
			//we call this func every 25 mic samples so every 250ms
			//the robot has to turn for 0.1ms and then it waits 0.2ms

			for (uint8_t i = 0; i < 2; i++){
				float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT + i);
				float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT + i);

            	SendFloatToComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, 2*FFT_SIZE);
				//SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);
			}

			//SendFloatToComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, 2*FFT_SIZE);
			//SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);

			turn_right(MOTOR_SPEED_LIMIT, 1);
    		//messagebus_topic_wait(phase_topic, &phase_value, sizeof(phase_value));
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
