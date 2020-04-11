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

	proximity_start();
	process_move_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	//messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	//messagebus_topic_t *phase_topic = messagebus_find_topic_blocking(&bus, "/phase");
	//proximity_msg_t proximity_values;
    /* Infinite loop. */
    while (1) {
    		//Done();
    		//messagebus_topic_wait(phase_topic, &phase_value, sizeof(phase_value));
    		chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
