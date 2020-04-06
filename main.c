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

#include <camera/po8030.h>

#include <sensors/proximity.h>
#include "sensors/VL53L0X/VL53L0X.h"
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
void Done(void)
{
	chprintf((BaseSequentialStream *)&SD3,"wesh\n");
}
void Done2(void)
{
	chprintf((BaseSequentialStream *)&SD3,"boucle\n");
}

int main(void)
{

	halInit();
	chSysInit();
	mpu_init();

	serial_start();
	usb_start();
	motors_init();

	acoustic_init();
	mic_start(&processAudioData);

	//if degree is possible
	//int32_t degree= get_degree();
	//advance_or_turn_x_left(degree ,true);

	//turn_x_degree(360);
	//parameter(x, angle(true) or cm(false)?, right(true) or left(false)?)
	//advance_or_turn_x_left(360,true);
	//advance_or_turn_x_right(360,true);


	proximity_start();
	//process_move_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t proximity_values;
    /* Infinite loop. */
    while (1) {
    		//Done();
    		messagebus_topic_wait(proximity_topic, &proximity_values, sizeof(proximity_values));
    		chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
