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
	chprintf((BaseSequentialStream *)&SD3, "counterright_=%d\n", right_motor_get_pos());
	chprintf((BaseSequentialStream *)&SD3, "counterleft_=%d\n", left_motor_get_pos());

}
void Send_value3(void)
{
	chprintf((BaseSequentialStream *)&SD3,"VLdistance_cm0_=%d\n",VL53L0X_get_dist_mm());
}
void Done(void)
{
	chprintf((BaseSequentialStream *)&SD3,"turn\n");
}

int main(void)
{

	halInit();
	chSysInit();
	mpu_init();

	serial_start();
	usb_start();
	motors_init();

	motor_set_position(PERIMETER_EPUCK/2,PERIMETER_EPUCK/2);
	motor_one_turn();

	//turn_x_degree(360);
	proximity_start();
	process_move_start();
	
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t proximity_values;
    /* Infinite loop. */
    while (1) {
    		messagebus_topic_wait(proximity_topic, &proximity_values, sizeof(proximity_values));
    		Send_value2();
    		chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
