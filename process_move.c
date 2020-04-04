#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <process_move.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include <sensors/proximity.h>

static int16_t position_to_reach_right = 0;	    // in [step]
static int16_t position_to_reach_left = 0;	    // in [step]
static int16_t counter_step_left = 0; 		    // in [step]
static int16_t counter_step_right = 0;

void go_forward(void){
	right_motor_set_speed(MOTOR_SPEED);
	left_motor_set_speed(MOTOR_SPEED);
}

void turn_right(void){
	right_motor_set_speed(-MOTOR_SPEED);
	left_motor_set_speed(MOTOR_SPEED);
}

void turn_left(void){
	right_motor_set_speed(MOTOR_SPEED);
	left_motor_set_speed(-MOTOR_SPEED);
}

void counter_motor_step_init(int32_t counter_value){
	left_motor_set_pos(counter_value);
	right_motor_set_pos(counter_value);
}

void motor_set_position(float position_r, float position_l)
{
	//Set global variable with position to reach in step
	position_to_reach_left = position_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = position_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	chprintf((BaseSequentialStream *)&SD3, "pos_to_reach=%d\n", position_to_reach_left);
}

void advance_or_turn_x_left(int x, bool unit){
	float dist_left;
	float dist_right;
	counter_motor_step_init(0);

	//if x in degree convert to dist
	if(unit){
		dist_left=  x* PERIMETER_EPUCK /(360) ;
		dist_right= x* PERIMETER_EPUCK /(360);
		motor_set_position(dist_right,dist_left);
	}
	else //not an angle direct convert cm to step
		motor_set_position(x,x);

	while(abs(position_to_reach_left) > abs(counter_step_left)){
		Send_value2();
		counter_step_left= left_motor_get_pos();
		turn_left();
	}
}

void advance_or_turn_x_right(int x, bool unit){
	float dist_left;
	float dist_right;
	counter_motor_step_init(0);

	//if x in degree convert to dist
	if(unit){
		dist_left=  x* PERIMETER_EPUCK /(360) ;
		dist_right= x* PERIMETER_EPUCK /(360);
		motor_set_position(dist_right,dist_left);
	}
	else //not an angle direct convert cm to step
		motor_set_position(x,x);

	while(abs(position_to_reach_right) > abs(counter_step_right)){
		counter_step_right= right_motor_get_pos();
		turn_right();
	}
}

void motor_stop(void){
	right_motor_set_speed(STOP);
	left_motor_set_speed(STOP);
}

static THD_WORKING_AREA(waProcessmove, 256);
static THD_FUNCTION(Processmove, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    uint16_t  dist_front_TL;
    int ir_sensor7;
    int ir_sensor6;

    while(1){
    		dist_front_TL = VL53L0X_get_dist_mm();
    		ir_sensor7 = get_calibrated_prox(7);
    		ir_sensor6 = get_calibrated_prox(6);
    		Send_value2();
    		//if (dist_front_TL > 100){

    			if( ir_sensor6 >= COLLISION || ir_sensor7 >= COLLISION){
    				while( ir_sensor6 > COLLISION || ir_sensor7 > COLLISION ){
    					ir_sensor7 = get_calibrated_prox(7);
    					ir_sensor6 = get_calibrated_prox(6);
    					//Done();
    					turn_right();
    				}
    			//}


    		}
    			go_forward();
    		//else{
    			//Done();
    		//	motor_stop();
    		//}
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


void process_move_start(void){
	chThdCreateStatic(waProcessmove, sizeof(waProcessmove), NORMALPRIO, Processmove, NULL);
}
