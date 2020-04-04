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

void motor_set_position(float x, bool unit )
{
	float dist_left; //useless?
	float dist_right;
	if(unit){ //if x in degree convert to dist
		dist_left=  x* PERIMETER_EPUCK /(360);
		dist_right= x* PERIMETER_EPUCK /(360); //useless?
		x= dist_right;
	}
	//Set global variable with position to reach in step
	position_to_reach_left = x * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = x * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	//chprintf((BaseSequentialStream *)&SD3, "pos_to_reach=%d\n", position_to_reach_left);
}

void advance_or_turn_x_left(float x, bool unit){
	counter_motor_step_init(0);
	motor_set_position(x,unit);
	while(abs(position_to_reach_left) > abs(counter_step_left)){
		counter_step_left= left_motor_get_pos();
		turn_left();
	}
}

void advance_or_turn_x_right(float x, bool unit){
	counter_motor_step_init(0);
	motor_set_position(x,unit);
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
    int ir_sensor1;
    int ir_sensor0;

    while(1){
    		dist_front_TL = VL53L0X_get_dist_mm();
    		ir_sensor7 = get_calibrated_prox(7);
    		ir_sensor6 = get_calibrated_prox(6);
    		ir_sensor1 = get_calibrated_prox(1);
    		ir_sensor0 = get_calibrated_prox(0);
    		if( ir_sensor6 >= COLLISION || ir_sensor7 >= COLLISION){
    			while(ir_sensor6 >= COLLISION || ir_sensor7 >= COLLISION){
    				ir_sensor7 = get_calibrated_prox(7);
    				ir_sensor6 = get_calibrated_prox(6);
    				turn_right();
    			}
    			advance_or_turn_x_right(30,true);
    		}
    		if( ir_sensor0 >= COLLISION || ir_sensor1 >= COLLISION){
    		    	while(ir_sensor0 >= COLLISION || ir_sensor1 >= COLLISION){
    		    		ir_sensor1 = get_calibrated_prox(1);
    		    		ir_sensor0 = get_calibrated_prox(0);
    		    		turn_left();
    		    	}
    		    	advance_or_turn_x_left(30,true);
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
