#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <process_move.h>
#include "audio.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include <sensors/proximity.h>


#define MOTOR_SPEED   		338 // [step/s]
#define STOP  				0
#define COLLISION   			600
#define DIRECTION_CHANGED   	0

#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     13 // [cm]

#define TOLERANCE_DELAY_PHASE 	5
#define GOAL_ANGLE				0

#define KP						50.0f

static int16_t position_to_reach_right = 0;	    // in [step]
static int16_t counter_step_right = 0;

void go_forward(void){
	right_motor_set_speed(MOTOR_SPEED);
	left_motor_set_speed(MOTOR_SPEED);
}

void turn_right(int16_t speed){
	right_motor_set_speed(-speed);
	left_motor_set_speed(speed);
}

void turn_left(int16_t speed){
	right_motor_set_speed(speed);
	left_motor_set_speed(-speed);
}

void go_forward_x_cm(float dist)
{
	//initialization
	right_motor_set_pos(0);
	position_to_reach_right = dist * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	while(abs(position_to_reach_right) > abs(counter_step_right)){
		counter_step_right= right_motor_get_pos();
		go_forward();
	}
}

int16_t pi_regulator(float distance, float goal){
	float error=0;
	float speed=0;
	error= distance - goal;

	speed = KP * error;

	return (int16_t)speed;
}


void motor_stop(void){
	right_motor_set_speed(STOP);
	left_motor_set_speed(STOP);
}


static THD_WORKING_AREA(waProcessMove, 256);
static THD_FUNCTION(ProcessMove, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t speed;

    while(1){
    		if(get_detection()){
    			float phase_delay_x= get_phase_delay_X()*180/PI;
    			float phase_delay_y= get_phase_delay_Y()*180/PI;
    			speed= pi_regulator(phase_delay_x,GOAL_ANGLE);

    			if(phase_delay_x > TOLERANCE_DELAY_PHASE || phase_delay_x < -TOLERANCE_DELAY_PHASE ){ //if the sound in front of the robot do nothing
    				if(phase_delay_x < 0)
    					turn_left(abs(speed));
    				if(phase_delay_x > 0)
    					turn_right(abs(speed));
    			}
    			//case when the sound is just behind the robot
    			else if((phase_delay_x > 0 && phase_delay_y < 0) || (phase_delay_x < 0 && phase_delay_y < 0))
    				turn_right(abs(speed));
    			else
    				move_and_escape();
    		}
    		else
    			motor_stop();
    	}

}

void move_and_escape(void){ //add after
	int ir_sensor7 = get_calibrated_prox(7);
	int ir_sensor6 = get_calibrated_prox(6);
	int ir_sensor1 = get_calibrated_prox(1);
	int ir_sensor0 = get_calibrated_prox(0);

	if( ir_sensor6 >= COLLISION || ir_sensor7 >= COLLISION){
	   while(ir_sensor6 >= COLLISION || ir_sensor7 >= COLLISION){
	    		ir_sensor7 = get_calibrated_prox(7);
	    		ir_sensor6 = get_calibrated_prox(6);
	    		turn_right(MOTOR_SPEED);
	    	}
	   go_forward_x_cm(10);
	}
	if( ir_sensor0 >= COLLISION || ir_sensor1 >= COLLISION){
	    while(ir_sensor0 >= COLLISION || ir_sensor1 >= COLLISION){
	    		ir_sensor1 = get_calibrated_prox(1);
	    		ir_sensor0 = get_calibrated_prox(0);
	    		turn_left(MOTOR_SPEED);
	    }
	    go_forward_x_cm(10);
	}
	go_forward();
}

void process_move_start(void){
	chThdCreateStatic(waProcessMove, sizeof(waProcessMove), NORMALPRIO-1, ProcessMove, NULL);
}
