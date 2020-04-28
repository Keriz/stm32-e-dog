#include "ch.h"
#include "hal.h"
#include <math.h>
#include <main.h>
#include <motors.h>
#include <process_move.h>
#include "audio.h"
#include <sensors/proximity.h>


#define MOTOR_SPEED   			338 // [step/s]
#define MOTOR_STOP  			0
#define IR_THRESOLD_COLLISION	600

#define NSTEP_ONE_TURN     		1000 // number of step for 1 turn of the motor
#define PI                  	3.1415926536f
#define WHEEL_PERIMETER     	13 // [cm]

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

/*
 * Moves forward with a given distance.
 * dist: distance in cm
 */
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

/* P regulator to adjust the speed without bumps
 * distance: actual position
 * goal:	 final goal position
 */
int16_t p_regulator(float distance, float goal){
	float error=0;
	float speed=0;
	error= distance - goal;

	speed = KP * error;

	return (int16_t)speed;
}

/*
 * stops the robot
 */
void motor_stop(void){
	right_motor_set_speed(MOTOR_STOP);
	left_motor_set_speed(MOTOR_STOP);
}


static THD_WORKING_AREA(waProcessMove, 256);
static THD_FUNCTION(ProcessMove, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int16_t speed;

    while(1){

    		if(is_voice_detected()){
    			float phase_delay_x= get_phase_delay_X()*180/PI;
    			float phase_delay_y= get_phase_delay_Y()*180/PI;
    			speed= p_regulator(phase_delay_x, GOAL_ANGLE);

    			if(phase_delay_x > TOLERANCE_DELAY_PHASE || phase_delay_x < -TOLERANCE_DELAY_PHASE ){ //if the sound is in front of the robot , it doens't move
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

/*
 * The robot moves forward towards the voice, and if it hits an obstacles it moves around it using the IR sensors.
 */
void move_and_escape(void){
	int ir_sensor7 = get_calibrated_prox(IR_SENSOR_7);
	int ir_sensor6 = get_calibrated_prox(IR_SENSOR_6);
	int ir_sensor1 = get_calibrated_prox(IR_SENSOR_1);
	int ir_sensor0 = get_calibrated_prox(IR_SENSOR_0);

	if( ir_sensor6 >= IR_THRESOLD_COLLISION || ir_sensor7 >= IR_THRESOLD_COLLISION){
	   while(ir_sensor6 >= IR_THRESOLD_COLLISION || ir_sensor7 >= IR_THRESOLD_COLLISION){
	    		ir_sensor7 = get_calibrated_prox(IR_SENSOR_7);
	    		ir_sensor6 = get_calibrated_prox(IR_SENSOR_6);
	    		turn_right(MOTOR_SPEED);
	    	}
	   go_forward_x_cm(10);
	}
	if( ir_sensor0 >= IR_THRESOLD_COLLISION || ir_sensor1 >= IR_THRESOLD_COLLISION){
	    while(ir_sensor0 >= IR_THRESOLD_COLLISION || ir_sensor1 >= IR_THRESOLD_COLLISION){
	    		ir_sensor1 = get_calibrated_prox(IR_SENSOR_1);
	    		ir_sensor0 = get_calibrated_prox(IR_SENSOR_0);
	    		turn_left(MOTOR_SPEED);
	    }
	    go_forward_x_cm(10);
	}
	go_forward();
}

void process_move_start(void){
	chThdCreateStatic(waProcessMove, sizeof(waProcessMove), NORMALPRIO, ProcessMove, NULL);
}
