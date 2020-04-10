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

static int16_t position_to_reach_right = 0;	    // in [step]
static int16_t position_to_reach_left = 0;	    // in [step]
static int16_t counter_step_right = 0;

#define TOLERANCE_DELAY_PHASE 7
#define UNIT_ANGLE 0
#define UNIT_DISTANCE 1

void go_forward(void){
	right_motor_set_speed(MOTOR_SPEED);
	left_motor_set_speed(MOTOR_SPEED);
}

/*
 * int speed: should be between 0 and MOTOR_SPEED
 * float degree: if specified the robot turns until it reached the given angle
 */
void turn_right(int16_t speed, float degree){
	if (degree == 0){
		right_motor_set_speed(-speed);
		left_motor_set_speed(speed);
	}
	else
	{
		counter_motor_step_init(0);
		motor_set_position(degree,UNIT_ANGLE);
		while(abs(position_to_reach_right) > abs(counter_step_right)){
				counter_step_right= right_motor_get_pos();
				right_motor_set_speed(-speed);
				left_motor_set_speed(speed);
			}
	}
}

/*
 * int speed: should be between 0 and MOTOR_SPEED
 * float degree: if specified the robot turns until it reached the given angle
 */
void turn_left(int16_t speed, float degree){
	if (degree == 0){
		right_motor_set_speed(speed);
		left_motor_set_speed(-speed);
	}
	else
	{
		counter_motor_step_init(0);
		motor_set_position(degree,UNIT_ANGLE);
		while(abs(position_to_reach_right) > abs(counter_step_right)){
				counter_step_right= right_motor_get_pos();
				right_motor_set_speed(speed);
				left_motor_set_speed(-speed);
			}
	}
}

void counter_motor_step_init(int32_t counter_value){
	left_motor_set_pos(counter_value);
	right_motor_set_pos(counter_value);
}

void motor_set_position(float x, bool unit )
{
	float dist = x; //useless?
	if(unit == UNIT_ANGLE){ //if x in degree convert to dist
		dist=  x* PERIMETER_EPUCK /(360);
	}

	position_to_reach_left = dist * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = dist * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

int16_t pi_regulator(float distance, float goal){
	float error=0;
	float speed=0;

	static float sum_error =0;
	error= distance - goal;

	if(fabs(error) < ERROR_THRESHOLD){
			return 0;
		}
	sum_error += error;
	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}
	speed = KP * error + KI * sum_error;

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
    		if(move_or_not()){
    			go_forward();
    			chprintf((BaseSequentialStream *)&SD3,"wesh\n");
    		}

    	//	messagebus_topic_wait(phase_topic, &phase_value, sizeof(phase_value));
    		float dephase_rad_x= get_phase_delay_X();
    		//if no sound don't move
			if(dephase_rad_x != NOT_FOUND){


				//we are in the case where we found sound
				float dephase_x= dephase_rad_x*180/PI;
				//chprintf((BaseSequentialStream *)&SD3,"before_turn_dephasage=%f\n",dephase_x);
				if(dephase_x > TOLERANCE_DELAY_PHASE || dephase_x < -TOLERANCE_DELAY_PHASE ){ //if the siynd in front of the robot do nothing
					if(dephase_x < 0){
						//chprintf((BaseSequentialStream *)&SD3,"turn_left_dephasage=%f\n",dephase_x);
						//while(dephase_x < 7 || dephase_x > -7 ){
						//	dephase_x= get_dephasage_x()*180/PI;
						//	dephase_rad_x= get_dephasage_x();
						//	if(dephase_rad_x== NOT_FOUND)
						//		break;
						//	speed= pi_regulator(dephase_x,GOAL_ANGLE);
							//turn_left(abs(speed), 0);
						//}
					}
					if(dephase_x > 0){
						//chprintf((BaseSequentialStream *)&SD3,"turn_right_dephasage=%f\n",dephase_x);
						//while(dephase_x < 5 || dephase_x > -5 ){
						//	dephase_x= get_dephasage_x()*180/PI;
						//	dephase_rad_x= get_dephasage_x();
						//	if(dephase_rad_x== NOT_FOUND)
						//		break;
						//	speed= pi_regulator(dephase_x,GOAL_ANGLE);
							//turn_right(abs(speed), 0);
						//}
					}
				}
				else{
					//chprintf((BaseSequentialStream *)&SD3,"go_forward=%f\n",dephase_rad_x*180/PI);
					//go_forward();
					move_and_escape();
				}

			}
			else if(dephase_rad_x == NOT_FOUND){
				//chprintf((BaseSequentialStream *)&SD3,"stopping\n");
				motor_stop();
			}
        //chThdSleepUntilWindowed(time, time + MS2ST(10));
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
	    		turn_right(MOTOR_SPEED,0);
	    	}
	   turn_right(MOTOR_SPEED,10);
	}
	if( ir_sensor0 >= COLLISION || ir_sensor1 >= COLLISION){
	    while(ir_sensor0 >= COLLISION || ir_sensor1 >= COLLISION){
	    		ir_sensor1 = get_calibrated_prox(1);
	    		ir_sensor0 = get_calibrated_prox(0);
	    		turn_left(MOTOR_SPEED,0);
	    }
	    turn_left(MOTOR_SPEED,10);
	}
	go_forward();
}

void process_move_start(void){
	chThdCreateStatic(waProcessMove, sizeof(waProcessMove), NORMALPRIO+1, ProcessMove, NULL);
}
