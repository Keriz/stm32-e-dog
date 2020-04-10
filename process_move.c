#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <process_move.h>
#include <process_audio.h>
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

void turn_right_PD(int speed){
	right_motor_set_speed(-speed);
	left_motor_set_speed(speed);
}

void turn_left_PD(int speed){
	right_motor_set_speed(speed);
	left_motor_set_speed(-speed);
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
	position_to_reach_left = x * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = x * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

void advance_or_turn_x_left(float x, bool unit){
	counter_motor_step_init(0);
	motor_set_position(x,unit);
	while(abs(position_to_reach_left) > abs(counter_step_left)){
		counter_step_left= left_motor_get_pos();
		if(unit)
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

void advance_x(float x, bool unit){
	counter_motor_step_init(0);
	motor_set_position(x,unit);
	while(abs(position_to_reach_right) > abs(counter_step_right)){
		counter_step_right= right_motor_get_pos();
		go_forward();
	}
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


static THD_WORKING_AREA(waProcessmove, 256);
static THD_FUNCTION(Processmove, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //systime_t time;
    int speed;

    while(1){
    		if(move_or_not()){
    			go_forward();
    			chprintf((BaseSequentialStream *)&SD3,"wesh\n");
    		}


    		float dephase_rad_x= get_dephasage_x();
    		//if no sound don't move
		if(dephase_rad_x != NOT_FOUND){
			//we are in the case where we found sound
			float dephase_x= dephase_rad_x*180/PI;
			//chprintf((BaseSequentialStream *)&SD3,"before_turn_dephasage=%f\n",dephase_x);
			if(dephase_x > 7 || dephase_x < -7 ){ //if the siynd in front of the robot do nothing
				if(dephase_x < 0){
					//chprintf((BaseSequentialStream *)&SD3,"turn_left_dephasage=%f\n",dephase_x);
					//while(dephase_x < 7 || dephase_x > -7 ){
					//	dephase_x= get_dephasage_x()*180/PI;
					//	dephase_rad_x= get_dephasage_x();
					//	if(dephase_rad_x== NOT_FOUND)
					//		break;
					//	speed= pi_regulator(dephase_x,GOAL_ANGLE);
						//turn_left_PD(abs(speed));
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
						//turn_right_PD(abs(speed));
					//}
				}
			}
			else{
				//chprintf((BaseSequentialStream *)&SD3,"go_forward=%f\n",dephase_rad_x*180/PI);
				//go_forward();
				//advance_x(10,false);
				//moving_by_escaping();
			}

		}
		else if(dephase_rad_x == NOT_FOUND){
			//chprintf((BaseSequentialStream *)&SD3,"stopping\n");
			motor_stop();
		}
        //chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void moving_by_escaping(void){ //add after
	int ir_sensor7 = get_calibrated_prox(7);
	int ir_sensor6 = get_calibrated_prox(6);
	int ir_sensor1 = get_calibrated_prox(1);
	int ir_sensor0 = get_calibrated_prox(0);

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
}

void process_move_start(void){
	chThdCreateStatic(waProcessmove, sizeof(waProcessmove), NORMALPRIO+1, Processmove, NULL);
}
