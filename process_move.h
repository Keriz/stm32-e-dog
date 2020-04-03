#ifndef EXPLORE
#define EXPLORE

void counter_motor_step_init(int32_t counter_value);
void turn_x_degree(int degree);

void process_move_start(void);
void motor_stop(void);
void go_forward(void);

void motor_set_position(float position_r, float position_l);
void motor_one_turn(void);
#endif
