#ifndef EXPLORE
#define EXPLORE

void process_move_start(void);
void motor_stop(void);
void go_forward(void);

void motor_set_position(float position_r, float position_l);
void motor_one_turn(void);
#endif
