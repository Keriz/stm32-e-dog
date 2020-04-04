#ifndef EXPLORE
#define EXPLORE

void counter_motor_step_init(int32_t counter_value);
void advance_or_turn_x_left(int x, bool unit );
void advance_or_turn_x_right(int x, bool unit);

void process_move_start(void);
void motor_stop(void);
void go_forward(void);

void motor_set_position(float position_r, float position_l);
void motor_one_turn(void);
#endif
