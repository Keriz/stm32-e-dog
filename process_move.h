#ifndef EXPLORE
#define EXPLORE

#define TOO_LONG -50;

void moving_by_escaping(void);

void turn_right_PD(int speed);
void turn_left_PD(int speed);

float get_dephase_mean(void);

void counter_motor_step_init(int32_t counter_value);
void advance_or_turn_x_left(float x, bool unit );
void advance_or_turn_x_right(float x, bool unit);
void advance_or_turn_x_right_temp(int x, bool unit);

int16_t pi_regulator(float distance, float goal);

void process_move_start(void);
void motor_stop(void);
void go_forward(void);

void motor_set_position(float x, bool unit );
void motor_one_turn(void);
#endif
