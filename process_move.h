#ifndef EXPLORE
#define EXPLORE

#define TOO_LONG -50;

void move_and_escape(void);

void turn_right(int16_t speed, float angle);
void turn_left(int16_t speed, float angle);

void counter_motor_step_init(int32_t counter_value);

int16_t pi_regulator(float distance, float goal);

void process_move_start(void);
void motor_stop(void);
void go_forward(void);

void motor_set_position(float x, bool unit );
#endif
