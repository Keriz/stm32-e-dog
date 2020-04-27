#ifndef EXPLORE
#define EXPLORE


void move_and_escape(void);

void turn_right(int16_t speed);
void turn_left(int16_t speed);

int16_t pi_regulator(float distance, float goal);

void process_move_start(void);
void motor_stop(void);
void go_forward(void);
void go_forward_x_cm(float dist);

#endif
