#ifndef PROCESS_MOVE_H
#define PROCESS_MOVE_H

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	IR_SENSOR_0 = 0,
	IR_SENSOR_1,
	IR_SENSOR_2,
	IR_SENSOR_3,
	//Arrays containing the computed magnitude of the complex numbers
	IR_SENSOR_4,
	IR_SENSOR_5,
	IR_SENSOR_6,
	IR_SENSOR_7
} IR_Sensors_t;

void move_and_escape(void);

void turn_right(int16_t speed);
void turn_left(int16_t speed);

int16_t p_regulator(float distance, float goal);

void process_move_start(void);
void motor_stop(void);
void go_forward(void);
void go_forward_x_cm(float dist);

#endif
