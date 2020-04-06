#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define MOTOR_SPEED   338 // [step/s]
#define STOP  		0
#define COLLISION   600
#define DIRECTION_CHANGED   0


#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     13 // [cm]

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MIN_LINE_WIDTH			40
#define MAX_DISTANCE 			25.0f

#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2

#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera

#define KP						600.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;


void Send_value2(void) ;
void Send_value3(void);
void Done(void);
void Done2(void);

#ifdef __cplusplus
}
#endif

#endif
