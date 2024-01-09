/*
 * pid.h
 *
 *  Created on: 23.09.2019
 *      Author: Mariusz
 */

#ifndef PID_H_
#define PID_H_

#include "main.h"
#include "math.h"

// KP 6.525  Ti 0.3,   TD (-0.03055f / 0.005f)

#define KP_MOTOR_1 3.845f
#define TI_MOTOR_1 0.1f
#define TD_MOTOR_1 -2.652f

#define KP_MOTOR_2 3.845f
#define TI_MOTOR_2 0.1f
#define TD_MOTOR_2 -2.652f

#define KP_MOTOR_3 3.845f
#define TI_MOTOR_3 0.1f
#define TD_MOTOR_3 -2.652f

#define KP_MOTOR_4 3.845f
#define TI_MOTOR_4 0.1f
#define TD_MOTOR_4 -2.652f



#define KP_ORIENTATION 0.5f
#define TI_ORIENTATION 0
#define TD_ORIENTATION -0



#define KP_MOTOR_5 5.0f
#define TI_MOTOR_5 0
#define TD_MOTOR_5 -0

#define KP_MOTOR_6 10.0f
#define TI_MOTOR_6 0
#define TD_MOTOR_6 -0

#define KP_MOTOR_7 15.0f
#define TI_MOTOR_7 0.005f
#define TD_MOTOR_7 -0.01f

/*
#define KP_MOTOR_7 0.01f
#define TI_MOTOR_7 0.005f
#define TD_MOTOR_7 -0.01f
*/

typedef struct  {

	double Kp, Ti, Td;

	double pwm;

	double set_speed;
	int measured_speed;

	double speed_error;
	double speed_error_sum;
	double speed_error_last;

	double derivative;


} PID;

void PID_Set_parameters(PID* pid, double Kp, double Ti, double Td);

double PID_Calculate(PID* pid);




#endif /* PID_H_ */
