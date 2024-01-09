/*
 * pid.c
 *
 *  Created on: 23.09.2019
 *      Author: Mariusz
 */


#include "pid.h"

void PID_Set_parameters(PID* pid, double Kp, double Ti, double Td){

	pid->Kp = Kp;
	pid->Ti = Ti;
	pid->Td = Td;

	pid->pwm = 0;

	pid->set_speed = 0;
	pid->measured_speed = 0;

	pid->speed_error = 0;
	pid->speed_error_sum = 0;
	pid->speed_error_last = 0;

}

double PID_Calculate(PID* pid){

	pid->speed_error = pid->set_speed - pid->measured_speed;
	pid->speed_error_sum += pid->speed_error;

	pid->derivative = pid->speed_error - pid->speed_error_last;


	pid->pwm = (pid->Kp* pid->speed_error + pid->Ti* pid->speed_error_sum + pid->Td* pid->derivative);

	pid->speed_error_last = pid->speed_error;



	return pid->pwm ;
}


