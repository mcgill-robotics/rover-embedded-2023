/*
 * pid.c
 *
 *  Created on: Oct 17, 2022
 *      Author: Mohamed
 *      From: https://embeddedexpert.io/?p=960
 *      	  https://embeddedexpert.io/?p=968
 *      	  https://embeddedexpert.io/?p=1035
 *
 */


#include "pid.h"
#include "math.h"

float Kp,Ki,Kd,Ts,Outmin,Outmax,set_point,errorThreshold;
float error,prev_input,error_sum, dError; //do they start as 0 at first?


void PID_init(PID_Param_t *par)
	{

	;
	Kp=par->Kp;
	Ki=par->Ki;
	Kd=par->Kd;
	Ts=par->Ts;
	set_point=par->Set_point;
//	antiwinduperror=par->Anti_windup_error;
//	Outmin=par->Outmin;
//	Outmax=par->Outmax;
	errorThreshold=par->errorThreshold;

float PID_Calculation(float inputVar)
	{
	error=(set_point-inputVar);
	error_sum+=error; //added here
	//dError = (abs(error)-abs(prev_error))/Ts;
	dError = fabs(error-prev_error);
	propTerm = Kp*(error);
	intgrlTerm = Ki*(error_sum); //maybe times Ts
	derivTerm = Kd*dError;

	float outputVar;
		if(fabs(error)>fabs(errorThreshold)) { //I'm not sure why this is giving a warning, use abs or fabs
			outputVar = propTerm + derivTerm; //Do PD control when the error is too high
			}
		else {
			outputVar = propTerm + intgrlTerm + derivTerm; //Within a certain thereshold do PID
			}

		}

		// if(outputVar>Outmax){outputVar=Outmax;}
		// if(outputVar<Outmin){outputVar=Outmin;}
		prev_error=error;
		return outputVar;

	}
