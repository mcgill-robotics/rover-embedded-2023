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

float Kp,Ki,Kd,Ts,Outmin,Outmax,set_point,antiwinduperror;
int windup;
float error,prev_input,error_sum; //do they start as 0 at first?


void PID_init(PID_Param_t *par)
	{

	;
	Kp=par->Kp;
	Ki=par->Ki;
	Kd=par->Kd;
	Ts=par->Ts;
	set_point=par->Set_point;
	antiwinduperror=par->Anti_windup_error;
	Outmin=par->Outmin;
	Outmax=par->Outmax;
	windup=par->Anti_windup;

	if(0==par->Anti_windup_error){antiwinduperror=10;}
	}

float PID_Calculation(float inputSpeed)
	{
	error=(set_point-inputSpeed);
	error_sum+=error; //added here

	float outputSpeed;
	if(Anti_windup_enabled==windup){ //maybe have another look at that

		if(antiwinduperror<abs(error)) {
			outputSpeed=Kp*(error)-( Kd*(input-prev_input)/Ts);
			// used to be outputSpeed=Kp*(error)+Kd*(inputSpeed-prev_input)/Ts;
			}
		else {
			outputSpeed=(Kp*(error)) +(Ki*(error_sum)*Ts) -( Kd*(input-prev_input)/Ts);
			//used to be outputSpeed=(Kp*(error)) +( Ki*(Ki_sum)*Ts) -( Kd*(input-prev_input)/Ts);
			}

		}

	else {
		outputSpeed=Kp*(error) + Ki*(error_sum)*Ts -( Kd*(input-prev_input)/Ts);
		// used to be outputSpeed=Kp*(error) + Ki*(Ki_sum)*Ts - Kd*(input-prev_input)/Ts;
		}

		//used to have Ki_sum=Ki_sum+(Ki_sum); //what's this for?
	    Ki_sum=Ki_sum+(Ki_sum);
		if(outputSpeed>Outmax){outputSpeed=Outmax;}
		if(outputSpeed<Outmin){outputSpeed=Outmin;}
		prev_input=inputSpeed;
		return outputSpeed;

	}
