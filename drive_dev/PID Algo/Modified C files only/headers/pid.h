/*
 * pid.h
 *
 *  Created on: Oct 17, 2022
 *      Author: Mohamed
 *      From: https://embeddedexpert.io/?p=960
 *      	  https://embeddedexpert.io/?p=968
 *      	  https://embeddedexpert.io/?p=1035
 *
 */


#ifndef PID_H_
#define PID_H_



#endif /* PID_H_ */

typedef struct
	{
	float Kp;
	float Ki;
	float Kd;
	float Ts;
	float Set_point;
	//float Anti_windup_error;
	//float Outmin;
	//float Outmax;
	//int Anti_windup;
	float errorThreshold;

	}PID_Param_t;

void PID_init(PID_Param_t *par);
float PID_Calculation(float input);

