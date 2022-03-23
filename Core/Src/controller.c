/*
 * controller.c
 *
 *  Created on: 2022. 2. 17.
 *      Author: LEE
 */

#include "main.h"
#include "controller.h"

extern int16_t duty1;
extern int16_t duty3;
extern int16_t duty4;

CONTROLLER_SIGNAL Control()
{
	if(duty1 >= 550 && duty1 <= 950)
	{
		if(duty3 >= 550 && duty3 <= 950)
		{
			if(duty4 < 550) return FORWARD;  //FR:정, FL:정, RR:정, RL:정
			else if(duty4 > 950) return BACKWARD;  //FR:역, FL:역, RR:역, RL:역
			else return STOP;
		}

		else
		{
			if(duty4 >= 550 && duty4 <= 950)
			{
				if(duty3 < 550) return RIGHT;  //FR:역, FL:정, RR:정, RL:역
				else if(duty3 > 950) return LEFT;  //FR:정, FL:역, RR:역, RL:정
			}

			else if(duty4 < 550)
			{
				if(duty3 < 550) return FR45;  //FR:-, FL:정, RR:정, RL:-
				else if(duty3 > 950) return FL45;  //FR:정, FL:-, RR:-, RL:정
			}

			else if(duty4 > 950)
			{
				if(duty3 < 550) return BR45;  //FR:-, FL:정, RR:정, RL:-
				else if(duty3 > 950) return BL45;  //FR:정, FL:-, RR:-, RL:정
			}
		}
	}
	else
	{
		if(duty1 < 550) return CW;  //FR:역, FL:정, RR:역, RL:정
		else if(duty1 > 950) return CCW;  //FR:정, FL:역, RR:정, RL:역
	}
}


