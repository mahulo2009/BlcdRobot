#ifndef Wheel_BLCD_H
#define Wheel_BLCD_H

#include "Arduino.h"
#include <WheelBase.h>
#include <Pid.h>
#include <TaskSchedulerDeclarations.h>
#include <bldc_interface_uart.h>
#include <bldc_interface.h>


//#define WHEEL_BLCD 1

class WheelBLCD : public WheelBase {

  public:

    	WheelBLCD();												//default constructor.  	 	
		
    	virtual void move(double velocity);                          //velocity demanded radians per second.
    	virtual void stop();										//reset duty to 0 and direction to forward

		void setCurrentVelocity(double currentVelocity) 
		{
			this->currentVelocity_ = currentVelocity;
		}

		void update();
			
	private:

    	double period_pid_controller_;								//Period of the periodic task, Pid controller

		Pid * pid_;													//Pid

        

};
#endif