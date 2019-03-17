#include "BLCDHardwareController.h"

BLCDHardwareController::BLCDHardwareController(double max_speed,int min_duty, int max_duty,int can_id,int invert) : 
    max_speed_(max_speed), 
    min_duty_(min_duty), 
    max_duty_(max_duty),
    can_id_(can_id),
    invert_(invert)
{
    wheel_direction_ = FORWARD;
}

void BLCDHardwareController::setupDirection(Wheel_Direction direction) 
{
  switch(direction) 
  {
    case FORWARD:

        wheel_direction_ = FORWARD;
        break;

    case BACKWARD:

		wheel_direction_ = BACKWARD;
        break;
   }   
}

void BLCDHardwareController::velocity(double velocity)
{
    if (velocity < 0) 
    {
		setupDirection(BACKWARD);	        
	} 
    else 
    {
  		setupDirection(FORWARD);
	}

    #ifdef BLCD_HARDWARE_CONTROLLER_DEBUG
    Serial.print("velocity:");
    Serial.println(velocity);
    #endif
    
    //Convert radiand per second to rpm
    float rpm = abs(velocity) * (60/(2*PI)) * 15;
    power(rpm);
}

void BLCDHardwareController::power(double rpm)
{
    if (rpm > 1250) 
        rpm = 1250; //TODO DEFINE THIS IN CONFIGURATION FILE.
    if (rpm < 900) 
        rpm = 0;

    #ifdef BLCD_HARDWARE_CONTROLLER_DEBUG
    Serial.print("bldc_interface_set_forward_can:");
    Serial.println(can_id_);
    #endif
  
    bldc_interface_set_forward_can(can_id_);

    if ( wheel_direction_ == FORWARD )
    {
        #ifdef BLCD_HARDWARE_CONTROLLER_DEBUG
        Serial.print("bldc_interface_set_duty_cycle:");
        Serial.println(rpm * invert_);
        #endif
        bldc_interface_set_rpm( rpm * invert_);
    }
    else 
    {
        #ifdef BLCD_HARDWARE_CONTROLLER_DEBUG
        Serial.print("bldc_interface_set_duty_cycle:");
        Serial.println(-rpm * invert_);
        #endif
        bldc_interface_set_rpm(-rpm * invert_);
    }
}