#include "BLCDHardwareController.h"

void handle_interrupt_1(mc_values *val)
{
	BLCDHardwareController::instance_[0]->handler_(val);
}

void handle_interrupt_2(mc_values *val)
{
	BLCDHardwareController::instance_[1]->handler_(val);
}

void handle_interrupt_3(mc_values *val)
{
	BLCDHardwareController::instance_[2]->handler_(val);
}

void handle_interrupt_4(mc_values *val)
{
	BLCDHardwareController::instance_[3]->handler_(val);
}

BLCDHardwareController *BLCDHardwareController::instance_[MAX_BLCD_HARDWARE_CONTROLLER];
int BLCDHardwareController::controller_index_=0;


void(*handler_function[MAX_BLCD_HARDWARE_CONTROLLER])(mc_values *val)={handle_interrupt_1,
											handle_interrupt_2,
											handle_interrupt_3,
											handle_interrupt_4};

BLCDHardwareController::BLCDHardwareController(int index,double max_speed,int min_duty, int max_duty,int can_id,int invert) : 
    index_(index), 
    max_speed_(max_speed), 
    min_duty_(min_duty), 
    max_duty_(max_duty),
    can_id_(can_id),
    invert_(invert),
    current_velocity_(0)
{
    wheel_direction_ = FORWARD;

    if (BLCDHardwareController::controller_index_ < MAX_BLCD_HARDWARE_CONTROLLER) 
    {
        instance_[BLCDHardwareController::controller_index_++]=this;
    }

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
    //float rpm = abs(velocity) * (60/(2*PI)) * 15;
    float rpm = abs(velocity) * 0.015;
    power(rpm);
}

void BLCDHardwareController::power(double rpm)
{
    if (rpm > 0.2) 
        rpm = 0.2; //TODO DEFINE THIS IN CONFIGURATION FILE.
    if (rpm < 0.05) 
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
        //bldc_interface_set_rpm( rpm * invert_);
        bldc_interface_set_duty_cycle( rpm * invert_);
    }
    else 
    {
        #ifdef BLCD_HARDWARE_CONTROLLER_DEBUG
        Serial.print("bldc_interface_set_duty_cycle:");
        Serial.println(-rpm * invert_);
        #endif
        //bldc_interface_set_rpm(-rpm * invert_);
        bldc_interface_set_duty_cycle( -rpm * invert_);
    }
}

void BLCDHardwareController::buffer_clean()
{
    bldc_interface_uart_run_timer();
}

void BLCDHardwareController::init()
{
    comm_uart_init();
}

void BLCDHardwareController::buffer_read()
{    
    while (Serial1.available()) 
    {        
	    bldc_interface_uart_process_byte(Serial1.read());
	}
}


void BLCDHardwareController::update(double dt) 
{
    bldc_interface_set_rx_value_func(handler_function[index_]);
    bldc_interface_set_forward_can(can_id_);
    bldc_interface_get_values();   
}

void BLCDHardwareController::handler_(mc_values *val)
{
    this->current_velocity_ =  ( (val->rpm/15.0) * (2*PI)/60 * invert_ );  //radians per second	

    #ifdef BLCD_HARDWARE_CONTROLLER_DEBUG
    Serial.print("current_velocity_:");
    Serial.println(current_velocity_);
    #endif
}
