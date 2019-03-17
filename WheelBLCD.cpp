#include "WheelBLCD.h"

WheelBLCD::WheelBLCD() : pid_(0)
{
}

void WheelBLCD::move(double velocity)
{
    this->targetVelocity_=velocity;

	if (pid_ != 0) 
	{
		pid_->setTarget(this->targetVelocity_);
	}
}

void WheelBLCD::stop()
{
    this->targetVelocity_=0;

	if (pid_ != 0) 
	{
		pid_->setTarget(this->targetVelocity_);
	}  
}

void WheelBLCD::update()
{

	if (pid_ != 0) 
	{
		this->demandedVelocity_= 
			pid_->update(this->currentVelocity_,this->period_pid_controller_); 
	} 
	else 
	{
		this->demandedVelocity_ = targetVelocity_;
	}
	  
  	this->controller_->velocity(demandedVelocity_);

	#ifdef WHEEL_BLCD
  	Serial.print("WheelBLCD::update_:");
	Serial.print("\t");
  	Serial.print(targetVelocity_);
  	Serial.print("\t");
  	Serial.print(currentVelocity_);
  	Serial.print("\t");
  	Serial.print(this->demandedVelocity_);
  	Serial.print("\n");
  	#endif	  
}