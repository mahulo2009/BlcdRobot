#ifndef BLCD_HardwareController_H
#define BLCD_HardwareController_H

#include "HardwareController.h"
#include "Arduino.h"
#include "bldc_interface.h"

//#define BLCD_HARDWARE_CONTROLLER_DEBUG 1

class BLCDHardwareController : public HardwareController {

    public:

        BLCDHardwareController(double max_speed,int min_duty, int max_duty,int can_id,int invert);
        
        virtual void velocity(double velocity);

        int getCANId() 
        {
            return can_id_;
        }

        int invert_;
    
    protected:
    
        virtual void setupDirection(Wheel_Direction direction);       
        virtual void power(double duty);


    private:

        float max_speed_;			    //maximun speed of the motor in radians per second.
        int min_duty_;				    //minimum duty
        int max_duty_;				    //maximun duty
        int can_id_;

        Wheel_Direction wheel_direction_;       
};
#endif