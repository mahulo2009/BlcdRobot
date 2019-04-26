#ifndef BLCD_HardwareController_H
#define BLCD_HardwareController_H

#include "HardwareController.h"
#include "Arduino.h"
#include "bldc_interface.h"
#include <bldc_interface_uart.h>
#include <comm_uart.h>

//#define BLCD_HARDWARE_CONTROLLER_DEBUG 1

#define MAX_BLCD_HARDWARE_CONTROLLER 4

class BLCDHardwareController : public HardwareController {

    public:

        static BLCDHardwareController *instance_[MAX_BLCD_HARDWARE_CONTROLLER];
	    static int controller_index_;

        BLCDHardwareController(int index, double max_speed,int min_duty, int max_duty,int can_id,int invert);
        
        virtual void    velocity(double velocity);
        virtual void    update(double dt);
        virtual double  getVelocity(double dt) {
           return current_velocity_;
        }

        static void     init();
        static void     buffer_clean();
        static void     buffer_read();
            
    protected:
    
        virtual void    setupDirection(Wheel_Direction direction);       
        virtual void    power(double duty);

    private:

        friend void     handle_interrupt_1(mc_values *val);
        friend void     handle_interrupt_2(mc_values *val);
        friend void     handle_interrupt_3(mc_values *val);
        friend void     handle_interrupt_4(mc_values *val);

        void            handler_(mc_values *val);

        int             index_;
        float           max_speed_;			    //maximun speed of the motor in radians per second.
        int             min_duty_;				    //minimum duty
        int             max_duty_;				    //maximun duty
        int             can_id_;
        int             invert_;
        double          current_velocity_;

        Wheel_Direction wheel_direction_;       
};
#endif