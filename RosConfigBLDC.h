#ifndef Ros_Config_BLDC_H
#define Ros_Config_BLDC_H

#include <RosConfigBase.h>

struct wheel_config_bldc_t {
    int can_id;
    int invert;
};

class RosConfigBLDC : public RosConfigBase {

  	public:

            RosConfigBLDC();

            virtual void read(ros::NodeHandle &nh);

            float robot_wheel_separation;
            float robot_wheel_radious;
            float max_speed;
            float min_duty;
            float max_duty;

            wheel_config_bldc_t wheel_config[2];
            
	protected:

  	private:
                
};
#endif