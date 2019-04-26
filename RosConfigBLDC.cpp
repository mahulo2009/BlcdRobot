#include <RosConfigBLDC.h>



RosConfigBLDC::RosConfigBLDC()
{

}

void RosConfigBLDC::read(ros::NodeHandle &nh)
{    

    nh.getParam("/gara/robot_wheel_separation", &this->robot_wheel_separation);
    nh.getParam("/gara/robot_wheel_radious", &this->robot_wheel_radious);      
    nh.getParam("/gara/max_speed", &this->max_speed);
    nh.getParam("/gara/min_duty", &this->min_duty);  
    nh.getParam("/gara/max_duty", &this->max_duty);
    nh.getParam("/gara/can_id_left", &this->wheel_config[0].can_id);
    nh.getParam("/gara/invert_left", &this->wheel_config[0].invert);
    nh.getParam("/gara/can_id_right", &this->wheel_config[1].can_id);
    nh.getParam("/gara/invert_right", &this->wheel_config[1].invert);     
    
}

