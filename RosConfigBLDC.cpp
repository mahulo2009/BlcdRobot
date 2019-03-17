#include <RosConfigBLDC.h>



RosConfigBLDC::RosConfigBLDC()
{

}

void RosConfigBLDC::read(ros::NodeHandle &nh)
{
    //TODO FIND OUT HOW TO USE THIS FROM CONFIG
    /*
    nh.getParam("/gara/robot_wheel_separation", &this->robot_wheel_separation);
    nh.getParam("/gara/robot_wheel_radious", &this->robot_wheel_radious);      
    nh.getParam("/gara/max_speed", &this->max_speed);
    nh.getParam("/gara/min_duty", &this->min_duty);  
    nh.getParam("/gara/max_duty", &this->max_duty);
    nh.getParam("/gara/can_id_left", &this->wheel_config[0].can_id);
    nh.getParam("/gara/invert_left", &this->wheel_config[0].invert);
    nh.getParam("/gara/can_id_right", &this->wheel_config[1].can_id);
    nh.getParam("/gara/invert_right", &this->wheel_config[1].invert);     
    */
   
    this->robot_wheel_separation = 0.585;
    this->robot_wheel_radious = 0.125;
    this->max_speed = 10;
    this->min_duty = 0.0;
    this->max_duty = 1.0;
    this->wheel_config[0].can_id = -1;   //Wheel left
    this->wheel_config[0].invert = -1;

    this->wheel_config[1].can_id = 2;  //Wheel right
    this->wheel_config[1].invert = 1;   
}

