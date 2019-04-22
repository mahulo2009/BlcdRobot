#include "RobotFactoryBLDC.h"

RobotFactoryBLDC::RobotFactoryBLDC(RosConfigBLDC  * robot_confing) 
{
     this->robot_confing = robot_confing;
}

RobotBase * RobotFactoryBLDC::buildRobot()
{
    RobotBase * robot = 
        new DifferentialWheeledRobot(robot_confing->robot_wheel_separation,robot_confing->robot_wheel_radious);
    return robot;
}

WheelBase * RobotFactoryBLDC::buildWheel(int index)
{
    //Controller
    BLCDHardwareController * controller = 
            new BLCDHardwareController( index,
                                        robot_confing->max_speed,
                                        robot_confing->min_duty,
                                        robot_confing->max_duty,
                                        robot_confing->wheel_config[index].can_id,
                                        robot_confing->wheel_config[index].invert);

    //Wheel
    WheelBase * wheel = new WheelEncoder();
    wheel->attachController(controller);

    return wheel;
}
