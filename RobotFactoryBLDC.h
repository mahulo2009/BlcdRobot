#ifndef Robot_Factory_BLDC_H
#define Robot_Factory_BLDC_H

#include <RobotFactory.h>
#include <DifferentialWheeledRobot.h>
#include "RosConfigBLDC.h"
#include "BLCDHardwareController.h"
#include "WheelEncoder.h"

class RobotFactoryBLDC : public RobotFactory {

  	public:

            RobotFactoryBLDC(RosConfigBLDC  * robot_confing);

            virtual RobotBase * buildRobot();

            virtual WheelBase * buildWheel(int index);

	protected:

  	private:

            RosConfigBLDC * robot_confing;
};
#endif