#ifndef ROS_COMMANDER_H
#define	ROS_COMMANDER_H

#include <Eigen/Eigen>
#include <eigen3/Eigen/src/Core/products/GeneralBlockPanelKernel.h>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <vector>
#include <string.h>
#include <unistd.h>
#include "CarPlannerCommon.h"
#include "BulletCarModel.h"

#include <ros/ros.h>
#include <carplanner_msgs/Command.h>

class ROSCommander
{
public:
    ROSCommander(std::string command_topic="command");
    ~ROSCommander();
    void SendCommand(ControlCommand command, bool sil=false);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

#endif // ROS_COMMANDER_H