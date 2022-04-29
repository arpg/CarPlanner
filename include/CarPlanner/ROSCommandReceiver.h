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

class ROSCommandReceiver
{
public:
    ROSCommandReceiver(std::string command_topic="command");
    ~ROSCommandReceiver();
    void CommandCallback(const carplanner_msgs::Command::ConstPtr &);
    void ReceiveCommand(ControlCommand &command, bool &, bool &);

private:
    ros::NodeHandle nh_;
    carplanner_msgs::Command cmd_msg_;
};

#endif // ROS_COMMANDER_H