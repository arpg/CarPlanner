#include <CarPlanner/ROSCommandReceiver.h>

///////////////////////////////////////////////////////
ROSCommandReceiver::ROSCommandReceiver(std::string command_topic/*="command"*/)
{
    ros::Subscriber sub = nh_.subscribe<carplanner_msgs::Command>(command_topic, 5, std::bind(&ROSCommandReceiver::CommandCallback, this, std::placeholders::_1));
}

/////////////////////////////////////////////////////////////
ROSCommandReceiver::~ROSCommandReceiver()
{
}

/////////////////////////////////////////////////////////////
void ROSCommandReceiver::CommandCallback(const carplanner_msgs::Command::ConstPtr& cmd_msg)
{
    cmd_msg_ = *cmd_msg;
}

/////////////////////////////////////////////////////////////
void ROSCommandReceiver::ReceiveCommand(ControlCommand &command, bool &bNoDelay, bool &bNoUpdate)
{
    command.fromROS(cmd_msg_);
    bNoDelay = cmd_msg_.no_delay;
    bNoUpdate = cmd_msg_.no_update;
}