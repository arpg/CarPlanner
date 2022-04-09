#include <CarPlanner/ROSCommander.h>

///////////////////////////////////////////////////////
ROSCommander::ROSCommander(std::string command_topic/*="command"*/)
{
    pub_ = nh_.advertise<carplanner_msgs::Command>(command_topic, 10);
}

/////////////////////////////////////////////////////////////
ROSCommander::~ROSCommander()
{
}

///////////////////////////////////////////////////////////////
void ROSCommander::SendCommand(ControlCommand command, bool sil/*=false*/)
{
    carplanner_msgs::Command command_msg = command.toROS();

    // sil doesn't matter for now but may in the near future
    if ( sil ) {
        //send Command to BulletCarModel
        pub_.publish(command_msg);
    }
    else {
        //send Command to NinjaCar
        pub_.publish(command_msg);
    }
}