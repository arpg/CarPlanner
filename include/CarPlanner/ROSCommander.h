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

#include <HAL/Messages.pb.h>
#include <HAL/Messages/Pose.h>
#include <HAL/Messages/Matrix.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <google/protobuf/message.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

class ROSCommander
{
public:
    ROSCommander();
    ~ROSCommander();
    void Start();
    void Stop();
    void SendCommand(ControlCommand command, bool sil=false);

private:
    bool m_bIsStarted;
    boost::thread* m_pThread;

    // UDP values
    unsigned m_MochaPort; // UDP port for this gui
    unsigned m_ComPort; // UDP port for m_bSIL (unused)
    unsigned m_CarPort; // UDP port of the ninja car
    struct sockaddr_in mochAddr; // Address of this gui (for sending commands from)
    struct sockaddr_in comAddr;  // Address for m_bSIL (currently not running 1/16/17)
    struct sockaddr_in carAddr;  // Address of the car (for sending commands to)
    socklen_t addrLen = sizeof(mochAddr);
    int sockFD;
};

#endif // ROS_COMMANDER_H