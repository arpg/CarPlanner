#ifndef HAL_COMMANDER_H
#define	HAL_COMMANDER_H

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

class HALCommandReceiver
{
public:
    HALCommandReceiver();
    ~HALCommandReceiver();
    void ReceiveCommand(ControlCommand &command, bool &bNoDelay, bool &bNoUpdate);

private:
    // UDP values
    unsigned m_ComPort; // UDP port for m_bSIL (unused)
    unsigned m_MochaPort; // UDP port for this gui
    struct sockaddr_in comAddr;  // Address for m_bSIL (currently not running 1/16/17)
    struct sockaddr_in mochAddr; // Address of this gui (for sending commands from)
    socklen_t addrLen = sizeof(mochAddr);
    int comSockFD;
    int mochSockFD;
};

#endif // HAL_COMMANDER_H