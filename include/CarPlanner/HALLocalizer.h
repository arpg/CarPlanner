/*
 * File:   Localizer.h
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#ifndef HAL_LOCALIZER_H
#define	HAL_LOCALIZER_H


#include <CarPlanner/Localizer.h>

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

class HALLocalizer : Localizer
{
public:
    HALLocalizer();
    virtual ~HALLocalizer();
    void TrackObject(const std::string& sObjectName, bool bRobotFrame = true);
    void TrackObject(const std::string& sObjectName, Sophus::SE3d dToffset, bool bRobotFrame = true);
    void Start();
    void Stop();
    Sophus::SE3d GetPose(const std::string& sObjectName , bool blocking = false, double *time = NULL, double *rate = NULL);
    //Eigen::Matrix<double,6,1> GetdPose( const std::string& sObjectName );
    eLocType WhereAmI( Eigen::Vector3d P );
    eLocType WhereAmI( Eigen::Vector6d P );

private:
    Sophus::SE3d LookupPose(std::string objectName);
    double LookupTime();

    // UDP values
    unsigned m_CarPort;
    unsigned m_LocPort;
    struct sockaddr_in locAddr;
    struct sockaddr_in carAddr;
    socklen_t addrLen = sizeof(locAddr);
    int recvLen;
    int sockFD;
    unsigned char buf[2048];
    unsigned int msgSize = 0;
    hal::PoseMsg posys;
};

#endif	/* HAL_LOCALIZER_H */
