/*
 * File:   Localizer.h
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#ifndef LOCALIZER_H
#define	LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Eigen>
#include <eigen3/Eigen/src/Core/products/GeneralBlockPanelKernel.h>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

// #include <HAL/Messages.pb.h>
// #include <HAL/Messages/Pose.h>
// #include <HAL/Messages/Matrix.h>
#include <vector>
#include <string.h>
#include <unistd.h>
#include "CarPlannerCommon.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <google/protobuf/message.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>


//struct MochaEntity
//{
//    std::string m_Name;
//    Eigen::Vector6d m_Pose;
//    Eigen::Vector6d m_DPose;
//};

class Localizer
{
    public:
        Localizer();
        virtual ~Localizer();
        void TrackObject(const std::string& sObjectName, const std::string& sHost , bool bRobotFrame = true);
        void TrackObject(const std::string& sObjectName, const std::string& sHost , Sophus::SE3d dToffset, bool bRobotFrame = true);
        void Start();
        void Stop();
        Sophus::SE3d GetPose(const std::string& sObjectName , bool blocking = false, double *time = NULL, double *rate = NULL);
        //Eigen::Matrix<double,6,1> GetdPose( const std::string& sObjectName );
        eLocType WhereAmI( Eigen::Vector3d P );
        eLocType WhereAmI( Eigen::Vector6d P );

    private:
//        void _ThreadFunction(Localizer *pVT);
        void _ThreadFunction(const nav_msgs::Odometry::ConstPtr);

        // UDP values
//        unsigned m_CarPort;
//        unsigned m_LocPort;
//        struct sockaddr_in locAddr;
//        struct sockaddr_in carAddr;
//        socklen_t addrLen = sizeof(locAddr);
//        int recvLen;
//        int sockFD;
//        unsigned char buf[2048];
//        unsigned int msgSize = 0;
//        hal::PoseMsg posys;

    private:

        struct TrackerObject
        {
            Sophus::SE3d        m_dSensorPose;
            Sophus::SE3d        m_dToffset;
            double              m_dTime;
            Localizer*          m_pLocalizerObject;
            boost::mutex        m_Mutex;
            boost::condition    m_PoseUpdated;

            //metrics
            double              m_dLastTime;
            int                 m_nNumPoses;
            double              m_dPoseRate;
            bool                m_bRobotFrame;
            bool                m_bPoseUpdated;
            TrackerObject() : m_dLastTime(-1), m_nNumPoses(-1) , m_bPoseUpdated(false)
            {
            }

            TrackerObject(const Localizer::TrackerObject& obj): m_Mutex(),
                m_PoseUpdated()
            {
                m_dSensorPose = obj.m_dSensorPose;
                m_dTime = obj.m_dTime;
                m_pLocalizerObject = obj.m_pLocalizerObject;
            }
        };

        std::map< std::string,  TrackerObject >     m_mObjects;
        bool                                        m_bIsStarted;
//        boost::thread*								m_pThread;

        ros::NodeHandle m_nh;
        ros::Subscriber m_threadSub;

};

#endif	/* LOCALIZER_H */
