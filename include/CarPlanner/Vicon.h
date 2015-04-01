/*
 * File:   Vicon.h
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#ifndef VICON_H
#define	VICON_H


#include <Eigen/Eigen>
#include <eigen3/Eigen/src/Core/products/GeneralBlockPanelKernel.h>

//This is required to compile with Wall
#pragma GCC diagnostic push
    # pragma GCC diagnostic ignored "-Wunused-variable"
    #include <vrpn_Tracker.h>
#pragma GCC diagnostic pop

#include <vector>
#include <condition_variable>
#include "CarPlannerCommon.h"


//struct MochaEntity
//{
//    std::string m_Name;
//    Eigen::Vector6d m_Pose;
//    Eigen::Vector6d m_DPose;
//};

class Vicon
{
    public:
        Vicon();
        virtual ~Vicon();
        void TrackObject(const std::string& sObjectName, const std::string& sHost , bool bRobotFrame = true);
        void TrackObject(const std::string& sObjectName, const std::string& sHost , Sophus::SE3d dToffset, bool bRobotFrame = true);
        void Start();
        void Stop();
        Sophus::SE3d GetPose(const std::string& sObjectName , bool blocking = false, double *time = NULL, double *rate = NULL);
        //Eigen::Matrix<double,6,1> GetdPose( const std::string& sObjectName );
        eLocType WhereAmI( Eigen::Vector3d P );
        eLocType WhereAmI( Eigen::Vector6d P );

    private:
        static void _ThreadFunction(Vicon *pVT);
        static void VRPN_CALLBACK _MoCapHandler(void* uData, const vrpn_TRACKERCB tData );
        static void VRPN_CALLBACK _MoCapVelHandler( void* uData, const vrpn_TRACKERVELCB tData );

    private:

        struct TrackerObject
        {
            Sophus::SE3d                        m_dSensorPose;
            Sophus::SE3d                        m_dToffset;
            double                                 m_dTime;
            vrpn_Tracker_Remote*                   m_pTracker;
            Vicon*                                 m_pViconObject;
            std::mutex														m_Mutex;
            std::condition_variable 							m_PoseUpdated;

            //metrics
            double                                  m_dLastTime;
            int                                     m_nNumPoses;
            double                                  m_dPoseRate;
            bool                                    m_bRobotFrame;
            bool                                    m_bPoseUpdated;
            TrackerObject() : m_dLastTime(-1), m_nNumPoses(-1) , m_bPoseUpdated(false)
            {
            }

            TrackerObject(const Vicon::TrackerObject& obj): m_Mutex(),
                m_PoseUpdated()
            {
                m_dSensorPose = obj.m_dSensorPose;
                m_dTime = obj.m_dTime;
                m_pTracker = obj.m_pTracker;
                m_pViconObject = obj.m_pViconObject;

            }
        };

        std::map< std::string,  TrackerObject >     m_mObjects;

        bool                                        m_bIsStarted;
        std::thread*								m_pThread;
        vrpn_Connection*                            m_pViconConnection;
};

#endif	/* VICON_H */
