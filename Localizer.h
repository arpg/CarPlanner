/*
 * File:   Localizer.h
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#ifndef LOCALIZER_H
#define	LOCALIZER_H


#include <Eigen/Eigen>
#include <eigen3/Eigen/src/Core/products/GeneralBlockPanelKernel.h>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <node/Node.h>
#include <HAL/Messages.pb.h>
#include <vector>
#include "CarPlannerCommon.h"


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
        void _ThreadFunction(Localizer *pVT);

    private:

        struct TrackerObject
        {
            Sophus::SE3d                        m_dSensorPose;
            Sophus::SE3d                        m_dToffset;
            double                                 m_dTime;
            bool																m_bNodeSubscribed;
            Localizer*                                 m_pLocalizerObject;
            boost::mutex                           m_Mutex;
            boost::condition                       m_PoseUpdated;

            //metrics
            double                                  m_dLastTime;
            int                                     m_nNumPoses;
            double                                  m_dPoseRate;
            bool                                    m_bRobotFrame;
            bool                                    m_bPoseUpdated;
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
        node::node*																	m_pNode;
        bool                                        m_bIsStarted;
        boost::thread*															m_pThread;
};

#endif	/* LOCALIZER_H */
