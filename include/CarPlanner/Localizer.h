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

#include <vector>
#include <string.h>
#include <unistd.h>
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
    virtual void TrackObject(const std::string& sObjectName, bool bRobotFrame = true);
    virtual void TrackObject(const std::string& sObjectName, Sophus::SE3d dToffset, bool bRobotFrame = true);
    virtual void Start();
    virtual void Stop();
    virtual Sophus::SE3d GetPose(const std::string& sObjectName , bool blocking = false, double *time = NULL, double *rate = NULL);
    // virtual //Eigen::Matrix<double,6,1> GetdPose( const std::string& sObjectName );
    // virtual eLocType WhereAmI( Eigen::Vector3d P );
    // virtual eLocType WhereAmI( Eigen::Vector6d P );

protected:
    virtual Sophus::SE3d LookupPose(std::string objectName) = 0;
    virtual double LookupTime() = 0;
    void _ThreadFunction(Localizer *pVT);

protected:
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
    boost::thread*								m_pThread;
};

#endif	/* LOCALIZER_H */
