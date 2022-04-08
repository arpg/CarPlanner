/*
 * File:   Localizer.cpp
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#include <CarPlanner/TFLocalizer.h>

#include <math.h>

#include <CarPlanner/CarPlannerCommon.h>
#include <CarPlanner/MochaException.h>
#include <CarPlanner/RpgUtils.h>

//////////////////////////////////////////////////////////////////
TFLocalizer::TFLocalizer()
{
    LOG(INFO) << "New TFLocalizer created.";
}

//////////////////////////////////////////////////////////////////
void TFLocalizer::TrackObject(
        const std::string& sObjectName,
        bool bRobotFrame /*= true*/
        ){
    TrackObject(sObjectName,Sophus::SE3d(),bRobotFrame);
}

//////////////////////////////////////////////////////////////////
void TFLocalizer::TrackObject(
        const std::string& sObjectName,
        Sophus::SE3d dToffset,
        bool bRobotFrame /*= true*/
        )
{
    LOG(INFO) << "Tracking object " << sObjectName << ".";

    TrackerObject* pObj = &m_mObjects[ sObjectName ];

    pObj->m_dToffset = dToffset;
    pObj->m_bRobotFrame = bRobotFrame;
    pObj->m_pLocalizerObject = this;
}

//////////////////////////////////////////////////////////////////
TFLocalizer::~TFLocalizer()
{
}

//////////////////////////////////////////////////////////////////
void TFLocalizer::Start()
{
    LOG(INFO) << "Starting TFLocalizer.";

    if( m_bIsStarted == true ) {
        LOG(ERROR) << "TFLocalizer thread already started";
        //throw MochaException("The Localizer thread has already started.");
        return;
    }

    m_pThread = new boost::thread([this] () { TFLocalizer::_ThreadFunction(this); } );
    m_bIsStarted = true;

    //LOG(INFO) << "Localizer thread started.";
}

//////////////////////////////////////////////////////////////////
void TFLocalizer::Stop()
{
    if( m_bIsStarted == false ) {
        LOG(ERROR) << "No thread running!";
        //throw MochaException("No thread is running!");
        return;
    }

    m_pThread->interrupt();
    m_pThread->join();

    m_bIsStarted = false;

    LOG(INFO) << "TFLocalizer thread stopped.";
}

//////////////////////////////////////////////////////////////////
//
Sophus::SE3d TFLocalizer::GetPose( const std::string& sObjectName, bool blocking/* = false */,
                                          double* time /*= NULL*/, double* rate /*= NULL*/)
{
    if( m_mObjects.find( sObjectName ) == m_mObjects.end() ){
        throw MochaException("Invalid object name.");
    }

    TrackerObject& obj = m_mObjects[sObjectName];
    boost::mutex::scoped_lock lock(obj.m_Mutex);

    //if blocking wait until we have a signal that the pose for this object has been updated
    if(blocking && obj.m_bPoseUpdated == false){
        obj.m_PoseUpdated.wait(lock);
    }
    obj.m_bPoseUpdated = false;

    Sophus::SE3d pose = obj.m_dSensorPose;
    if(time != NULL){
        *time = m_mObjects[sObjectName].m_dTime;
    }
    if(rate != NULL){
        *rate = m_mObjects[sObjectName].m_dPoseRate;
    }
    return pose;
}

//////////////////////////////////////////////////////////////////
//
//Eigen::Matrix<double,6,1> TFLocalizer::GetdPose( const std::string& sObjectName )
//{
//    if( m_mObjects.find( sObjectName ) == m_mObjects.end() ){
//        throw MochaException("Invalid object name.");
//    }

//    //lock the sensor pose for readout
//    lock();
//    Eigen::Matrix<double,6,1> pose = m_mObjects[sObjectName].m_dDSensorPose;
//    unlock();

//    return pose;
//}

//////////////////////////////////////////////////////////////////////////////
Sophus::SE3d TFLocalizer::LookupPose(std::string objectName)
{
    // ROS_INFO("Looking up pose...");
    std::string parent_frame = "map";
    tf::StampedTransform Tmv;
    try
    {
        m_tflistener.waitForTransform(parent_frame, objectName, ros::Time::now(), ros::Duration(0.1));
        m_tflistener.lookupTransform(parent_frame, objectName, ros::Time(0), Tmv);
    }
    catch (const tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        return Sophus::SE3d();
    }

    Eigen::Vector3d Pos(Tmv.getOrigin().getX(), Tmv.getOrigin().getY(), Tmv.getOrigin().getZ());
    Eigen::Quaterniond Quat(Tmv.getRotation().getW(), Tmv.getRotation().getX(), Tmv.getRotation().getY(), Tmv.getRotation().getZ());
    Sophus::SE3d T( Sophus::SO3d(Quat), Pos );

    m_lastTime = Tmv.stamp_.toSec();

    // ROS_INFO("Got pose: %s->%s @ %f, px %f py %f pz %f qx %f qy %f qz %f qw %f", 
    //     parent_frame.c_str(), objectName.c_str(), m_lastTime, 
    //     T.translation().x(), T.translation().y(), T.translation().z(),
    //     T.unit_quaternion().x(), T.unit_quaternion().y(), T.unit_quaternion().z(), T.unit_quaternion().w());

    return T;
}

double TFLocalizer::LookupTime()
{
    return m_lastTime;
}