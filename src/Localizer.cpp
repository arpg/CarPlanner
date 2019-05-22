/*
 * File:   Localizer.cpp
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#include <CarPlanner/Localizer.h>

#include <math.h>

#include <CarPlanner/CarPlannerCommon.h>
#include <CarPlanner/MochaException.h>
#include <CarPlanner/RpgUtils.h>

/// MUST set up a Posys object using the command:
/// PoseToNode -posys vicon://IP_address_to_vicon_machine:[NinjaCar]
///
/// When running PoseToNode, do NOT use a different node name; it is
/// hard coded to be `posetonode'.

//////////////////////////////////////////////////////////////////
Localizer::Localizer()
{
    LOG(INFO) << "New Localizer created.";
}

//////////////////////////////////////////////////////////////////
void Localizer::TrackObject(
        const std::string& sObjectName,
        const std::string& sHost,
        bool bRobotFrame /*= true*/
        ){
    TrackObject(sObjectName,sHost,Sophus::SE3d(),bRobotFrame);
}

//////////////////////////////////////////////////////////////////
void Localizer::TrackObject(
        const std::string& sObjectName,
        const std::string& sHost,
        Sophus::SE3d dToffset,
        bool bRobotFrame /*= true*/
        )
{
    std::string sUri = sHost + "/" + sObjectName ;

    LOG(INFO) << "Tracking object at topic " << sUri << ".";

    TrackerObject* pObj = &m_mObjects[ sObjectName ];

    pObj->m_dToffset = dToffset;
    pObj->m_bRobotFrame = bRobotFrame;
    pObj->m_pLocalizerObject = this;
}

//////////////////////////////////////////////////////////////////
Localizer::~Localizer()
{
}

//////////////////////////////////////////////////////////////////
void Localizer::Start()
{
    if( m_bIsStarted == true ) {
        LOG(ERROR) << "Localizer thread already started";
        //throw MochaException("The Localizer thread has already started.");
        return;
    }

//    m_pThread = new boost::thread([this] () { Localizer::_ThreadFunction(this); } );
    m_threadSub = m_nh.subscribe<nav_msgs::Odometry>("pose", 1, boost::bind(&Localizer::_ThreadFunction, this, _1));
    m_bIsStarted = true;

    //LOG(INFO) << "Localizer thread started.";
}

//////////////////////////////////////////////////////////////////
void Localizer::Stop()
{
    if( m_bIsStarted == false ) {
        LOG(ERROR) << "No thread running!";
        //throw MochaException("No thread is running!");
        return;
    }

//    m_pThread->interrupt();
//    m_pThread->join();

//    close(sockFD);

    m_bIsStarted = false;

    LOG(INFO) << "Localizer thread stopped.";
}

//////////////////////////////////////////////////////////////////
//
Sophus::SE3d Localizer::GetPose( const std::string& sObjectName, bool blocking/* = false */,
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
//Eigen::Matrix<double,6,1> Localizer::GetdPose( const std::string& sObjectName )
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

//////////////////////////////////////////////////////////////////
eLocType Localizer::WhereAmI( Eigen::Vector3d P )
{
    return VT_AIR;
}

//////////////////////////////////////////////////////////////////
eLocType Localizer::WhereAmI( Eigen::Matrix<double, 6, 1 > P )
{
    Eigen::Vector3d Pv = P.block < 3, 1 > (0, 0);
    return WhereAmI( Pv );
}

//////////////////////////////////////////////////////////////////
void Localizer::_ThreadFunction(const nav_msgs::Odometry::ConstPtr odom) {
  while (1) {
    std::map< std::string, TrackerObject >::iterator it;
    for( it = m_mObjects.begin(); it != m_mObjects.end(); it++ ) {
        // extract data from odom

      {
        boost::mutex::scoped_lock lock(it->second.m_Mutex);

        Eigen::Matrix4d T;
        if(it->second.m_bRobotFrame){
          T << 1, 0, 0, 0,
              0, -1, 0, 0,
              0, 0, 1, 0,
              0, 0 , 0, 1;
        } else {
          T = Eigen::Matrix4d::Identity();
        }

          Eigen::Vector3d Pos(odom->pose.pose.position.x, odom->pose.pose.position.y, -odom->pose.pose.position.z);
          Eigen::Quaterniond Quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

          //get the pose and transform it as necessary
          Sophus::SE3d Twc( Sophus::SO3d(Quat), Pos );

          Eigen::Matrix4d Tlw;
          Tlw <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0,-1, 0,
                  0, 0, 0, 1;



          it->second.m_dSensorPose = Sophus::SE3d(T) * (it->second.m_dToffset * Twc);
          // ^^MochaGui.cpp/_LocalizerReadFunc()/m_Localizer.GetPose()

          //std::cout << it->second.m_dSensorPose << std::endl; //debugging

          //now calculate the time derivative
          //used to be the next line, but now is modified for hal::PosysMsg.
          //double localizerTime = tData.msg_time.tv_sec + 1e-6*tData.msg_time.tv_usec;
          double localizerTime = odom->header.stamp.sec + 1e-9*odom->header.stamp.nsec;

          //calculate metrics
          if(it->second.m_dLastTime == -1){
            it->second.m_dLastTime = localizerTime;
            it->second.m_nNumPoses = 0;
            it->second.m_dPoseRate = 0;
          }else if((localizerTime - it->second.m_dLastTime) > 1){
            it->second.m_dPoseRate = it->second.m_nNumPoses /(localizerTime - it->second.m_dLastTime) ;
            it->second.m_dLastTime = localizerTime;
            it->second.m_nNumPoses = 0;
          }
          it->second.m_nNumPoses++;
          it->second.m_dTime = localizerTime;

      }

      //signal that the object has been update
      it->second.m_bPoseUpdated = true;
      it->second.m_PoseUpdated.notify_all();
      //LOG(INFO) << "Pose Updated";
    }

    boost::this_thread::interruption_point();
  }

  //small sleep to not eat up all the cpu
  usleep(1000);
}
