/*
 * File:   Localizer.h
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#ifndef TF_LOCALIZER_H
#define	TF_LOCALIZER_H

#include <CarPlanner/Localizer.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

class TFLocalizer : Localizer
{
public:
    TFLocalizer();
    virtual ~TFLocalizer();
    void TrackObject(const std::string& sObjectName, bool bRobotFrame = true);
    void TrackObject(const std::string& sObjectName, Sophus::SE3d dToffset, bool bRobotFrame = true);
    void Start();
    void Stop();
    Sophus::SE3d GetPose(const std::string& sObjectName , bool blocking = false, double *time = NULL, double *rate = NULL);
    //Eigen::Matrix<double,6,1> GetdPose( const std::string& sObjectName );

private:
    Sophus::SE3d LookupPose(std::string objectName);
    double LookupTime();

    // ROS variables
    tf::TransformListener m_tflistener;
    double m_lastTime;
};

#endif	/* TF_LOCALIZER_H */
