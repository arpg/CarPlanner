/*
 * File:   Localizer.h
 * Author: jmf
 *
 * Created on February 16, 2012, 5:07 PM
 */

#ifndef ROS_LOCALIZER_H
#define	ROS_LOCALIZER_H

#include <CarPlanner/Localizer.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <carplanner_tools/velocity_calculator.hpp>

class ROSLocalizer : public Localizer
{
public:
    ROSLocalizer();
    virtual ~ROSLocalizer();
    // void TrackObject(const std::string& sObjectName, bool bRobotFrame = true);
    // void TrackObject(const std::string& sObjectName, Sophus::SE3d dToffset, bool bRobotFrame = true);
    void Start();
    void Stop();
    // Sophus::SE3d GetPose(const std::string& sObjectName , bool blocking = false, double *time = NULL, double *rate = NULL);
    Sophus::Vector6d GetVelocity(const std::string& sObjectName , bool blocking = false, double *time = NULL, double *rate = NULL);
    //Eigen::Matrix<double,6,1> GetdPose( const std::string& sObjectName );
    inline std::string GetLocalizerType() {return "ROSLocalizer"; }

private:
    Sophus::SE3d LookupPose(std::string objectName);
    double LookupTime();

    // ROS variables
    tf::TransformListener m_tflistener;
    double m_lastTime;

    carplanner_tools::VelocityCalculator m_VelocityCalculator;
};

#endif	/* ROS_LOCALIZER_H */
