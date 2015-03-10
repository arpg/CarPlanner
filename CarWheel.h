#ifndef CARWHEEL_H
#define CARWHEEL_H

#include "Eigen/Eigen"
#include "CarPlannerCommon.h"
#include "RigidBody.h"

class CarWheel
{
public:
    CarWheel(Eigen::Vector6d cgOffset, double k, double d, double maxExtension,double minExtension)
    {
        m_dLastTime = -1;
        //initialize the position of the wheels.
        m_dK = k;
        m_dD = d;
        m_dMaxExtension = maxExtension;
        m_dMinExtension = minExtension;

        //assume max extension
        m_dExtension = m_dMaxExtension;

        m_vCgOffset = mvl::Cart2T( cgOffset );
        
        m_bInContact = false;
    }
    
    double PredictForce(double groundOffset, double dT, bool contact) {
        double oldExt = m_dExtension;
        //first add the offset to the current offset
        double ext = m_dExtension + groundOffset;
        
        //cap the current offset
        if(ext > m_dMaxExtension) {
            ext = m_dMaxExtension;
        }else if( ext < m_dMinExtension ) {
            ext = m_dMinExtension;
        }
        
        //based on this, calcualte the force 
        double dForce = (m_dMaxExtension-ext)*m_dK;
        if(dT > 0) {
            dForce += (-(ext-oldExt)/dT)*m_dD;
        }
        
        return dForce;
    }

    //groundOffset +ve is away from the car and -ve is towards the car
    //the force is measured as +ve away from the car
    void UpdateState(double groundOffset, double dT, bool contact)
    {
        double oldExt = m_dExtension;
        //first add the offset to the current offset
        m_dExtension += groundOffset;
        
        //cap the current offset
        if(m_dExtension > m_dMaxExtension) {
            m_dExtension = m_dMaxExtension;
        }else if( m_dExtension < m_dMinExtension ) {
            m_dExtension = m_dMinExtension;
        }
        
        //based on this, calcualte the force 
        m_dForce = (m_dMaxExtension-m_dExtension)*m_dK;
        if(dT > 0) {
            m_dForce += (-(m_dExtension-oldExt)/dT)*m_dD;
        }
    }
    
    Eigen::Matrix4d& GetRelativeWheelPose() {
        Eigen::Matrix4d pose = m_vCgOffset;
        //add the current extension of the wheel to the offset
        Eigen::Vector6d vExtension;
        vExtension << 0,0,m_dExtension,0,0,0;
        
        Eigen::Matrix4d extension = mvl::Cart2T(vExtension);
        m_vRelativeOffset = pose*extension;
        return m_vRelativeOffset;
    }

    Eigen::Matrix4d& GetCgOffset(){ 
        return m_vCgOffset; 
    }
    
    Eigen::Vector6d GetWorldPose() {
        return m_WorldPose;
    }
    
    void SetWorldPose(Eigen::Vector6d pose) {
        m_WorldPose = pose;
    }
    
    void SetInContact(bool bInContact) {
        m_bInContact = bInContact;
    }
    
    bool GetInContact() {
        return m_bInContact;
    }
    
    double GetExtension() {
        return m_dExtension;
    }
    
    const double GetMinExtension() {
        return m_dMinExtension;
    }
    
    const double GetMaxExtension() {
        return m_dMaxExtension;
    }
    
    double GetForce() {
        return m_dForce;
    }
    
    void SetExtension(double dExtension) {
        m_dExtension = dExtension;
    }
    
    bool IsAtMaxExtension() {
        return m_dExtension == m_dMaxExtension;
    }
    
    
    void OffsetExtension(double dOffset) {
        m_dExtension += dOffset;
        
        if(m_dExtension > m_dMaxExtension) {
            m_dExtension = m_dMaxExtension;
        }else if( m_dExtension < m_dMinExtension ) {
            m_dExtension = m_dMinExtension;
        }
    }

private:
    //the offset location wrt the center of mass in the reference frame of the car
//    Eigen::Vector6d m_vCgOffset;
    Eigen::Matrix4d m_vCgOffset;
    Eigen::Matrix4d m_vRelativeOffset;
    Eigen::Vector6d m_WorldPose;
    double m_dK;
    double m_dD;
    double m_dLastTime;
    //state space:
    double m_dExtension;    //how far has the spring extended
    double m_dVelocity;     //what is the velocity of the contact point?
    double m_dForce;        //the force of the spring and damper combined

    //physical constraints
    double m_dMaxExtension;
    double m_dMinExtension;
    
    bool m_bInContact;

};

#endif // CARWHEEL_H
