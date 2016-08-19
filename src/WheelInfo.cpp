/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
*/
#include <CarPlanner/WheelInfo.h>
#include "BulletDynamics/Dynamics/btRigidBody.h" // for pointvelocity


btScalar WheelInfo::getSuspensionRestLength() const
{

    return m_suspensionRestLength1;

}

void	WheelInfo::updateWheel(const btRigidBody& chassis,RaycastInfo& raycastInfo)
{
    (void)raycastInfo;


    if (m_raycastInfo.m_isInContact)

    {
        btScalar	project= m_raycastInfo.m_contactNormalWS.dot( m_raycastInfo.m_wheelDirectionWS );
        btVector3	 chassis_velocity_at_contactPoint;
        btVector3 relpos = m_raycastInfo.m_contactPointWS - chassis.getCenterOfMassPosition();
        chassis_velocity_at_contactPoint = chassis.getVelocityInLocalPoint( relpos );
        btScalar projVel = m_raycastInfo.m_contactNormalWS.dot( chassis_velocity_at_contactPoint );
        if ( project >= btScalar(-0.1))
        {
            m_suspensionRelativeVelocity = btScalar(0.0);
            m_clippedInvContactDotSuspension = btScalar(1.0) / btScalar(0.1);
        }
        else
        {
            btScalar inv = btScalar(-1.) / project;
            m_suspensionRelativeVelocity = projVel * inv;
            m_clippedInvContactDotSuspension = inv;
        }

    }

    else	// Not in contact : position wheel in a nice (rest length) position
    {
        m_raycastInfo.m_suspensionLength = this->getSuspensionRestLength();
        m_suspensionRelativeVelocity = btScalar(0.0);
        m_raycastInfo.m_contactNormalWS = -m_raycastInfo.m_wheelDirectionWS;
        m_clippedInvContactDotSuspension = btScalar(1.0);
    }
}

WheelInfo::WheelInfo()
{

}

WheelInfo::WheelInfo(const WheelInfo &wi)
{
    m_raycastInfo = wi.m_raycastInfo;
    m_worldTransform = wi.m_worldTransform;
    m_chassisConnectionPointCS = wi.m_chassisConnectionPointCS; //const
    m_wheelDirectionCS = wi.m_wheelDirectionCS;//const
    m_wheelAxleCS = wi.m_wheelAxleCS; // const or modified by steering
    m_suspensionRestLength1 = wi.m_suspensionRestLength1;//const
    m_maxSuspensionTravelCm = wi.m_maxSuspensionTravelCm;
    m_wheelsRadius = wi.m_wheelsRadius;//const
    m_suspensionStiffness = wi.m_suspensionStiffness;//const
    m_wheelsDampingCompression = wi.m_wheelsDampingCompression;//const
    m_wheelsDampingRelaxation = wi.m_wheelsDampingRelaxation;//const
    m_frictionSlip = wi.m_frictionSlip;
    m_steering = wi.m_steering;
    m_rotation = wi.m_rotation;
    m_deltaRotation = wi.m_deltaRotation;
    m_rollInfluence = wi.m_rollInfluence;
    m_maxSuspensionForce = wi.m_maxSuspensionForce;
    m_engineForce = wi.m_engineForce;
    m_brake = wi.m_brake;
    m_bIsFrontWheel = wi.m_bIsFrontWheel;
    m_clientInfo = wi.m_clientInfo;//can be used to store pointer to sync transforms...
    m_clippedInvContactDotSuspension = wi.m_clippedInvContactDotSuspension;
    m_suspensionRelativeVelocity = wi.m_suspensionRelativeVelocity;
    m_wheelsSuspensionForce = wi.m_wheelsSuspensionForce;
    m_skidInfo = wi.m_skidInfo;
}

WheelInfo::WheelInfo(WheelInfoConstructionInfo& ci)

{
    m_suspensionRestLength1 = ci.m_suspensionRestLength;
    m_maxSuspensionTravelCm = ci.m_maxSuspensionTravelCm;

    m_wheelsRadius = ci.m_wheelRadius;
    m_suspensionStiffness = ci.m_suspensionStiffness;
    m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
    m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
    m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
    m_wheelDirectionCS = ci.m_wheelDirectionCS;
    m_wheelAxleCS = ci.m_wheelAxleCS;
    m_frictionSlip = ci.m_frictionSlip;
    m_steering = btScalar(0.);
    m_engineForce = btScalar(0.);
    m_rotation = btScalar(0.);
    m_deltaRotation = btScalar(0.);
    m_brake = btScalar(0.);
    m_rollInfluence = btScalar(0.1);
    m_bIsFrontWheel = ci.m_bIsFrontWheel;
    m_maxSuspensionForce = ci.m_maxSuspensionForce;
    m_raycastInfo.m_suspensionLength = m_suspensionRestLength1;

}
