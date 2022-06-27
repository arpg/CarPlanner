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

#include "LinearMath/btVector3.h"
#include <CarPlanner/RaycastVehicle.h>

#include "BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
#include "LinearMath/btQuaternion.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include <CarPlanner/CarPlannerCommon.h>

#include "BulletCollision/CollisionShapes/btCollisionShape.h"

#define ROLLING_INFLUENCE_FIX

/////////////////////////////////////////////////////////////////////////////////////////
RaycastVehicle::RaycastVehicle(){

}

/////////////////////////////////////////////////////////////////////////////////////////
RaycastVehicle::RaycastVehicle(const btVehicleTuning& tuning,btRigidBody* chassis,	btVehicleRaycaster* raycaster,btContactSolverInfo *pSolverInfo )
    :m_vehicleRaycaster(raycaster), m_pSolverInfo(pSolverInfo),m_pitchControl(btScalar(0.))
{
    m_chassisBody = chassis;
    m_indexRightAxis = 0;
    m_indexUpAxis = 2;
    m_indexForwardAxis = 1;
    defaultInit(tuning);
    m_dAckermanSteering = 0;
}


/////////////////////////////////////////////////////////////////////////////////////////
btScalar RaycastVehicle::resolveSingleCollision(
        btRigidBody* body1,
        btCollisionObject* colObj2,
        const btVector3& contactPositionWorld,
        const btVector3& contactNormalOnB,
        const btContactSolverInfo& solverInfo,
        btScalar distance)
{
    btRigidBody* body2 = btRigidBody::upcast(colObj2);


    const btVector3& normal = contactNormalOnB;

    btVector3 rel_pos1 = contactPositionWorld - body1->getWorldTransform().getOrigin();
    btVector3 rel_pos2 = contactPositionWorld - colObj2->getWorldTransform().getOrigin();

    btVector3 vel1 = body1->getVelocityInLocalPoint(rel_pos1);
    btVector3 vel2 = body2? body2->getVelocityInLocalPoint(rel_pos2) : btVector3(0,0,0);
    btVector3 vel = vel1 - vel2;
    btScalar rel_vel;
    rel_vel = normal.dot(vel);

    btScalar combinedRestitution = 0.f;
    btScalar restitution = combinedRestitution* -rel_vel;

    btScalar positionalError = solverInfo.m_erp *-distance /solverInfo.m_timeStep ;
    btScalar velocityError = -(1.0f + restitution) * rel_vel;// * damping;
    btScalar denom0 = body1->computeImpulseDenominator(contactPositionWorld,normal);
    btScalar denom1 = body2? body2->computeImpulseDenominator(contactPositionWorld,normal) : 0.f;
    btScalar relaxation = 1.f;
    btScalar jacDiagABInv = relaxation/(denom0+denom1);

    btScalar penetrationImpulse = positionalError * jacDiagABInv;
    btScalar velocityImpulse = velocityError * jacDiagABInv;

    btScalar normalImpulse = penetrationImpulse+velocityImpulse;
    normalImpulse = 0.f > normalImpulse ? 0.f: normalImpulse;

    body1->applyImpulse(normal*(normalImpulse), rel_pos1);
    if (body2)
        body2->applyImpulse(-normal*(normalImpulse), rel_pos2);

    return normalImpulse;
}

/////////////////////////////////////////////////////////////////////////////////////////
RaycastVehicle::RaycastVehicle(RaycastVehicle & vehicle)
{
    m_pSolverInfo = vehicle.m_pSolverInfo;
    m_axle = vehicle.m_axle;
    m_chassisBody = vehicle.m_chassisBody;
    m_currentVehicleSpeedKmHour = vehicle.m_currentVehicleSpeedKmHour;
    m_damping = vehicle.m_damping;
    m_forwardImpulse = vehicle.m_forwardImpulse;
    m_forwardWS = vehicle.m_forwardWS;
    m_indexForwardAxis = vehicle.m_indexForwardAxis;
    m_indexRightAxis = vehicle.m_indexRightAxis;
    m_indexUpAxis = vehicle.m_indexUpAxis;
    m_pitchControl = vehicle.m_pitchControl;
    m_sideImpulse = vehicle.m_sideImpulse;
    m_steeringValue = vehicle.m_steeringValue;
    m_tau = vehicle.m_tau;
    m_userConstraintId = vehicle.m_userConstraintId;
    m_userConstraintType = vehicle.m_userConstraintType;
    m_vehicleRaycaster = vehicle.m_vehicleRaycaster;
    m_wheelInfo = vehicle.m_wheelInfo;
    m_dAckermanSteering = vehicle.m_dAckermanSteering;
}

/////////////////////////////////////////////////////////////////////////////////////////
void RaycastVehicle::defaultInit(const btVehicleTuning& tuning)
{
    (void)tuning;
    m_currentVehicleSpeedKmHour = btScalar(0.);
    m_steeringValue = btScalar(0.);

}

/////////////////////////////////////////////////////////////////////////////////////////
RaycastVehicle::~RaycastVehicle()
{
}


/////////////////////////////////////////////////////////////////////////////////////////
WheelInfo&	RaycastVehicle::addWheel( const btVector3& connectionPointCS, const btVector3& wheelDirectionCS0,const btVector3& wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius,const btVehicleTuning& tuning, bool isFrontWheel)
{
    WheelInfoConstructionInfo ci;

    ci.m_chassisConnectionCS = connectionPointCS;
    ci.m_wheelDirectionCS = wheelDirectionCS0;
    ci.m_wheelAxleCS = wheelAxleCS;
    ci.m_suspensionRestLength = suspensionRestLength;
    ci.m_wheelRadius = wheelRadius;
    ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
    ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
    ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
    ci.m_frictionSlip = tuning.m_frictionSlip;
    ci.m_bIsFrontWheel = isFrontWheel;
    ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
    ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

    m_wheelInfo.push_back( WheelInfo(ci));

    WheelInfo& wheel = m_wheelInfo[getNumWheels()-1];

    updateWheelTransformsWS( wheel , false );
    updateWheelTransform(getNumWheels()-1,false);
    return wheel;
}

/////////////////////////////////////////////////////////////////////////////////////////
const btTransform&	RaycastVehicle::getWheelTransformWS( int wheelIndex ) const
{
    btAssert(wheelIndex < getNumWheels());
    const WheelInfo& wheel = m_wheelInfo[wheelIndex];
    return wheel.m_worldTransform;

}

/////////////////////////////////////////////////////////////////////////////////////////
void	RaycastVehicle::updateWheelTransform( const int nWheelIndex , bool bInterpolatedTransform)
{
    WheelInfo& wheel = m_wheelInfo[ nWheelIndex ];
    updateWheelTransformsWS(wheel,bInterpolatedTransform);
    UpdateWheelSteeringTransform(nWheelIndex);
    wheel.m_worldTransform.setOrigin(
        wheel.m_raycastInfo.m_hardPointWS + wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength
    );
}

/////////////////////////////////////////////////////////////////////////////////////////
void RaycastVehicle::UpdateWheelSteeringTransform(const int nWheelIndex){
    //rotate around steering over de wheelAxleWS
    WheelInfo& wheel = m_wheelInfo[ nWheelIndex ];

    btVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
    const btVector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
    btVector3 fwd = right.cross(up);
    fwd = fwd.normalize();

    btScalar steering = wheel.m_steering;

    btQuaternion steeringOrn(up,steering);//wheel.m_steering);
    btMatrix3x3 steeringMat(steeringOrn);

    btQuaternion rotatingOrn(right,-wheel.m_rotation);
    btMatrix3x3 rotatingMat(rotatingOrn);

    btMatrix3x3 basis2(
        fwd[0],right[0],up[0],
        fwd[1],right[1],up[1],
        fwd[2],right[2],up[2]
    );

    wheel.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
}

/////////////////////////////////////////////////////////////////////////////////////////
void RaycastVehicle::resetSuspension()
{
    for (int ii=0;ii<m_wheelInfo.size();	ii++){
            WheelInfo& wheel = m_wheelInfo[ii];
            wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
            wheel.m_suspensionRelativeVelocity = btScalar(0.0);

            wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
            //wheel_info.setContactFriction(btScalar(0.0));
            wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void	RaycastVehicle::updateWheelTransformsWS(WheelInfo& wheel , bool interpolatedTransform)
{
    wheel.m_raycastInfo.m_isInContact = false;

    btTransform chassisTrans = getChassisWorldTransform();
    if (interpolatedTransform && (getRigidBody()->getMotionState())){
        getRigidBody()->getMotionState()->getWorldTransform(chassisTrans);
    }

    wheel.m_raycastInfo.m_hardPointWS = chassisTrans( wheel.m_chassisConnectionPointCS );
    wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() *  wheel.m_wheelDirectionCS ;
    wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

/////////////////////////////////////////////////////////////////////////////////////////
btScalar RaycastVehicle::rayCast(WheelInfo& wheel)
{
    updateWheelTransformsWS( wheel,false);


    btScalar depth = -1;

    btScalar raylen = wheel.getSuspensionRestLength()+wheel.m_wheelsRadius;

    //Instead of a ray going from the wheel contact down, extend the ray back up as well
    btVector3 rayvector = wheel.m_raycastInfo.m_wheelDirectionWS * (raylen);
    btVector3& source = wheel.m_raycastInfo.m_hardPointWS;
    wheel.m_raycastInfo.m_contactPointWS = source + rayvector;
    source = source - rayvector;
    const btVector3& target = wheel.m_raycastInfo.m_contactPointWS;

    btScalar param = btScalar(0.);

    btVehicleRaycaster::btVehicleRaycasterResult	rayResults;

    btAssert(m_vehicleRaycaster);

    void* object = m_vehicleRaycaster->castRay(source,target,rayResults);

    wheel.m_raycastInfo.m_groundObject = 0;

    if (object){
        //readjust the distFraction as now we are casting a ray from further above
        rayResults.m_distFraction = (rayResults.m_distFraction*2 - 1);
        param = rayResults.m_distFraction;
        depth = raylen * rayResults.m_distFraction;
        wheel.m_raycastInfo.m_contactNormalWS  = rayResults.m_hitNormalInWorld;
        wheel.m_raycastInfo.m_isInContact = true;

        wheel.m_raycastInfo.m_groundObject = &getFixedBody();///@todo for driving on dynamic/movable objects!;
        //wheel.m_raycastInfo.m_groundObject = object;


        btScalar hitDistance = param*raylen;
        wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;
        //clamp on max suspension travel

        btScalar  minSuspensionLength = wheel.getSuspensionRestLength() - wheel.m_maxSuspensionTravelCm*btScalar(0.01);
        btScalar maxSuspensionLength = wheel.getSuspensionRestLength()+ wheel.m_maxSuspensionTravelCm*btScalar(0.01);

        if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength){
            wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
        }

        if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength){
            wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;
        }

        wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

        btScalar denominator= wheel.m_raycastInfo.m_contactNormalWS.dot( wheel.m_raycastInfo.m_wheelDirectionWS );

        btVector3 chassis_velocity_at_contactPoint;
        btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS-getRigidBody()->getCenterOfMassPosition();

        chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

        btScalar projVel = wheel.m_raycastInfo.m_contactNormalWS.dot( chassis_velocity_at_contactPoint );

        if ( denominator >= btScalar(-0.1)){
            wheel.m_suspensionRelativeVelocity = btScalar(0.0);
            wheel.m_clippedInvContactDotSuspension = btScalar(1.0) / btScalar(0.1);
        }
        else{
            btScalar inv = btScalar(-1.) / denominator;
            wheel.m_suspensionRelativeVelocity = projVel * inv;
            wheel.m_clippedInvContactDotSuspension = inv;
        }

    } else{
        //put wheel info as in rest position
        wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
        wheel.m_suspensionRelativeVelocity = btScalar(0.0);
        wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
        wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
    }

    return depth;
}

/////////////////////////////////////////////////////////////////////////////////////////
const btTransform& RaycastVehicle::getChassisWorldTransform() const
{
    return getRigidBody()->getCenterOfMassTransform();
}

/////////////////////////////////////////////////////////////
void RaycastVehicle::updateCollision(btCollisionWorld* world)
{
    // std::cout << "*** CHecking for collision (" << std::to_string(world->getDispatcher()->getNumManifolds()) << " manifolds) ** " << std::endl;
    m_bChassisInCollision = false;
	for (int i = 0; i < world->getDispatcher()->getNumManifolds(); i++)
	{
		btPersistentManifold* manifold = world->getDispatcher()->getManifoldByIndexInternal(i);
		if (!manifold->getNumContacts())
			continue;

		btScalar minDist = 0.01f;
		for (int v = 0; v < manifold->getNumContacts(); v++)
		{
			minDist = btMin(minDist, manifold->getContactPoint(v).getDistance());
		}
		if (minDist > 0.)
			continue;

        
        // std::cout << "**** DETECte COLLISION " << std::to_string(manifold->getNumContacts()) << " ****" << std::endl;

		btCollisionObject* colObj0 = (btCollisionObject*)manifold->getBody0();
		btCollisionObject* colObj1 = (btCollisionObject*)manifold->getBody1();

        btCollisionObject* otherBody;
        if (colObj0==m_chassisBody)
            otherBody = colObj1;
        else if (colObj1==m_chassisBody)
            otherBody = colObj0;
        else
            continue;

        // if we reach this point, at least one of the collision objects is the chassis

        // std::cout << "*** chassis in collision ";
        // if (otherBody->getCollisionShape()->getUserIndex()==CSI_GROUNDPLANE)
        //     std::cout << "with groundplane ***" << std::endl;
        // else if (otherBody->getCollisionShape()->getUserIndex()==CSI_TERRAIN)
        //     std::cout << "with terrain ***" << std::endl;

        // only set chassisincollision to true if the other body is the terrain
        if (otherBody->getCollisionShape()->getUserIndex()==CSI_TERRAIN)
        {
            m_bChassisInCollision = true;
            return;
        }
	}
}

// void RaycastVehicle::updateCollision(btCollisionWorld* world)
// {
//     std::vector<CollisionEvent> collisions;
//     getCollisions(pWorld->m_pDynamicsWorld, collisions);
// }

// static
// void RaycastVehicle::getCollisions(btCollisionWorld* world, std::vector<CollisionEvent> collisions)
// {
//     int numManifolds = world->getDispatcher()->getNumManifolds();
//     for (int i=0;i<numManifolds;i++)
//     {
//         btPersistentManifold* contactManifold =  world->getDispatcher()->getManifoldByIndexInternal(i);
//         const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
//         const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());

//         btVector3 avgpt(0,0,0);

//         int numContacts = contactManifold->getNumContacts();
//         for (int j=0;j<numContacts;j++)
//         {
//             btManifoldPoint& pt = contactManifold->getContactPoint(j);
//             if (pt.getDistance()<0.f)
//             {
//                 const btVector3& ptA = pt.getPositionWorldOnA();
//                 const btVector3& ptB = pt.getPositionWorldOnB();
//                 const btVector3& normalOnB = pt.m_normalWorldOnB;

//                 avgpt += ptA + ptB;
//             }
//         }

//         if (numContacts>0)
//         {
//             avgpt /= numContacts*2;
//             collisions.push_back(CollisionEvent(obA, obB, avgpt));
//         }
//     }
// }

/////////////////////////////////////////////////////////////////////////////////////////
void RaycastVehicle::updateVehicle( btScalar step )
{
    {
        for (size_t i=0;i<getNumWheels();i++){
            updateWheelTransform(i,false);
        }
    }


    m_currentVehicleSpeedKmHour = btScalar(3.6) * getRigidBody()->getLinearVelocity().length();

    const btTransform& chassisTrans = getChassisWorldTransform();

    btVector3 forwardW (
        chassisTrans.getBasis()[0][m_indexForwardAxis],
        chassisTrans.getBasis()[1][m_indexForwardAxis],
        chassisTrans.getBasis()[2][m_indexForwardAxis]);

    if (forwardW.dot(getRigidBody()->getLinearVelocity()) < btScalar(0.)){
        m_currentVehicleSpeedKmHour *= btScalar(-1.);
    }

    //
    // simulate suspension
    //

    int i=0;
    for (i=0;i<m_wheelInfo.size();i++){
        //btScalar depth;
        rayCast( m_wheelInfo[i]);
    }

    updateSuspension();


    for (i=0;i<m_wheelInfo.size();i++){
        //apply suspension force
        WheelInfo& wheel = m_wheelInfo[i];

        btScalar suspensionForce = wheel.m_wheelsSuspensionForce;

        if (suspensionForce > wheel.m_maxSuspensionForce){
            suspensionForce = wheel.m_maxSuspensionForce;
        }
        btVector3 impulse = wheel.m_raycastInfo.m_contactNormalWS * suspensionForce * step;
        btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

        getRigidBody()->applyImpulse(impulse, relpos);

    }

    //get the total gravity force before applying the friction
    m_dTotalGravityForce = (getRigidBody()->getWorldTransform().getRotation() * getRigidBody()->getTotalForce()).x();

    updateFriction( step);

    for (i=0;i<m_wheelInfo.size();i++){
        WheelInfo& wheel = m_wheelInfo[i];
        btVector3 relpos = wheel.m_raycastInfo.m_hardPointWS - getRigidBody()->getCenterOfMassPosition();
        btVector3 vel = getRigidBody()->getVelocityInLocalPoint( relpos );

        if (wheel.m_raycastInfo.m_isInContact){
            const btTransform&	chassisWorldTransform = getChassisWorldTransform();

            btVector3 fwd (
                chassisWorldTransform.getBasis()[0][m_indexForwardAxis],
                chassisWorldTransform.getBasis()[1][m_indexForwardAxis],
                chassisWorldTransform.getBasis()[2][m_indexForwardAxis]);

            btScalar proj = fwd.dot(wheel.m_raycastInfo.m_contactNormalWS);
            fwd -= wheel.m_raycastInfo.m_contactNormalWS * proj;

            btScalar proj2 = fwd.dot(vel);

            wheel.m_deltaRotation = (proj2 * step) / (wheel.m_wheelsRadius);
            wheel.m_rotation += wheel.m_deltaRotation;

        } else{
            wheel.m_rotation += wheel.m_deltaRotation;
        }

        wheel.m_deltaRotation *= btScalar(0.99);//damping of rotation when not in contact

    }



}

void RaycastVehicle::SetAckermanSteering(btScalar steering)
{
    //get the baseline of the two wheels
    double width = fabs(m_wheelInfo[1].m_chassisConnectionPointCS[1]-m_wheelInfo[0].m_chassisConnectionPointCS[1]);
    double length = fabs(m_wheelInfo[1].m_chassisConnectionPointCS[0]-m_wheelInfo[2].m_chassisConnectionPointCS[0]);


    //get the inner angle
    btScalar inner =  atan(1.0/((1.0/tan(fabs(steering))) - width/(2*length) ));
    //get the outer angle
    btScalar outer = atan(1.0/(width/length + (1.0/tan(inner))));

    inner *= sgn(steering);
    outer *= sgn(steering);

    //now set the wheel angles depending on the steering
    if(steering >= 0 ){
        _SetSteeringValue(inner,1);
        _SetSteeringValue(outer,0);
    }else{
        _SetSteeringValue(inner,0);
        _SetSteeringValue(outer,1);
    }
    m_dAckermanSteering = steering;
}

/////////////////////////////////////////////////////////////////////////////////////////
void	RaycastVehicle::_SetSteeringValue(btScalar steering,int wheel)
{
    btAssert(wheel>=0 && wheel < getNumWheels());

    WheelInfo& wheelInfo = getWheelInfo(wheel);
    wheelInfo.m_steering = steering;
}

/////////////////////////////////////////////////////////////////////////////////////////
btScalar	RaycastVehicle::_GetSteeringValue(int wheel) const
{
    return getWheelInfo(wheel).m_steering;
}

/////////////////////////////////////////////////////////////////////////////////////////
void	RaycastVehicle::applyEngineForce(btScalar force, int wheel)
{
    btAssert(wheel>=0 && wheel < getNumWheels());
    WheelInfo& wheelInfo = getWheelInfo(wheel);
    wheelInfo.m_engineForce = force;
}

/////////////////////////////////////////////////////////////////////////////////////////
const WheelInfo&	RaycastVehicle::getWheelInfo(int index) const
{
    btAssert((index >= 0) && (index < 	getNumWheels()));

    return m_wheelInfo[index];
}

/////////////////////////////////////////////////////////////////////////////////////////
WheelInfo&	RaycastVehicle::getWheelInfo(int index)
{
    btAssert((index >= 0) && (index < 	getNumWheels()));

    return m_wheelInfo[index];
}

/////////////////////////////////////////////////////////////////////////////////////////
void RaycastVehicle::setBrake(btScalar brake,int wheelIndex)
{
    btAssert((wheelIndex >= 0) && (wheelIndex < 	getNumWheels()));
    getWheelInfo(wheelIndex).m_brake = brake;
}


/////////////////////////////////////////////////////////////////////////////////////////
void	RaycastVehicle::updateSuspension()
{
    //(void)deltaTime;

    //btScalar chassisMass = btScalar(1.) / m_chassisBody->getInvMass();

    for (size_t w_it=0; w_it<getNumWheels(); w_it++){
        WheelInfo &wheel_info = m_wheelInfo[w_it];

        if ( wheel_info.m_raycastInfo.m_isInContact ){
            btScalar force;
            //	Spring
            {
                btScalar	susp_length			= wheel_info.getSuspensionRestLength();
                btScalar	current_length = wheel_info.m_raycastInfo.m_suspensionLength;

                btScalar length_diff = (susp_length - current_length);

                if(current_length <= 0){
                    force  = 0;
                    btCollisionObject obj;
                    resolveSingleCollision(m_chassisBody,&obj,wheel_info.m_raycastInfo.m_contactPointWS,
                                           wheel_info.m_raycastInfo.m_contactNormalWS,*m_pSolverInfo,current_length);

                }else{
                    force = wheel_info.m_suspensionStiffness * length_diff * wheel_info.m_clippedInvContactDotSuspension;
                }
            }

            // Damper
            {
                btScalar projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
                {
                    btScalar	susp_damping;
                    if ( projected_rel_vel < btScalar(0.0) ){
                        susp_damping = wheel_info.m_wheelsDampingCompression;
                    }
                    else{
                        susp_damping = wheel_info.m_wheelsDampingRelaxation;
                    }
                    force -= susp_damping * projected_rel_vel;
                }
            }

            // RESULT
            wheel_info.m_wheelsSuspensionForce = force; //<< why was this multiplied by mass??
            if (wheel_info.m_wheelsSuspensionForce < btScalar(0.)){
                wheel_info.m_wheelsSuspensionForce = btScalar(0.);
            }
        }
        else{
            wheel_info.m_wheelsSuspensionForce = btScalar(0.0);
        }
    }

}


/////////////////////////////////////////////////////////////////////////////////////////
btScalar CalcRollingFriction(btWheelContactPoint& contactPoint);
btScalar CalcRollingFriction(btWheelContactPoint& contactPoint)
{

    btScalar j1=0.f;

    const btVector3& contactPosWorld = contactPoint.m_frictionPositionWorld;

    btVector3 rel_pos1 = contactPosWorld - contactPoint.m_body0->getCenterOfMassPosition();
    btVector3 rel_pos2 = contactPosWorld - contactPoint.m_body1->getCenterOfMassPosition();

    btScalar maxImpulse  = contactPoint.m_maxImpulse;

    btVector3 vel1 = contactPoint.m_body0->getVelocityInLocalPoint(rel_pos1);
    btVector3 vel2 = contactPoint.m_body1->getVelocityInLocalPoint(rel_pos2);
    btVector3 vel = vel1 - vel2;

    btScalar vrel = contactPoint.m_frictionDirectionWorld.dot(vel);

    // calculate j that moves us to zero relative velocity
    j1 = -vrel * contactPoint.m_jacDiagABInv;
    btSetMin(j1, maxImpulse);
    btSetMax(j1, -maxImpulse);

    return j1;
}


/////////////////////////////////////////////////////////////////////////////////////////
void	RaycastVehicle::updateFriction(btScalar	timeStep)
{
        //btScalar sideFrictionStiffness2 = btScalar(1.0);
        //calculate the impulse, so that the wheels don't move sidewards
        int numWheel = getNumWheels();
        if (!numWheel)
            return;

        m_forwardWS.resize(numWheel);
        m_axle.resize(numWheel);
        m_forwardImpulse.resize(numWheel);
        m_sideImpulse.resize(numWheel);

        int numWheelsOnGround = 0;


        //collapse all those loops into one!
        for (size_t i=0;i<getNumWheels();i++){
            WheelInfo& wheelInfo = m_wheelInfo[i];
            class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;
            if (groundObject)
                numWheelsOnGround++;
            m_sideImpulse[i] = btScalar(0.);
            m_forwardImpulse[i] = btScalar(0.);

        }

        {

            for (size_t i=0;i<getNumWheels();i++){
                WheelInfo& wheelInfo = m_wheelInfo[i];

                class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

                if (groundObject){

                    const btTransform& wheelTrans = getWheelTransformWS( i );

                    btMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
                    m_axle[i] = btVector3(
                        wheelBasis0[0][m_indexRightAxis],
                        wheelBasis0[1][m_indexRightAxis],
                        wheelBasis0[2][m_indexRightAxis]);

                    const btVector3& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
                    btScalar proj = m_axle[i].dot(surfNormalWS);
                    m_axle[i] -= surfNormalWS * proj;
                    m_axle[i] = m_axle[i].normalize();

                    m_forwardWS[i] = surfNormalWS.cross(m_axle[i]);
                    m_forwardWS[i].normalize();


                    resolveSingleBilateral(*m_chassisBody, wheelInfo.m_raycastInfo.m_contactPointWS,
                              *groundObject, wheelInfo.m_raycastInfo.m_contactPointWS,
                              btScalar(0.), m_axle[i],m_sideImpulse[i],timeStep);

                    //calculate the slip angle
                    btVector3 rel_pos1 = wheelInfo.m_raycastInfo.m_contactPointWS - m_chassisBody->getCenterOfMassPosition();
                    btVector3 vel_dir = m_chassisBody->getVelocityInLocalPoint(rel_pos1).normalized();
                    double slip = fabs(acos(fabs(vel_dir.dot(m_forwardWS[i]))));
                    //DLOG(INFO) << "Current slip angle for wheel " << i << " is " << slip*180.0/M_PI << " degrees.";


                    //now calculate the maximum sideways impulse
                    //double dMaxSideImpulse = std::min(fabs(timxeStep*wheelInfo.m_wheelsSuspensionForce*m_dSideFriction),
                     //                                 fabs(timeStep*m_dSlipCoefficient*wheelInfo.m_wheelsSuspensionForce*slip)) /fabs(cos(wheelInfo.m_steering)) ;

                    double D = fabs(wheelInfo.m_wheelsSuspensionForce*m_dSideFriction);
                    double dMaxSideImpulse = timeStep*D*sin(m_dC*atan(m_dB*slip - m_dE*(m_dB*slip - atan(m_dB*slip))));
                    //use a soft max to make this smooth
                    //std::cout << "soft min between " << dMaxSideImpulse << " and " << m_sideImpulse[i] << " is " << SoftMinimum(dMaxSideImpulse*1000,m_sideImpulse[i]*100)/100 << std::endl;
                    //m_sideImpulse[i] = SoftMinimum(fabs(dMaxSideImpulse)*100,fabs(m_sideImpulse[i])*100)/100 * sgn(m_sideImpulse[i]);
                    m_sideImpulse[i] = std::min(fabs(dMaxSideImpulse),fabs(m_sideImpulse[i]))* sgn(m_sideImpulse[i]);
                }
            }
        }

    //btScalar sideFactor = btScalar(1.);
    //btScalar fwdFactor = 0.5;

    //bool sliding = false;
    {
        for (size_t wheel =0;wheel <getNumWheels();wheel++){
            WheelInfo& wheelInfo = m_wheelInfo[wheel];
            class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

            btScalar	rollingFriction = 0.f;

            if (groundObject){
                //bool bDynamic = false;
                btScalar engineImpulse = wheelInfo.m_engineForce* timeStep;

                //calculate the friction force based on whether we are moving or not

                btScalar maxDynamicFrictionImpulse = timeStep * wheelInfo.m_wheelsSuspensionForce * m_dFrictionDynamic;
                btWheelContactPoint contactPt(m_chassisBody,groundObject,wheelInfo.m_raycastInfo.m_contactPointWS,m_forwardWS[wheel],maxDynamicFrictionImpulse);
                btScalar dynamicFrictionImpulse = CalcRollingFriction(contactPt);
                //apply both the engine and the dynamic friction impulses
                rollingFriction = dynamicFrictionImpulse + engineImpulse;


//                if (wheelInfo.m_engineForce != 0.f)
//                {
//                    rollingFriction = wheelInfo.m_engineForce* timeStep;
//                } else
//                {
//                    btScalar defaultRollingFrictionImpulse = 0.f;
//                    btScalar maxImpulse = wheelInfo.m_brake ? wheelInfo.m_brake : defaultRollingFrictionImpulse;
//                    btWheelContactPoint contactPt(m_chassisBody,groundObject,wheelInfo.m_raycastInfo.m_contactPointWS,m_forwardWS[wheel],maxImpulse);
//                    rollingFriction = CalcRollingFriction(contactPt);
//                }
            }

            //switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

            m_forwardImpulse[wheel] = btScalar(0.);
            m_wheelInfo[wheel].m_skidInfo= btScalar(1.);

            if (groundObject){
                m_wheelInfo[wheel].m_skidInfo= btScalar(1.);

                //btScalar maximp = wheelInfo.m_wheelsSuspensionForce * timeStep *  wheelInfo.m_frictionSlip;
                //btScalar maximpSide = maximp;

                //btScalar maximpSquared = maximp * maximpSide;


                m_forwardImpulse[wheel] = rollingFriction;//wheelInfo.m_engineForce* timeStep;

//                btScalar x = (m_forwardImpulse[wheel] ) * fwdFactor;
//                btScalar y = (m_sideImpulse[wheel] ) * sideFactor;

//                btScalar impulseSquared = (x*x + y*y);

//                if (impulseSquared > maximpSquared)
//                {
//                    sliding = true;

//                    btScalar factor = maximp / btSqrt(impulseSquared);

//                    m_wheelInfo[wheel].m_skidInfo *= factor;
//                }
            }
        }
    }

//    if (sliding)
//    {
//        for (int wheel = 0;wheel < getNumWheels(); wheel++)
//        {
//            if (m_sideImpulse[wheel] != btScalar(0.))
//            {
//                if (m_wheelInfo[wheel].m_skidInfo< btScalar(1.))
//                {
//                    m_forwardImpulse[wheel] *=	m_wheelInfo[wheel].m_skidInfo;
//                    m_sideImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
//                }
//            }
//        }
//    }

    // apply the impulses
    {
        for (size_t wheel = 0;wheel<getNumWheels() ; wheel++){
            WheelInfo& wheelInfo = m_wheelInfo[wheel];

            btVector3 rel_pos = wheelInfo.m_raycastInfo.m_contactPointWS -
                    m_chassisBody->getCenterOfMassPosition();

            if (m_forwardImpulse[wheel] != btScalar(0.))
            {
                m_chassisBody->applyImpulse(m_forwardWS[wheel]*(m_forwardImpulse[wheel]),rel_pos);
            }
            if (m_sideImpulse[wheel] != btScalar(0.) ){
                class btRigidBody* groundObject = (class btRigidBody*) m_wheelInfo[wheel].m_raycastInfo.m_groundObject;

                btVector3 rel_pos2 = wheelInfo.m_raycastInfo.m_contactPointWS -
                    groundObject->getCenterOfMassPosition();

                //btVector3 sideImp = m_axle[wheel] * (wheel > 1 ? m_sideImpulse[wheel] / 5 : m_sideImpulse[wheel]);
                btVector3 sideImp = m_axle[wheel] * (m_sideImpulse[wheel]);

                //roll influence hack -- this effectively translates the side force imparted on the wheels
                //up towards the CG of the car to prevent rolling. This should be set to high values (m_rollInfluence ~= 1)
                //for accurate physical simulation
                rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
                m_chassisBody->applyImpulse(sideImp,rel_pos);

                //apply friction impulse on the ground
                groundObject->applyImpulse(-sideImp,rel_pos2);
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
std::pair<btScalar,btScalar> RaycastVehicle::GetSteeringRequiredAndMaxForce(const int nWheelId, const double dPhi, const double dt)
{
    std::pair<btScalar,btScalar> res(0,0);
    WheelInfo& wheelInfo = m_wheelInfo[nWheelId];
    const double dOrigSteering = m_dAckermanSteering;
    //set and update the steering
    SetAckermanSteering(-dPhi);
    UpdateWheelSteeringTransform(nWheelId);

    //now calculate the required force and the maximum force
    class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;
    if(groundObject && nWheelId < m_forwardWS.size() ){
        const btTransform& wheelTrans = getWheelTransformWS( nWheelId );

        btMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
        btVector3 axle = btVector3(
            wheelBasis0[0][m_indexRightAxis],
            wheelBasis0[1][m_indexRightAxis],
            wheelBasis0[2][m_indexRightAxis]);

        const btVector3& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
        btScalar proj = axle.dot(surfNormalWS);
        axle -= surfNormalWS * proj;
        axle = axle.normalize();

        btScalar sideImpulse;
        resolveSingleBilateral(*m_chassisBody, wheelInfo.m_raycastInfo.m_contactPointWS,
                  *groundObject, wheelInfo.m_raycastInfo.m_contactPointWS,
                  btScalar(0.), axle,sideImpulse,dt);

        //now calculate the maximum sideways impulse
        btVector3 rel_pos1 = wheelInfo.m_raycastInfo.m_contactPointWS - m_chassisBody->getCenterOfMassPosition();
        btVector3 vel_dir = m_chassisBody->getVelocityInLocalPoint(rel_pos1).normalized();
        double slip = fabs(acos(fabs(vel_dir.dot(m_forwardWS[nWheelId]))));
        //now calculate the maximum sideways impulse
        //double dMaxSideImpulse = dt*wheelInfo.m_wheelsSuspensionForce*m_dSideFriction + dt*m_dSlipCoefficient*slip/fabs(cos(wheelInfo.m_steering));

        double D = fabs(wheelInfo.m_wheelsSuspensionForce*m_dSideFriction);
        double dMaxSideImpulse = dt*D*sin(m_dC*atan(m_dB*slip - m_dE*(m_dB*slip - atan(m_dB*slip))));

        res.first = sideImpulse*cos(fabs(wheelInfo.m_steering));
        res.second = dMaxSideImpulse;/**cos(fabs(wheelInfo.m_steering))*/;
    }

    //reset the original steering
    SetAckermanSteering(dOrigSteering);
    UpdateWheelSteeringTransform(nWheelId);

    //return the ratio
    return res;
}

/////////////////////////////////////////////////////////////////////////////////////////
btScalar RaycastVehicle::CalculateMaxFrictionImpulse(int wheelnum, btScalar timeStep, bool& bDynamic){
     WheelInfo& wheelInfo = m_wheelInfo[wheelnum];
    btScalar rollingFriction = 0;

    class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

    //calculate the friction force based on whether we are moving or not
    if(groundObject && m_forwardWS.size() != 0 ){
            btScalar maxDynamicFrictionImpulse = timeStep * wheelInfo.m_wheelsSuspensionForce * m_dFrictionDynamic;
            btWheelContactPoint contactPt(m_chassisBody,groundObject,wheelInfo.m_raycastInfo.m_contactPointWS,m_forwardWS[wheelnum],maxDynamicFrictionImpulse);
            btScalar dynamicFrictionImpulse = CalcRollingFriction(contactPt);
            //apply both the engine and the dynamic friction impulses
            rollingFriction = dynamicFrictionImpulse;
            bDynamic = true;
    }
    return rollingFriction;
}

/////////////////////////////////////////////////////////////////////////////////////////
void	RaycastVehicle::debugDraw(btIDebugDraw* debugDrawer)
{
    for (size_t v=0;v<this->getNumWheels();v++){
        btVector3 wheelColor(0,1,1);
        if (getWheelInfo(v).m_raycastInfo.m_isInContact){
            wheelColor.setValue(0,0,1);
        } else{
            wheelColor.setValue(1,0,1);
        }

        btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

        btVector3 axle = btVector3(
            getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
            getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
            getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

        //debug wheels (cylinders)
        debugDrawer->drawLine(wheelPosWS,wheelPosWS+axle,wheelColor);
        debugDrawer->drawLine(wheelPosWS,getWheelInfo(v).m_raycastInfo.m_contactPointWS,wheelColor);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void RaycastVehicle::SetMass(btScalar dMass, btVector3 localInertia)
{
    getRigidBody()->setMassProps(dMass,localInertia);
}

/////////////////////////////////////////////////////////////////////////////////////////
void* DefaultVehicleRaycaster::castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result)
{
//	RayResultCallback& resultCallback;
    btCollisionWorld::ClosestRayResultCallback rayCallback(from,to);

    m_dynamicsWorld->rayTest(from, to, rayCallback);

    if (rayCallback.hasHit()){
        const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
        if (body && body->hasContactResponse()){
            result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
            result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
            result.m_hitNormalInWorld.normalize();
            result.m_distFraction = rayCallback.m_closestHitFraction;
            return (void*)body;
        }
    }
    return 0;
}

