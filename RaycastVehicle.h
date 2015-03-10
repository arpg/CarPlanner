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
#ifndef RAYCASTVEHICLE_H
#define RAYCASTVEHICLE_H

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/Vehicle/btVehicleRaycaster.h"
class btDynamicsWorld;
#include "LinearMath/btAlignedObjectArray.h"
#include "WheelInfo.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"
#include <iostream>
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"


class btVehicleTuning;

/////////////////////////////////////////////////////////////////////////////////////////
struct btWheelContactPoint
{
    btRigidBody* m_body0;
    btRigidBody* m_body1;
    btVector3	m_frictionPositionWorld;
    btVector3	m_frictionDirectionWorld;
    btScalar	m_jacDiagABInv;
    btScalar	m_maxImpulse;


    btWheelContactPoint(btRigidBody* body0,btRigidBody* body1,const btVector3& frictionPosWorld,const btVector3& frictionDirectionWorld, btScalar maxImpulse)
        :m_body0(body0),
        m_body1(body1),
        m_frictionPositionWorld(frictionPosWorld),
        m_frictionDirectionWorld(frictionDirectionWorld),
        m_maxImpulse(maxImpulse)
    {
        btScalar denom0 = body0->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
        btScalar denom1 = body1->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
        btScalar	relaxation = 1.f;
        m_jacDiagABInv = relaxation/(denom0+denom1);
    }
};


///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class RaycastVehicle : public btActionInterface
{

        btAlignedObjectArray<btVector3>	m_forwardWS;
        btAlignedObjectArray<btVector3>	m_axle;
        btAlignedObjectArray<btScalar>	m_forwardImpulse;
        btAlignedObjectArray<btScalar>	m_sideImpulse;

        ///backwards compatibility
        int	m_userConstraintType;
        int	m_userConstraintId;

public:
    class btVehicleTuning
        {
            public:

            btVehicleTuning()
                :m_suspensionStiffness(btScalar(5.88)),
                m_suspensionCompression(btScalar(0.83)),
                m_suspensionDamping(btScalar(0.88)),
                m_maxSuspensionTravelCm(btScalar(500.)),
                m_frictionSlip(btScalar(10.5)),
                m_maxSuspensionForce(btScalar(6000.))
            {
            }
            btScalar	m_suspensionStiffness;
            btScalar	m_suspensionCompression;
            btScalar	m_suspensionDamping;
            btScalar	m_maxSuspensionTravelCm;
            btScalar	m_frictionSlip;
            btScalar	m_maxSuspensionForce;

        };
private:
    btScalar    m_dB, m_dE, m_dC;       //Magic formula params
    btScalar    m_dSlipCoefficient;
    btScalar    m_dSideFriction;
    btScalar    m_dFrictionDynamic;
    btScalar	m_tau;
    btScalar	m_damping;
    btVehicleRaycaster*	m_vehicleRaycaster;
    btContactSolverInfo *m_pSolverInfo;
    btScalar		m_pitchControl;
    btScalar	m_steeringValue;
    btScalar m_currentVehicleSpeedKmHour;
    btAlignedObjectArray<WheelInfo>	m_wheelInfo;
    btScalar m_dAckermanSteering;

    btRigidBody* m_chassisBody;

    int m_indexRightAxis;
    int m_indexUpAxis;
    int	m_indexForwardAxis;

    btScalar m_dTotalGravityForce;

    void defaultInit(const btVehicleTuning& tuning);
    void _SetSteeringValue(btScalar steering,int wheel);
    btScalar _GetSteeringValue(int wheel) const;

public:
    btScalar GetTotalGravityForce() { return m_dTotalGravityForce; }
    RaycastVehicle();
    RaycastVehicle(RaycastVehicle & vehicle);


    //constructor to create a car from an existing rigidbody
    RaycastVehicle(const btVehicleTuning& tuning, btRigidBody* chassis,	btVehicleRaycaster* raycaster , btContactSolverInfo *pSolverInfo);
    virtual ~RaycastVehicle() ;


    ///btActionInterface interface
    virtual void updateAction( btCollisionWorld* collisionWorld, btScalar step)
    {
        (void) collisionWorld;
        updateVehicle(step);
    }


    ///btActionInterface interface
    void debugDraw(btIDebugDraw* debugDrawer);
    const btTransform& getChassisWorldTransform() const;
    btScalar rayCast(WheelInfo& wheel);
    virtual void updateVehicle(btScalar step);
    void resetSuspension();
    void SetAckermanSteering(btScalar steering);
    btScalar GetAckermanSteering(){ return m_dAckermanSteering; }
    void applyEngineForce(btScalar force, int wheel);
    const btTransform&	getWheelTransformWS( int wheelIndex ) const;
    btScalar CalculateMaxFrictionImpulse(int wheelnum, btScalar timeStep, bool& bDynamic);
    void updateWheelTransform(const int nWheelIndex, bool bInterpolatedTransform = true );
//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,btScalar depth);
    WheelInfo& addWheel( const btVector3& connectionPointCS0,
                         const btVector3& wheelDirectionCS0,
                         const btVector3& wheelAxleCS,
                         btScalar suspensionRestLength,
                         btScalar wheelRadius,
                         const btVehicleTuning& tuning,
                         bool isFrontWheel);

    inline size_t getNumWheels() const { return size_t (m_wheelInfo.size()); }
    const WheelInfo& getWheelInfo(int index) const;
    WheelInfo&	getWheelInfo(int index);
    void updateWheelTransformsWS(WheelInfo& wheel , bool interpolatedTransform = true);
    void setBrake(btScalar brake,int wheelIndex);
    void setPitchControl(btScalar pitch) { m_pitchControl = pitch; }
    void updateSuspension();
    virtual void updateFriction(btScalar	timeStep);
    inline btRigidBody* getRigidBody() { return m_chassisBody; }
    const btRigidBody* getRigidBody() const{ return m_chassisBody; }
    inline int	getRightAxis() const { return m_indexRightAxis; }
    inline int getUpAxis() const { return m_indexUpAxis; }
    inline int getForwardAxis() const { return m_indexForwardAxis; }
    ///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
    btScalar	getCurrentSpeedKmHour() const { return m_currentVehicleSpeedKmHour; }
    ///backwards compatibility
    int getUserConstraintType() const { return m_userConstraintType ; }
    void setUserConstraintType(int userConstraintType) { m_userConstraintType = userConstraintType; };
    void setUserConstraintId(int uid) { m_userConstraintId = uid; }
    int getUserConstraintId() const { return m_userConstraintId; }
    inline void SetStaticSideFrictionCoefficient(btScalar coef) { m_dSideFriction = coef; }
    inline void SetDynamicFrictionCoefficient(btScalar coef) { m_dFrictionDynamic = coef; }
    inline void SetSlipCoefficient(btScalar coef) { m_dSlipCoefficient = coef; }
    inline void SetMagicFormulaCoefficients(btScalar B, btScalar C, btScalar E) { m_dB = B; m_dC = C; m_dE = E; }
    void SetMass(btScalar dMass, btVector3 localInertia);
    void UpdateWheelSteeringTransform(const int nWheelIndex);
    std::pair<btScalar, btScalar> GetSteeringRequiredAndMaxForce(const int nWheelId, const double dPhi, const double dt);

    ///Worldspace forward vector
    btVector3 getForwardVector() const
    {
        const btTransform& chassisTrans = getChassisWorldTransform();

        btVector3 forwardW (
              chassisTrans.getBasis()[0][m_indexForwardAxis],
              chassisTrans.getBasis()[1][m_indexForwardAxis],
              chassisTrans.getBasis()[2][m_indexForwardAxis]);

        return forwardW;
    }

    virtual void	setCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
    {
        m_indexRightAxis = rightIndex;
        m_indexUpAxis = upIndex;
        m_indexForwardAxis = forwardIndex;
    }
    btScalar resolveSingleCollision(btRigidBody *body1, btCollisionObject *colObj2, const btVector3 &contactPositionWorld, const btVector3 &contactNormalOnB, const btContactSolverInfo &solverInfo, btScalar distance);
};

class DefaultVehicleRaycaster : public btVehicleRaycaster
{
    btDynamicsWorld*	m_dynamicsWorld;
public:
    DefaultVehicleRaycaster(btDynamicsWorld* world)
        :m_dynamicsWorld(world)
    {
    }

    virtual void* castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result);

};


#endif //RAYCASTVEHICLE_H

