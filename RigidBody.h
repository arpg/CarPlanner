#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Eigen/Eigen"
#include "SE3.h"
#include "CarPlannerCommon.h"

struct State {
    float x; // position
    float v; // velocity
};

struct Derivative {
    float dx; // derivative of position: velocity
    float dv; // derivative of velocity: acceleration
};

class RigidBody {
public:

    RigidBody() {
        m_vG << 0, 0, 9.81;
    }

    void Init(Eigen::Matrix3d inertia, double dMass) {
        m_vI = inertia;

        //the inverse of the inertia tensor
        m_vII = m_vI.inverse();
        m_dMass = dMass;

        m_dTime = -1;

        //reset everything
        m_vV << 0, 0, 0;
        m_vW = m_vPosition = m_vRotation = m_vW;
    }

    void RollBackMotion(double dT) {
        m_vPosition -= (m_vV * dT);
        m_vRotation += (m_vW * dT);
    }

    
    // the forces and moments applied are relative to body coordinates

    void UpdateMotion(Eigen::Vector3d force, Eigen::Vector3d torque, double forceDt = -1, bool bIncludeGravity = true, bool bInBodyCoords = false) {
        //if this is the first time the function is called, get a timestamp and exit
        if (m_dTime == -1) {
            m_dTime = Tic();
            return;
        }




        Eigen::Matrix3d R = fusion::Cart2R(m_vW);
        //if given in body coordinates, rotate them to suit
        if (bInBodyCoords == true) {
            force = R*force;
            torque = R*torque;
        }

        //if gravity is to be included, we have to have it in body coordinates
        if (bIncludeGravity == true) {
            //add the rotated force to the total force acting on the body
            force += R*m_vG;

        }

        //get the time elapsed since the last time this function was called
        double dT = Toc(m_dTime);
        m_dTime = Tic();

        //allow the external caller to enforce dT
        if (forceDt != -1) {
            dT = forceDt;
        }

        //store this for corrections
        m_dLastDt = dT;

        //this is a great reference: http://adg.stanford.edu/aa208/dynamics/eom.html
        //calcualte the 3 accelerations based on the force (in body coordinates)
        Eigen::Vector3d accel = force / m_dMass - m_vW.cross(m_vW);

        /*
        //enfore holonomic constraints by applying sideways forces
        //first we have to rotate velocity into body frame
        Eigen::Vector3d vBodyV = R.transpose()*m_vV;
        
        //calculate the acceleration required to remove this component
        Eigen::Vector3d vNhAccel;
        vNhAccel << 0, vBodyV(1) / dT, 0;
        
        //rotate the NH constraints into body coords
        vNhAccel = R*vNhAccel;
        
        //add them to the total acceleration
        accel += vNhAccel;
        */
        
        //double integrate to get new velocity/position
        m_vV += (accel * dT);
        
        
        
        m_vLastPosition = m_vPosition;
        m_vPosition += (m_vV * dT);

        Eigen::Vector3d waccel = m_vII * torque + m_vII * (m_vW.cross(m_vII * m_vW));
        //double integrate to get new angular velocity/rotation
        m_vW += (waccel * dT);
        m_vLastRotation = m_vRotation;
        m_vRotation += (m_vW * dT);
    }

    /*
    float acceleration(const State &state, float t) {
        const float k = 10;
        const float b = 1;
        return -k * state.x - b * state.v;
    }

    /////////////////////////////////////////////////////////////////////////////////////////

    Derivative evaluate(const State &initial, float dt, const Derivative &d) {
        State state;
        state.x = initial.x + d.dx*dt;
        state.v = initial.v + d.dv*dt;

        Derivative output;
        output.dx = state.v;
        output.dv = acceleration(state, t + dt);
        return output;
    }

    void integrate(State &state, float dt) {
        Derivative a = evaluate(state, 0.0f, Derivative());
        Derivative b = evaluate(state, dt * 0.5f, a);
        Derivative c = evaluate(state, dt * 0.5f, b);
        Derivative d = evaluate(state, dt, c);

        const float dxdt = 1.0f / 6.0f * (a.dx + 2.0f * (b.dx + c.dx) + d.dx);
        const float dvdt = 1.0f / 6.0f * (a.dv + 2.0f * (b.dv + c.dv) + d.dv)

                state.x = state.x + dxdt * dt;
        state.v = state.v + dvdt * dt;
    }
    */

    Eigen::Vector3d GetPosition() {
        return m_vPosition;
    }

    Eigen::Vector3d GetRotation() {
        return m_vRotation;
    }

    void SetPosition(Eigen::Vector3d pos) {
        m_vPosition = pos;
    }

    void SetRotation(Eigen::Vector3d rot) {
        m_vRotation = rot;
    }

    void CorrectPosition(Eigen::Vector3d pos) {
        //augment the velocity vector with this correction
        m_vV = (pos - m_vLastPosition) / m_dLastDt;
        m_vPosition = pos;
    }

    void CorrectRotation(Eigen::Vector3d rot) {
        m_vW = (rot - m_vLastRotation) / m_dLastDt;
        m_vRotation = rot;
    }

private:
    Eigen::Vector3d m_vPosition;
    Eigen::Vector3d m_vLastPosition;
    Eigen::Vector3d m_vRotation;
    Eigen::Vector3d m_vLastRotation;
    Eigen::Vector3d m_vV;
    Eigen::Vector3d m_vW;

    Eigen::Matrix3d m_vI;
    Eigen::Matrix3d m_vII;
    double m_dMass;
    double m_dTime;
    double m_dLastDt;
    Eigen::Vector3d m_vG;
};

#endif // RIGIDBODY_H
