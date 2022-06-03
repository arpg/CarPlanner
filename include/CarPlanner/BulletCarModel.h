/*
 * File:   BulletCarModel.h
 * Author: nima
 *
 * Created on May 7, 2012, 1:13 PM
 */

#ifndef BULLETCARMODEL_H
#define	BULLETCARMODEL_H

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"

#include "GLDebugDrawer.h"

#include <boost/thread.hpp>
#include <boost/signals2/mutex.hpp>

#include "CarParameters.h"
#include "RaycastVehicle.h"
#include "sophus/se3.hpp"

#include <stdio.h>
#include <chrono>
#include <thread>
#include <string.h>
#include <unistd.h>
#include <HAL/Messages/Command.h>
#include <HAL/Messages/Matrix.h>
#include <HAL/Messages/Pose.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <google/protobuf/message.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

#include <ros/ros.h>
#include <carplanner_msgs/VehicleState.h>
#include <carplanner_msgs/Command.h>
#include <tf/tf.h>

#define CAR_UP_AXIS 2   //this is the index for the bullet Z axis
#define CAR_FORWARD_AXIS 0   //this is the index for the bullet X axis
#define CAR_RIGHT_AXIS 1   //this is the index for the bullet Y axis
#define CUBE_HALF_EXTENTS 1

#define BULLET_MODEL_GRAVITY 9.81

#define MAX_SERVO_ANGLE 45


//collision filtering
#define COL_NOTHING 0
#define COL_RAY 1   //this is the "default" filter that we need to include for raycasting (on the ground only)
#define COL_CAR 2
#define COL_GROUND 4

//vehicle parameter ordering
#define VEHICLE_NUM_PARAMS 5
#define VEHICLE_WIDTH 0.21
#define VEHICLE_WHEEL_BASE 0.27
#define VEHICLE_WHEEL_RADIUS 0.04
#define VEHICLE_WHEEL_WIDTH 0.025
#define MIN_CONTROL_DELAY 0.0
#define MAX_CONTROL_DELAY 0.3


/// Structure to hold the steering, acceleration and reaction wheel caommands that are sent to the vehicle
class ControlCommand
{
public:
    ControlCommand(): m_dForce(0), m_dCurvature(0), m_dT(0),m_dPhi(0), m_dTorque(0,0,0),m_dTime(0)
    {

    }
    ControlCommand(const double& force,const double& curvature,
                    const Eigen::Vector3d& torques, const double& dt, const double& dPhi){

        m_dForce = force;
        //m_dControlAccel = accel;
        m_dCurvature = curvature;
        m_dTorque = torques;
        m_dT = dt;
        m_dPhi = dPhi;
    }

    carplanner_msgs::Command toROS()
    {
        carplanner_msgs::Command msg;

        msg.force = m_dForce;
        msg.curvature = m_dCurvature;
        msg.dt = m_dT;
        msg.dphi = m_dPhi;
        // msg.torques.clear();
        // msg.torques.push_back(m_dTorque[0]);
        // msg.torques.push_back(m_dTorque[1]);
        // msg.torques.push_back(m_dTorque[2]);
        msg.torques[0] = m_dTorque[0];
        msg.torques[1] = m_dTorque[1];
        msg.torques[2] = m_dTorque[2];
        msg.time = m_dTime;

        return msg;
    }

    void fromROS(const carplanner_msgs::Command& msg)
    {
        m_dForce = msg.force;
        m_dCurvature = msg.curvature;
        m_dT = msg.dt;
        m_dPhi = msg.dphi;
        assert(msg.torques.size() <= m_dTorque.size());
        for (uint i=0; i<msg.torques.size(); i++)
        {
            m_dTorque[i] = msg.torques[i];
        }
        m_dTime = msg.time;

        return;
    }

    //double m_dControlAccel;
    double m_dForce;
    double m_dCurvature;
    double m_dT;
    double m_dPhi;
    Eigen::Vector3d m_dTorque;
    double m_dTime;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

/// List holding the previous commands sent to the model (Newest commands at the front, oldest at the back)
typedef std::list<ControlCommand> CommandList;

struct VehicleState
{
    VehicleState()
    {
        m_dTwv = Sophus::SE3d();
        m_dV.setZero();
        m_dW.setZero();
        m_dSteering = 0;
        m_dCurvature = 0 ;
        m_dTime = 0;
    }

    VehicleState(const Sophus::SE3d& dTwv, const double dV, double dCurvature = 0)
    {
        m_dCurvature = dCurvature;
        m_dTwv = dTwv;
        Eigen::Vector3d vecV;
        vecV << dV,0,0;
        m_dV = m_dTwv.so3()*vecV;
        m_dW << 0,0,0;
        m_dSteering = 0;
    }

    bool IsWheelInContact(uint i) const
    {
        return m_vWheelContacts[i];
    }

    std::vector<bool> GetWheelContacts() const
    {
      std::vector<bool> contacts;

      for(uint ii=0; ii<m_vWheelContacts.size(); ii++)
      {
        bool contact = m_vWheelContacts[ii];
        contacts.push_back(contact);
      }

      return contacts;
    }

    // bool IsChassisInCollision() const
    // {
    //     return m_bChassisInCollision;
    // }

    double GetCurvature() const
    {
       return m_dCurvature;
    }

    double GetSteering() const
    {
       return m_dSteering;
    }

    double GetTime() const
    {
       return m_dTime;
    }

    Eigen::Vector6d GetVels() const
    {
      Eigen::Vector6d vels;
      vels << (*this).m_dV[0],
              (*this).m_dV[1],
              (*this).m_dV[2],
              (*this).m_dW[0],
              (*this).m_dW[1],
              (*this).m_dW[2];
      return vels;
    }

    VehicleState FlipCoordFrame()
    {
        Sophus::SE3d rot_180_x(Eigen::Quaterniond(0,1,0,0),Eigen::Vector3d(0,0,0));
        m_dTwv = rot_180_x * (*this).m_dTwv * rot_180_x;

        // (*this).m_dV = (*this).m_dTwv.so3() * (*this).m_dV;
        m_dV = (rot_180_x * Sophus::SE3d(Eigen::Quaterniond(1,0,0,0), (*this).m_dV) * rot_180_x).translation();

        return *this;
    }

    carplanner_msgs::VehicleState toROS(std::string map_frame="map", std::string base_link_frame="base_link", std::string wheel_link_frame="wheel_link") const
    {
    //   Sophus::SE3d rot_180_y(Eigen::Quaterniond(0,0,1,0),Eigen::Vector3d(0,0,0)); // Quat(w,x,y,z) , Vec(x,y,z)
    //   Sophus::SE3d rot_180_x(Eigen::Quaterniond(0,1,0,0),Eigen::Vector3d(0,0,0));

      carplanner_msgs::VehicleState state_msg;
      // state_msg.header.stamp.sec = (*this).GetTime();
      // state_msg.header.frame_id = "world";

    //   Sophus::SE3d Twv = rot_180_x*(*this).m_dTwv*rot_180_x;
      Sophus::SE3d Twv = (*this).m_dTwv;
    //   state_msg.time = (*this).GetTime();
      double time = (*this).GetTime();
      state_msg.header.stamp.sec = floor(time);
      state_msg.header.stamp.nsec = (time-state_msg.pose.header.stamp.sec)*1e9;
      state_msg.pose.header.stamp = ros::Time::now();
      state_msg.pose.header.frame_id = map_frame;
      state_msg.pose.child_frame_id = base_link_frame;
      state_msg.pose.transform.translation.x = Twv.translation()[0];
      state_msg.pose.transform.translation.y = Twv.translation()[1];
      state_msg.pose.transform.translation.z = Twv.translation()[2];
      state_msg.pose.transform.rotation.w = Twv.unit_quaternion().w();
      state_msg.pose.transform.rotation.x = Twv.unit_quaternion().x();
      state_msg.pose.transform.rotation.y = Twv.unit_quaternion().y();
      state_msg.pose.transform.rotation.z = Twv.unit_quaternion().z();

      for( unsigned int i=0; i<(*this).m_vWheelStates.size(); i++ )
      {
          geometry_msgs::TransformStamped tf;
          tf.header.stamp = ros::Time::now();
          tf.header.frame_id = map_frame;
          tf.child_frame_id = wheel_link_frame + "/" + std::to_string(i);

          Sophus::SE3d Twv = (*this).m_vWheelStates[i];
          tf.transform.translation.x = Twv.translation()[0];
          tf.transform.translation.y = Twv.translation()[1];
          tf.transform.translation.z = Twv.translation()[2];
          tf.transform.rotation.w = Twv.unit_quaternion().w();
          tf.transform.rotation.x = Twv.unit_quaternion().x();
          tf.transform.rotation.y = Twv.unit_quaternion().y();
          tf.transform.rotation.z = Twv.unit_quaternion().z();

          state_msg.wheel_poses.push_back( tf );
      }

    //   std::vector<bool> wheel_contacts = (*this).GetWheelContacts();
    //   for(uint ii=0; ii<wheel_contacts.size(); ii++)
    //   {
    //     bool contact = wheel_contacts[ii];
    //     state_msg.wheel_contacts.push_back(contact);
    //   }

    //   state_msg.chassis_collision = m_bChassisInCollision;

      Eigen::Vector6d vels = (*this).GetVels();
      state_msg.lin_vel.x = vels[0];
      state_msg.lin_vel.y = vels[1];
      state_msg.lin_vel.z = vels[2];
      state_msg.ang_vel.x = vels[3];
      state_msg.ang_vel.y = vels[4];
      state_msg.ang_vel.z = vels[5];

      state_msg.curvature = (*this).GetCurvature();

      state_msg.steering = (*this).GetSteering();

      return state_msg;
    }

    void fromROS(const carplanner_msgs::VehicleState msg)
    {  
        Eigen::Quaterniond quat(msg.pose.transform.rotation.w, msg.pose.transform.rotation.x, msg.pose.transform.rotation.y, msg.pose.transform.rotation.z);

        if (abs(sqrt(pow(quat.x(),2)+pow(quat.y(),2)+pow(quat.z(),2)+pow(quat.w(),2))-0.f) < 0.01f)
        {
            ROS_WARN("Got zero quaternion in fromROS. Setting to identity...");
            quat.x() = 0.f;
            quat.y() = 0.f;
            quat.z() = 0.f;
            quat.w() = 1.f;        
        }

        (*this).m_dTwv.translation() = Eigen::Vector3d(
            msg.pose.transform.translation.x,
            msg.pose.transform.translation.y,
            msg.pose.transform.translation.z);
        (*this).m_dTwv.setQuaternion(quat);

        // if (msg.wheel_poses.size() != 4)
        //     ResetWheels();
        // else
        {
            for( unsigned int i=0; i<(*this).m_vWheelStates.size(); i++ )
            {
                (*this).m_vWheelStates[i].translation() = Eigen::Vector3d(
                    msg.wheel_poses[i].transform.translation.x,
                    msg.wheel_poses[i].transform.translation.y,
                    msg.wheel_poses[i].transform.translation.z);
                (*this).m_vWheelStates[i].setQuaternion(Eigen::Quaterniond(
                    msg.wheel_poses[i].transform.rotation.w,
                    msg.wheel_poses[i].transform.rotation.x,
                    msg.wheel_poses[i].transform.rotation.y,
                    msg.wheel_poses[i].transform.rotation.z));
            }
        }

        // if (msg.wheel_contacts.size() != 4)
        //     ResetContacts();
        // else
        {
            for(uint i=0; i<m_vWheelContacts.size(); i++)
            {
                m_vWheelContacts[i] = msg.wheel_contacts[i];
            }
        }

        // m_bChassisInCollision = msg.chassis_collision;

        (*this).m_dV[0] = msg.lin_vel.x;
        (*this).m_dV[1] = msg.lin_vel.y;
        (*this).m_dV[2] = msg.lin_vel.z;
        (*this).m_dW[0] = msg.ang_vel.x;
        (*this).m_dW[1] = msg.ang_vel.y;
        (*this).m_dW[2] = msg.ang_vel.z;

        (*this).m_dCurvature = msg.curvature;

        (*this).m_dSteering = msg.steering;

        (*this).m_dTime = msg.header.stamp.sec + (double)msg.header.stamp.nsec*(double)1e-9;
    }

    static VehicleState GetInterpolatedState(const std::vector<VehicleState>& vStates,
                                             const int nStartIndex,
                                             const double& time,
                                             int& nIndex)
    {
        VehicleState stateOut = vStates.back();
        double currTime = vStates.front().m_dTime;
        for( size_t ii = nStartIndex; ii < vStates.size()-1 ; ii++){
            nIndex = ii;
            if(vStates[ii+1].m_dTime >= time){
                double interpolation = (time-currTime)/(vStates[ii+1].m_dTime-currTime);
                const VehicleState& state2 = vStates[ii+1];
                const VehicleState& state1 = vStates[ii];
                stateOut.m_dCurvature = interpolation*state2.m_dCurvature + (1-interpolation)*state1.m_dCurvature;
                stateOut.m_dSteering = interpolation*state2.m_dSteering + (1-interpolation)*state1.m_dSteering;
                stateOut.m_dV = interpolation*state2.m_dV + (1-interpolation)*state1.m_dV;
                stateOut.m_dW = interpolation*state2.m_dW + (1-interpolation)*state1.m_dW;
                Eigen::Vector3d trans = interpolation*state2.m_dTwv.translation() + (1-interpolation)*state1.m_dTwv.translation();
                Eigen::Quaterniond rot = state1.m_dTwv.so3().unit_quaternion().slerp(interpolation,state2.m_dTwv.so3().unit_quaternion());
                stateOut.m_dTwv = Sophus::SE3d(rot,trans);
                break;
            }else{
                currTime = vStates[ii+1].m_dTime;
            }
        }
        return stateOut;
    }

    // void ResetWheels(){
    //     // std::cout << "Resetting wheels..." << std::endl;
    //     std::vector<Sophus::SE3d> vWheelTransformsCS;
    //     vWheelTransformsCS.resize(4);
    //     for (size_t ii=0; ii<vWheelTransformsCS.size(); ii++)
    //     {
    //         // std::cout << "\t" << ii << std::endl;
    //         bool bIsBackWheel = (ii==2 || ii==3);
    //         bool bIsRightWheel = (ii==0 || ii==3);
    //         // std::cout << "\t\t" << (bIsBackWheel?"back":"front") << " " << (bIsRightWheel?"right":"left") << std::endl;
    //         Eigen::Vector3d connectionPointCS(m_dWheelBase, m_dChassisWidth+0.5*m_dWheelWidth, -m_dSuspConnectionHeight);
    //         if (bIsBackWheel)
    //             connectionPointCS[0] *= -1.f;
    //         if (bIsRightWheel)
    //             connectionPointCS[1] *= -1.f;
    //         // std::cout << "\t\t" << connectionPointCS[0] << " " << connectionPointCS[1] << " " << connectionPointCS[2] << std::endl;
    //         // Eigen::Vector4d connectionPointCSTemp; 
    //         // connectionPointCSTemp << connectionPointCS[0], connectionPointCS[1], connectionPointCS[2], 1.f;
    //         // Eigen::Vector4d connectionPointWSTemp = m_dTwv.matrix() * connectionPointCSTemp;
    //         // Eigen::Vector3d connectionPointWS = connectionPointWSTemp.head(3);
    //         // std::cout << "\t\t" << connectionPointWS[0] << " " << connectionPointWS[1] << " " << connectionPointWS[2] << std::endl;
    //         Eigen::Vector3d vWheelDirectionWS = m_dTwv.rotationMatrix() * m_vWheelDirectionCS;
    //         // std::cout << "\t\t" << vWheelDirectionWS[0] << " " << vWheelDirectionWS[1] << " " << vWheelDirectionWS[2] << std::endl;
    //         // Eigen::Vector3d vWheelAxleWS = m_dTwv.rotationMatrix() * m_vWheelAxleCS;
    //         // std::cout << "\t\t" << vWheelAxleWS[0] << " " << vWheelAxleWS[1] << " " << vWheelAxleWS[2] << std::endl;
    //         double raylen = m_dSuspRestLength + m_dWheelRadius;
    //         Eigen::Vector3d rayvector = m_vWheelDirectionCS * raylen;
    //         Eigen::Vector3d vContactPointCS = connectionPointCS + rayvector;
    //         // std::cout << "\t\t" << vContactPointCS[0] << " " << vContactPointCS[1] << " " << vContactPointCS[2] << std::endl;
    //         vWheelTransformsCS[ii].translation() = vContactPointCS;
            
    //         Eigen::Vector3d up = -vWheelDirectionWS;
    //         // Eigen::Vector3d right = vWheelAxleWS;
    //         // Eigen::Vector3d fwd = right.cross(up);

    //         Eigen::AngleAxisd aWheelSteer(m_dSteering, up);
    //         // Eigen::Matrix3d basis;
    //         // basis << fwd[0],right[0],up[0],
    //         //          fwd[1],right[1],up[1],
    //         //          fwd[2],right[2],up[2];
    //         // Eigen::Matrix3d mWheelOrnWS = aWheelSteer.toRotationMatrix() * basis;
    //         Eigen::Quaterniond qWheelOrnCS(aWheelSteer);
    //         // std::cout << "\t\t" << qWheelOrnCS.x() << " " << qWheelOrnCS.y() << " " << qWheelOrnCS.z() << " " << qWheelOrnCS.w() << std::endl;
    //         vWheelTransformsCS[ii].setQuaternion(qWheelOrnCS);
    //     }
    //     UpdateWheels(vWheelTransformsCS);
    // }

    // store CS wheel poses as WS states
    void UpdateWheels(const std::vector<Sophus::SE3d>& vWheelTransforms){
        Sophus::SE3d bodyT = m_dTwv;
        m_vWheelContacts.resize(4);
        m_vWheelStates.resize(4);
        for(size_t ii = 0 ; ii < m_vWheelStates.size() ; ii++){
            //position the wheels with respect to the body
            fflush(stdout);
            Sophus::SE3d T = bodyT* vWheelTransforms[ii];
            m_vWheelStates[ii] = T;
        }
    }

    // void ResetContacts(){
    //     std::vector<bool> vWheelContacts;
    //     vWheelContacts.resize(4);
    //     for (size_t ii=0; ii<vWheelContacts.size(); ii++)
    //     {
    //         vWheelContacts[ii] = false;
    //     }
    //     UpdateContacts(vWheelContacts);
    // }

    // void UpdateContacts(const std::vector<bool>& vWheelContacts){
    //     m_vWheelContacts.resize(4);
    //     for(size_t ii = 0 ; ii < vWheelContacts.size() ; ii++)
    //     {
    //         m_vWheelContacts[ii] = vWheelContacts[ii];
    //     }
    // }

    static void AlignWithVelocityVector(Sophus::SE3d& Twv, const Eigen::Vector3d& dV)
    {
        Eigen::Vector3d vel = dV.normalized();
        Eigen::Matrix3d R = Twv.so3().matrix();
        R.block<3,1>(0,0) = vel;
        R.block<3,1>(0,2) = R.block<3,1>(0,0).cross(R.block<3,1>(0,1)).normalized();
        R.block<3,1>(0,1) = R.block<3,1>(0,2).cross(R.block<3,1>(0,0)).normalized();
        Twv.so3() = Sophus::SO3d(R);
    }

    void AlignWithVelocityVector()
    {
        VehicleState::AlignWithVelocityVector(m_dTwv,m_dV);
    }

    /// Checks all ground contact points, and returns true if all wheels are off the ground
    bool IsAirborne() const
    {
        bool ground = false;
        if(m_vWheelContacts.size() == 4 )
        {
            for(unsigned int ii = 0 ; ii < 4 ; ii++){
                if(m_vWheelContacts[ii] == true) {
                    ground = true;
                }
            }
        }
        return !ground;
    }

    /// Uses the VehicleStateToPose function to convert the
    Eigen::Vector6d ToPose(){
        return VehicleStateToPose(*this);
    }

    double GetTheta() const
    {
        //calculate theta
        Eigen::Vector3d down,right,forward, trueForward;
        down << 0,0,1;
        trueForward << 1,0,0;
        forward = GetBasisVector(m_dTwv,0); //get the forward
        return atan2(forward[1],forward[0]);
    }

    /// This function gets a vehicle state and returns a 5d pose vector which is parametrized as
    /// [x,y,t,k,v] where t is the 2d angle, k is the path curvature and v is the velocity
    static Eigen::Vector6d VehicleStateToPose(const VehicleState& state //< The given state to construct the pose from
                                              )
    {
        //Eigen::Vector6d dPose = mvl::T2Cart(state.m_dTwv);
        //angle = dPose[5];
        Eigen::Vector6d poseOut;
        poseOut << state.m_dTwv.translation()[0],
                   state.m_dTwv.translation()[1],
                   state.m_dTwv.translation()[2],
                   state.GetTheta(),
                   state.m_dCurvature,
                   state.m_dV.norm();
        return poseOut;
    }



    Sophus::SE3d m_dTwv;                     //< 4x4 matrix denoting the state of the car
    std::vector<Sophus::SE3d> m_vWheelStates;   //< 4x4 matrices which denote the pose of each wheel
    std::vector<bool> m_vWheelContacts;         //< Angular velocity of the vehicle in world coordinates

    Eigen::Vector3d m_dV;                       //< Linear velocity of the vehicle in world coordinates
    Eigen::Vector3d m_dW;                       //< Angular velocity of the vehicle in world coordinates

    double m_dCurvature;                        //< The curvature at this point in the path
    double m_dSteering;                         //< The steering command given to the car at this point (used to reset rate limitations)
    double m_dTime;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class BulletVehicleState
{
public:

    BulletVehicleState() {}
    ~BulletVehicleState() {}

    void LoadState(RaycastVehicle *pVehicle)
    {
        //copy back the data
        *pVehicle = m_pVehicleBuffer;
        memcpy( (void*)pVehicle->getRigidBody(), m_pChassisBuffer,sizeof(RaycastVehicle));
    }

    void SaveState(RaycastVehicle *pVehicle)
    {
        //make a backup of the vhicle
        m_pVehicleBuffer = *pVehicle;
        memcpy(m_pChassisBuffer, (void*)pVehicle->getRigidBody(),sizeof(btRigidBody));
    }

private:
    unsigned char m_pChassisBuffer[sizeof(btRigidBody)];
    RaycastVehicle m_pVehicleBuffer;
};

struct BulletWorldInstance : public boost::mutex
{
    BulletWorldInstance()
    {
        m_pCarChassis = NULL;
        m_pVehicle = NULL;
        m_pTerrainShape = NULL;
        m_pHeightfieldData = NULL;
        m_dTime = -1;
        m_bParametersChanged = false;
        m_dTotalCommandTime = 0;
    }

    ~BulletWorldInstance()
    {
        if( m_pTerrainShape != NULL) {
            delete m_pTerrainShape;
        }
    }

    std::vector<Sophus::SE3d> m_vWheelTransforms;
    double m_dTime;

    btScalar *m_pHeightfieldData;
    btCollisionShape *m_pTerrainShape;
    btRigidBody *m_pTerrainBody;
    RaycastVehicle::btVehicleTuning	m_Tuning;
    btVehicleRaycaster*	m_pVehicleRayCaster;
    RaycastVehicle*	m_pVehicle;
    btCollisionShape* m_pVehicleChassisShape;

    btAlignedObjectArray<btCollisionShape*> m_vCollisionShapes;
    btAlignedObjectArray<btCollisionShape*> m_vVehicleCollisionShapes;
    class btDefaultCollisionConfiguration* m_pCollisionConfiguration;
    class btCollisionDispatcher*	m_pDispatcher;

    class btBroadphaseInterface*	m_pOverlappingPairCache;
    class btConstraintSolver*	m_pConstraintSolver;
    class btDiscreteDynamicsWorld* m_pDynamicsWorld;

    btRigidBody* m_pCarChassis;
    GLDebugDrawer	m_DebugDrawer;
    BulletVehicleState m_vehicleBackup;
    VehicleState m_state;

    CommandList m_lPreviousCommands;    //< List holding the previous commands sent to the model (Newest commands at the front, oldest at the back)
    double m_dTotalCommandTime;

    //static parameters
    CarParameterMap m_Parameters;

    //CarParameters m_Parameters;
    //std::vector<RegressionParameter> m_vLearningParameters;

    int m_nIndex;
    bool m_bParametersChanged;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class BulletCarModel
{
public:
    BulletCarModel();
    ~BulletCarModel();

    void _PoseThreadFunc();
    void _CommandThreadFunc();

    // UDP values
    unsigned m_CarPort;
    unsigned m_LocPort;
    unsigned m_ComPort;
    unsigned m_MochPort;
    struct sockaddr_in carAddr;
    struct sockaddr_in locAddr;
    struct sockaddr_in comAddr;
    struct sockaddr_in mochAddr;
    socklen_t addrLen = sizeof(locAddr);
    int recvLen;
    int comRecvLen;
    int sockFD;
    int comSockFD;
    unsigned char buf[2048];
    unsigned char comBuf[2048];
    unsigned int msgSize = 0;
    unsigned int comMsgSize = 0;
    hal::CommanderMsg* cmd;
    hal::PoseMsg* message;
    hal::VectorMsg* pose;
    hal::MatrixMsg* covar;

    void setTerrainMesh(uint worldId, btCollisionShape* meshShape, tf::StampedTransform& Twm);

    static btVector3 GetUpVector(int upAxis,btScalar regularValue,btScalar upValue);
    /////////////////////////////////////////////////////////////////////////////////////////
    static void GenerateStaticHull(const struct aiScene *pAIScene, const struct aiNode *pAINode, const aiMatrix4x4 parentTransform, const float flScale, btTriangleMesh &triangleMesh , btVector3& dMin, btVector3& dMax);
    void Init(btCollisionShape *pCollisionShape, const btVector3 &dMin, const btVector3 &dMax, CarParameterMap &parameters, unsigned int numWorlds, bool real=false );
    void Init(const struct aiScene *pAIScene,CarParameterMap& parameters, unsigned int numWorlds, bool real=false );
    void DebugDrawWorld(int worldId);

    std::pair<double, double> GetSteeringRequiredAndMaxForce(const int nWorldId, const int nWheelId, const double dPhi, const double dt);
    double GetTotalGravityForce(BulletWorldInstance* pWorld);
    double GetTotalWheelFriction(int worldId, double dt);
    double _CalculateWheelFriction(int wheelNum, BulletWorldInstance* pInstance, double dt);
    /////////////////////////////////////////////////////////////////////////////////////////
    void UpdateStateFromNode();
    void UpdateState(const int &worldId,
                     const ControlCommand command,
                     const double forceDt = -1,
                     const bool bNoDelay = false,
                     const bool bNoUpdate  = false );
    /////////////////////////////////////////////////////////////////////////////////////////
    virtual void GetVehicleState(int worldId, VehicleState &stateOut);
    //virtual VehicleState GetVehicleStateAsync(int worldId);
    Eigen::Vector3d GetVehicleLinearVelocity(int worldId);
    Eigen::Vector3d GetVehicleAngularVelocity(int worldId);
    Eigen::Vector3d GetVehicleInertiaTensor(int worldId);
    virtual void SetState(int nWorldId,  const VehicleState& state );
    //virtual void SetState( int worldId,  const Eigen::Matrix4d& vState  );
    virtual void SetStateNoReset(BulletWorldInstance *pWorld, const Sophus::SE3d &Twv );
    BulletWorldInstance *GetWorldInstance(int id){ return m_vWorlds[id]; }
    const BulletWorldInstance *GetWorldInstance(int id) const { return m_vWorlds[id]; }
    const int GetWorldCount() const { return m_vWorlds.size(); }

    double GetCorrectedSteering(double& dCurvature, int index);
    double  GetSteeringAngle(const double dcurvature, double& dCorrectedCurvature, int index, double steeringCoef /*= 1*/);
    std::vector<Sophus::SE3d> GetWheelTransforms(const int worldIndex);

    //getter functions
    Eigen::Vector3d     GetGravity() { return m_dGravity; }
    //HeightMap*          GetHeightMap() { return m_pHeightMap; }
    void                GetCommandHistory(int worldId,CommandList &previousCommandsOut);
    void                ResetCommandHistory(int worldId);
    CommandList&        GetCommandHistoryRef(int worldId);
    void                PushDelayedControl(int worldId, ControlCommand& delayedCommands);
    CarParameterMap& GetParameters(int index) { return GetWorldInstance(index)->m_Parameters; }
    const CarParameterMap& GetParameters(int index) const  { return GetWorldInstance(index)->m_Parameters; }
    void                SetCommandHistory(const int &worldId, const CommandList &previousCommands);
    bool RayCast(const Eigen::Vector3d& dSource, const Eigen::Vector3d& dRayVector, Eigen::Vector3d& dIntersect, const bool &biDirectional, int index = 0);

    void UpdateParameters(const std::vector<RegressionParameter>& vNewParams);
    void UpdateParameters(const std::vector<RegressionParameter>& vNewParams, BulletWorldInstance *pWorld);
    void UpdateParameters(const std::vector<RegressionParameter>& vNewParams,int index);
    void _InternalUpdateParameters(BulletWorldInstance* pWorld);


protected:

    void _GetDelayedControl(int worldId, double timeDelay, ControlCommand& delayedCommands);
    btRigidBody* _LocalCreateRigidBody(BulletWorldInstance *pWorld, double mass, const btTransform& startTransform, btCollisionShape* shape, short group, short mask);
    void _LocalDestroyRigidBody(BulletWorldInstance *, btCollisionShape* );
    void _InitVehicle(BulletWorldInstance* pWorld, CarParameterMap& parameters);
    void _InitWorld(BulletWorldInstance* pWorld, btCollisionShape *pGroundShape, btVector3 dMin, btVector3 dMax, bool centerMesh);

    std::vector< BulletWorldInstance * > m_vWorlds;
    //HeightMap *m_pHeightMap;
    boost::thread* m_pPoseThread;
    boost::thread* m_pCommandThread;

    Eigen::Vector3d m_dGravity;
    unsigned int m_nNumWorlds;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


#endif	/* BULLETCARMODEL_H */


