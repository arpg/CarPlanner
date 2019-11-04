/*
 * File:   BulletCarModel.h
 * Author: nima
 *
 * Created on May 7, 2012, 1:13 PM
 */

#ifndef BULLETCARMODEL_H
#define	BULLETCARMODEL_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <carplanner_msgs/VehicleState.h>
#include <carplanner_msgs/Command.h>
#include <mesh_msgs/TriangleMeshStamped.h>

#include "/home/mike/code/MochaGui_ros/conversion_tools.h"
//#include "/home/ohrad/code/mochagui/conversion_tools.h"
// #include <mochagui/conversion_tools.h>

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
// #include "ExampleBrowser/CollisionShape2TriangleMesh.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"

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
#include <exception>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <google/protobuf/message.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>


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

    void UpdateWheels(const std::vector<Sophus::SE3d>& vWheelTransforms){
        Sophus::SE3d bodyT = m_dTwv;
        m_vWheelContacts.resize(4);
        m_vWheelStates.resize(4);
        for(size_t ii = 0 ; ii < m_vWheelContacts.size() ; ii++){
            //position the wheels with respect to the body
            fflush(stdout);
            Sophus::SE3d T = bodyT* vWheelTransforms[ii];
            m_vWheelStates[ii] = T;
        }
    }

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
    static Eigen::Vector6d VehicleStateToXYZTCV(const VehicleState& state //< The given state to construct the pose from
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

    /// Uses the VehicleStateToXYZTCV function to convert the
    Eigen::Vector6d ToXYZTCV(){
        return VehicleStateToXYZTCV(*this);
    }

    // x y z roll pitch yaw
    static Eigen::Vector6d VehicleStateToXYZRPY(const VehicleState& state)
    {
        Eigen::Vector7d poseTmp = VehicleStateToXYZQuat(state);
        // Eigen::Quaterniond quatTmp(poseTmp[3],poseTmp[4],poseTmp[5],poseTmp[6]);
        Eigen::Quaterniond quatTmp = state.m_dTwv.unit_quaternion();
        // Eigen::Vector3d yprTmp = quatTmp.toYawPitchRoll();
        auto rpyTmp = quatTmp.toRotationMatrix().eulerAngles(0, 1, 2);
        Eigen::Vector6d poseOut;
        poseOut << state.m_dTwv.translation()[0],
                   state.m_dTwv.translation()[1],
                   state.m_dTwv.translation()[2],
                   rpyTmp[0],
                   rpyTmp[1],
                   rpyTmp[2];
        return poseOut;
    }

    Eigen::Vector6d ToXYZRPY(){
        return VehicleStateToXYZRPY(*this);
    }

    // x y z qx qy qz qw
    static Eigen::Vector7d VehicleStateToXYZQuat(const VehicleState& state)
    {
        Eigen::Vector7d poseOut;
        poseOut << state.m_dTwv.translation()[0],
                   state.m_dTwv.translation()[1],
                   state.m_dTwv.translation()[2],
                   state.m_dTwv.unit_quaternion().x(),
                   state.m_dTwv.unit_quaternion().y(),
                   state.m_dTwv.unit_quaternion().z(),
                   state.m_dTwv.unit_quaternion().w();
        return poseOut;
    }

    Eigen::Vector7d ToXYZQuat(){
        return VehicleStateToXYZQuat(*this);
    }

    static Sophus::SE3d VehicleStateToSE3d(const VehicleState& state)
    {
        Sophus::SE3d poseOut;
        Eigen::Vector7d state_vec = VehicleStateToXYZQuat(state);
        poseOut.translation() = *(new Eigen::Vector3d(state_vec[0], state_vec[1], state_vec[2]));
        poseOut.setQuaternion(Eigen::Quaterniond(state_vec[6], state_vec[3], state_vec[4], state_vec[5]));
        return poseOut;
    }

    Sophus::SE3d ToSE3d()
    {
        return VehicleStateToSE3d(*this);
    }

    double GetNormalVelocity(const VehicleState& state)
    {
      return state.m_dV.norm();
    }

    double GetNormalVelocity()
    {
      return GetNormalVelocity(*this);
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

    Eigen::Vector7d GetPose() const
    {
      Sophus::SE3d state = m_dTwv;
      Eigen::Vector3d trans = state.translation();
      Eigen::Quaterniond quat = state.unit_quaternion();

      Eigen::Vector7d pose;
      pose << trans[0],
              trans[1],
              trans[2],
              quat.x(),
              quat.y(),
              quat.z(),
              quat.w();

      return pose;
    }

    std::vector<Eigen::Vector7d> GetWheelPoses() const
    {
      std::vector<Eigen::Vector7d> poses;

      for(uint ii=0; ii<m_vWheelStates.size(); ii++)
      {
        Sophus::SE3d state = m_vWheelStates[ii];
        Eigen::Vector3d trans = state.translation();
        Eigen::Quaterniond quat = state.unit_quaternion();

        Eigen::Vector7d pose;
        pose << trans[0],
                trans[1],
                trans[2],
                quat.x(),
                quat.y(),
                quat.z(),
                quat.w();

        poses.push_back(pose);
      }

      return poses;
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

    double GetCurvature() const
    {
       return m_dCurvature;
    }

    double GetSteering() const
    {
       return m_dSteering;
    }

    carplanner_msgs::VehicleState toROS() const
    {
      Sophus::SE3d rot_180_y(Eigen::Quaterniond(0,0,1,0),Eigen::Vector3d(0,0,0)); // Quat(w,x,y,z) , Vec(x,y,z)
      Sophus::SE3d rot_180_x(Eigen::Quaterniond(0,1,0,0),Eigen::Vector3d(0,0,0));

      carplanner_msgs::VehicleState state_msg;
      // state_msg.header.stamp.sec = (*this).GetTime();
      // state_msg.header.frame_id = "world";

      Sophus::SE3d Twv = rot_180_x*(*this).m_dTwv*rot_180_x;
      // Sophus::SE3d Twv = (*this).m_dTwv;
      state_msg.pose.header.stamp.sec = (*this).GetTime();
      state_msg.pose.header.frame_id = "world";
      state_msg.pose.child_frame_id = "base_link";
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
          tf.header.frame_id = "world";
          tf.child_frame_id = "wheel_link" + std::to_string(i);

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

      std::vector<bool> wheel_contacts = (*this).GetWheelContacts();
      for(uint ii=0; ii<wheel_contacts.size(); ii++)
      {
        bool contact = wheel_contacts[ii];
        state_msg.wheel_contacts.push_back(contact);
      }

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
        m_bEnableGUI = false;
    }

    ~BulletWorldInstance()
    {
        if( m_pTerrainShape != NULL) {
            delete m_pTerrainShape;
        }
    }

    void enableGUI(bool do_enable_gui)
    {
      m_bEnableGUI = do_enable_gui;
    }

    std::vector<Sophus::SE3d> m_vWheelTransforms;
    double m_dTime;

    bool m_bEnableGUI;

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

    bool m_bEnableROS;
    ros::NodeHandle* m_nh;
    tf::TransformBroadcaster m_tfbr;
    tf::TransformListener m_tflistener;
    ros::Publisher m_statePub;
    ros::Publisher m_meshPub;
    ros::Subscriber m_meshSub;
    bool have_rxed_first_mesh;
    // ros::Subscriber m_meshSub2;
    // std::string m_meshSubTopic = "/infinitam/mesh";
    // std::string m_meshSubTopic = "/fake_mesh_publisher/mesh";
    // ros::Publisher m_posePub;
    // ros::Subscriber m_commandSub;

    // GUIHelperInterface* m_guiHelper;

    void InitROS();
    // void _PublisherFunc();
    // void _StatePublisherFunc();
    void _MeshPublisherFunc();
    // void _pubState();
    // void _pubState(VehicleState&);
    void _pubMesh();
    void _pubMesh(btCollisionShape*);
    void _pubMesh(btCollisionShape*, btTransform*);
    void _meshCB(const mesh_msgs::TriangleMeshStamped::ConstPtr&);
    // void _PoseThreadFunc();
    // void _CommandThreadFunc(const carplanner_msgs::Command::ConstPtr&);

    static btVector3 GetUpVector(int upAxis,btScalar regularValue,btScalar upValue);
    /////////////////////////////////////////////////////////////////////////////////////////
    static void GenerateStaticHull(const struct aiScene *pAIScene, const struct aiNode *pAINode, const aiMatrix4x4 parentTransform, const float flScale, btTriangleMesh &triangleMesh , btVector3& dMin, btVector3& dMax);
    void Init(btCollisionShape *pCollisionShape, const btVector3 &dMin, const btVector3 &dMax, CarParameterMap &parameters, unsigned int numWorlds, bool real=false , bool enableROS=false);
    void Init(const struct aiScene *pAIScene,CarParameterMap& parameters, unsigned int numWorlds, bool real=false , bool enableROS=false);
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
    btRigidBody* _LocalAddRigidBody(BulletWorldInstance *pWorld, double mass, const btTransform& startTransform, btCollisionShape* shape, short group, short mask);
    // btRigidBody* _LocalAddRigidBody(BulletWorldInstance *pWorld, double mass, const btTransform& startTransform, btCollisionShape* shape);
    void _LocalRemoveRigidBody(BulletWorldInstance *, btCollisionShape* );
    void _InitVehicle(BulletWorldInstance* pWorld, CarParameterMap& parameters);
    void _InitWorld(BulletWorldInstance* pWorld, btCollisionShape *pGroundShape, btVector3 dMin, btVector3 dMax, bool centerMesh);

    std::vector<BulletWorldInstance*> m_vWorlds;
    //HeightMap *m_pHeightMap;
    // boost::thread* m_pPoseThread;
    // boost::thread* m_pCommandThread;
    // boost::thread* m_pPublisherThread;
    boost::thread* m_pStatePublisherThread;
    boost::thread* m_pMeshPublisherThread;

    Eigen::Vector3d m_dGravity;
    unsigned int m_nNumWorlds;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


#endif	/* BULLETCARMODEL_H */
