﻿/**
 * @Author Subhransu Mishra (subhransu.kumar.mishra@gmail.com),
 *         Corin Sandford (corin.sandford@colorado.edu),
 *         Christoffer Heckman (christoffer.heckman@colorado.edu)
 * @date   July, 2016
 * @brief  cpp file to generate control primitives for bullet car
 *
 * Detailed description of file.
 */

#include "CarPlanner/BulletPrim.h"
#include <assimp/postprocess.h>
#include <assimp/cimport.h>
#include <boost/filesystem.hpp>

using boost::filesystem::path;
using boost::filesystem::absolute;
using boost::filesystem::exists;

BulletPrimConfig::BulletPrimConfig(int iteration_limit,bool inertial_control, double time_interval):
      iteration_limit_(iteration_limit), inertial_control_(inertial_control), time_interval_(time_interval){
}

BulletPrim::BulletPrim():initialized(false){};

BulletPrim::BulletPrim(const BulletPrim& bp){};

BulletPrim::BulletPrim(const std::string& sMeshFile, const std::string& sParamsFile, const BulletPrimConfig& cfg):initialized(false){
  init(sMeshFile, sParamsFile, cfg);
}

void BulletPrim::init(const std::string& sMeshFile, const std::string& sParamsFile, const BulletPrimConfig& cfg){
  cfg_ = cfg;
  //Check if the file exists and the right type
  path path_mesh(absolute(path(sMeshFile)));
  path path_params(absolute(path(sParamsFile)));

   bool file_probs = false;
   if(!exists(path_mesh)){
     file_probs = true;
     cout<<"path_mesh doesn't exist"<<endl;
   }else if(path_mesh.extension().string().compare(".ply") ){
     file_probs = true;
     cout<<"path_mesh has the wrong extension:"<<endl;
   }

   if(!exists(path_params)){
     file_probs = true;
     cout<<"path_params doesn't exist"<<endl;
   }else if(path_params.extension().string().compare(".csv") ){
     file_probs = true;
     cout<<"path_params has the wrong extension:"<<endl;
   }

   if(file_probs)
     return;

  const aiScene *pScene = aiImportFile( path_mesh.string().c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );

  pScene->mRootNode->mTransformation = aiMatrix4x4( 1, 0, 0, 0,
                                                    0, 1, 0, 0,
                                                    0, 0,-1, 0,
                                                    0, 0, 0, 1 );

  btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
  btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);

  btTriangleMesh *pTriangleMesh = new btTriangleMesh();

  /// Using pTriangleMesh and the terrain mesh, fill in the gaps to create a static hull.
  BulletCarModel::GenerateStaticHull(pScene,pScene->mRootNode,pScene->mRootNode->mTransformation,1.0,*pTriangleMesh,dMin,dMax);

  /// Now generate the collision shape from the triangle mesh --- to know where the ground is.
  btCollisionShape* pCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);

  /// Initialize the car parameters.
  CarParameters::LoadFromFile( path_params.string(), m_mDefaultParameters );

  /// Initialize the planner car
  m_PlanCarModel.Init( pCollisionShape, dMin, dMax, m_mDefaultParameters,  LocalPlanner::GetNumWorldsRequired( OPT_DIM )  );
  //m_PlanCarModel.Init( pCollisionShape, dMin, dMax, m_mDefaultParameters,  1  );

  initialized = true;
}


BulletPrim::~BulletPrim()
{
}

bool BulletPrim::solveBVP(const Vector4d& axyv_start, const Vector4d& axyv_goal, MotionSample& motion_sample){
  if(!initialized)
      return false;

    // Make sure both waypoints have well defined poses (no nans)
    if( std::isfinite(axyv_start.norm()) == false || std::isfinite(axyv_goal.norm()) == false ) {
      std::cout << "Waypoint poses not well defined." << std::endl;
      assert("Waypoint poses not well defined." && 0);
      return false; // invalid cost
    }

    //update start waypoints
    Eigen::Affine3d aff3d_start = Translation3d(axyv_start(1),axyv_start(2),0)*AngleAxisd(axyv_start(0),Vector3d::UnitZ());
    Eigen::Affine3d aff3d_goal  = Translation3d(axyv_goal(1) ,axyv_goal(2) ,0)*AngleAxisd(axyv_goal(0) ,Vector3d::UnitZ());

    // Project the pose on the terrain
    Vector3d dIntersect;
    Sophus::SE3d pose_start( aff3d_start.matrix() );
    if( m_PlanCarModel.RayCast( pose_start.translation(), GetBasisVector(pose_start,2)*0.2, dIntersect, true) ){
      pose_start.translation() = dIntersect;
    }else{
      cout << "Couldn't ray cast the starting waypoint"<<endl;
      assert("problem with raycasting starting waypoint" && 0);
      return false; // invalid cost
    }

    Sophus::SE3d pose_goal( aff3d_goal.matrix() );
    if( m_PlanCarModel.RayCast( pose_goal.translation(), GetBasisVector(pose_goal,2)*0.2, dIntersect, true) ){
      pose_goal.translation() = dIntersect;
    }else{
      cout << "Couldn't ray cast the ending waypoint"<<endl;
      assert("problem with raycasting goal waypoint" && 0);
      return false; // invalid cost
    }

    // Change the GLWaypoint to Vehicle State for the boundary value problem
    VehicleState vs_start( pose_start, axyv_start(3), 0 );
    VehicleState vs_goal( pose_goal, axyv_goal(3), 0 );

    /// Not sure what this is doing
    ApplyVelocitesFunctor5d f( &m_PlanCarModel, Eigen::Vector3d::Zero(), NULL );
    f.SetNoDelay( true );

    /// Define and initialize the local boundary value problem
    ///     //clear the vectors
    LocalProblem local_problem( &f, vs_start, vs_goal, cfg_.time_interval_);
    m_Planner.InitializeLocalProblem( local_problem, 0, NULL, eCostPoint );
    local_problem.m_bInertialControlActive = cfg_.inertial_control_;

    // Iterate the planner until the max number of iterations is reached or until the path is successfully planned
    bool success = false;
    bool maxiter_reached = false;
    int numIterations = 0;

    while ( !(success || maxiter_reached)  ) {
      Eigen::Vector3dAlignedVec vActualTrajectory, vControlTrajectory;
      success = m_Planner.Iterate( local_problem );
      m_Planner.SimulateTrajectory( motion_sample, local_problem, 0, true );
      numIterations++;

      if( numIterations > cfg_.iteration_limit_ )
        maxiter_reached = true;
    }

    //update primitives and the total cost
    if ( maxiter_reached == true ) {
      std::cout << "Max iteration reached." << std::endl;
      return false; // invalid cost
    }
    return true;
}


double BulletPrim::getPrim( const Vector4d& axyv_start, const Vector4d& axyv_goal, vector<Vector2d>& prim_ctrl, vector<Vector4d>& prim_path){

  //clear the vectors
  prim_ctrl.clear();
  prim_path.clear();

  MotionSample motion_sample;
  if(solveBVP(axyv_start,axyv_goal,motion_sample)){
    //update the controls
    prim_ctrl.resize(motion_sample.m_vCommands.size());
    for ( size_t i = 0; i < prim_ctrl.size(); i++ )
      prim_ctrl[i] << motion_sample.m_vCommands[i].m_dForce*motion_sample.m_vCommands[i].m_dT,motion_sample.m_vCommands[i].m_dPhi;

    //update the path
    prim_path.resize(motion_sample.m_vStates.size());
    for (size_t i = 0; i < prim_path.size(); i++ ){
      Vector6d xyztcv = motion_sample.m_vStates[i].ToPose();//x y z theta curvature velocity
      prim_path[i] << xyztcv[3], xyztcv[0], xyztcv[1], xyztcv[5];
    }

    //return cost
        return controlCommandsToCost( motion_sample.m_vCommands );
  }else{
    return -1;
  }

}

double BulletPrim::controlCommandsToCost(const vector<ControlCommand>& commands){
  double cost(0);

  //some heuristics to convert commands to cost based on force/steering/time/all etc
  std::for_each(commands.begin(), commands.end(), [&](const ControlCommand &c){ cost+=c.m_dForce* c.m_dT; });

  return cost;
}
