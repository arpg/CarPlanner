/**
 * @Author Subhransu Mishra (subhransu.kumar.mishra@gmail.com),
 *         Corin Sandford (corin.sandford@colorado.edu),
 *         Christoffer Heckman (christoffer.heckman@colorado.edu)
 * @date   July, 2016
 * @brief  cpp file to generate control primitives for bullet car
 *
 * Detailed description of file.
 */

#include <CarPlanner/BulletPrim.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>
#include <SceneGraph/GLWaypoint.h>

using namespace SceneGraph;

BulletPrimConfig::BulletPrimConfig(int iteration_limit,bool inertial_control, double time_interval):
  iteration_limit_(iteration_limit), inertial_control_(inertial_control), time_interval_(time_interval){

}

BulletPrim::BulletPrim(const std::string& sMesh, const std::string& sParamsFile, const BulletPrimConfig& cfg):
  cfg_(cfg){
  const aiScene *pScene = aiImportFile( sMesh.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );

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
  CarParameters::LoadFromFile( sParamsFile, m_mDefaultParameters );

  /// Initialize the planner car
  m_PlanCarModel.Init( pCollisionShape, dMin, dMax, m_mDefaultParameters,  LocalPlanner::GetNumWorldsRequired( OPT_DIM )  );
}


BulletPrim::BulletPrim( btCollisionShape* pCollisionShape, const std::string& sParamsFile , const BulletPrimConfig& cfg):
  cfg_(cfg){

  /// Initialize the car parameters.
  CarParameters::LoadFromFile( sParamsFile, m_mDefaultParameters );

  btVector3 dMin( DBL_MAX, DBL_MAX, DBL_MAX );
  btVector3 dMax( DBL_MIN, DBL_MIN, DBL_MIN );

  /// Initialize the planner car
  m_PlanCarModel.Init( pCollisionShape, dMin, dMax, m_mDefaultParameters,  LocalPlanner::GetNumWorldsRequired( OPT_DIM )  );
}

BulletPrim::~BulletPrim()
{
}

double BulletPrim::getPrim( const Vector4d& axyv_start, const Vector4d& axyv_goal, vector<Vector2d>& prim_ctrl, vector<Vector4d>& prim_path){
  //clear the vectors
  prim_ctrl.clear();
  prim_path.clear();

  //update start waypoints
  Vector6d xyzrpy_start; xyzrpy_start <<axyv_start(1), axyv_start(2),0, 0, 0, axyv_start(0);
  GLWayPoint glwp_start, glwp_goal;

  glwp_start.SetPose(xyzrpy_start);
  glwp_start.SetVelocity(axyv_start(3));

  //update goal waypoint
  Vector6d xyzrpy_goal;  xyzrpy_goal  <<axyv_goal(1),  axyv_goal(2), 0, 0, 0, axyv_goal(0);
  glwp_goal.SetPose(xyzrpy_goal);
  glwp_goal.SetVelocity(axyv_goal(3));

  // Make sure both waypoints have well defined poses (no nans)
  if( std::isfinite(glwp_start.GetPose5d().norm()) == false || std::isfinite(glwp_goal.GetPose5d().norm()) == false ) {
    std::cout << "Waypoint poses not well defined." << std::endl;
    assert("Waypoint is not well defined" && 0);
    return -1; // invalid cost
  }

  // Clean the waypoints
  Vector3d dIntersect;
  Sophus::SE3d pose_start( glwp_start.GetPose4x4_po() );
  if( m_PlanCarModel.RayCast( pose_start.translation(), GetBasisVector(pose_start,2)*0.2, dIntersect, true) ){
    pose_start.translation() = dIntersect;
    glwp_start.SetPose( pose_start.matrix() );
  }else{
    cout << "Couldn't ray cast the starting waypoint"<<endl;
    assert("problem with raycasting starting waypoint" && 0);
    return -1; // invalid cost
  }

  Sophus::SE3d pose_goal( glwp_goal.GetPose4x4_po() );
  if( m_PlanCarModel.RayCast( pose_goal.translation(), GetBasisVector(pose_goal,2)*0.2, dIntersect, true) ){
    pose_goal.translation() = dIntersect;
    glwp_goal.SetPose( pose_goal.matrix() );
  }else{
    cout << "Couldn't ray cast the ending waypoint"<<endl;
    assert("problem with raycasting goal waypoint" && 0);
    return -1; // invalid cost
  }

  // Change the GLWaypoint to Vehicle State for the boundary value problem
  VehicleState vs_start( Sophus::SE3d( glwp_start.GetPose4x4_po() ), glwp_start.GetVelocity(), 0 );
  VehicleState vs_goal( Sophus::SE3d( glwp_goal.GetPose4x4_po() ), glwp_goal.GetVelocity(), 0 );


  /// Not sure what this is doing
  ApplyVelocitesFunctor5d f( &m_PlanCarModel, Eigen::Vector3d::Zero(), NULL );
  f.SetNoDelay( true );

  /// Define and initialize the local boundary value problem
  LocalProblem local_problem( &f, vs_start, vs_goal, cfg_.time_interval_);
  m_Planner.InitializeLocalProblem( local_problem, 0, NULL, eCostPoint );
  local_problem.m_bInertialControlActive = cfg_.inertial_control_;

  // Iterate the planner until the max number of iterations is reached or until the path is successfully planned
  bool success = false;
  bool maxiter_reached = false;
  int numIterations = 0;

  MotionSample m_SegmentSample; // struct containing state vectors and command vectors along the planned path
  while ( !(success || maxiter_reached)  ) {
    Eigen::Vector3dAlignedVec vActualTrajectory, vControlTrajectory;
    m_SegmentSample.Clear();
    success = m_Planner.Iterate( local_problem );
    m_Planner.SimulateTrajectory( m_SegmentSample, local_problem, 0, true );
    numIterations++;

    if( numIterations > cfg_.iteration_limit_ )
      maxiter_reached = true;
  }

  //update primitives and the total cost
  if ( maxiter_reached == true ) {
    std::cout << "Max iteration reached." << std::endl;
    return -1; // invalid cost
  } else {
    //update the controls
    prim_ctrl.resize(m_SegmentSample.m_vCommands.size());
    for ( int i = 0; i < prim_ctrl.size(); i++ )
      prim_ctrl[i] << m_SegmentSample.m_vCommands[i].m_dForce*m_SegmentSample.m_vCommands[i].m_dT,m_SegmentSample.m_vCommands[i].m_dPhi;

    //update the path
    prim_path.resize(m_SegmentSample.m_vStates.size());
    for ( int i = 0; i < prim_path.size(); i++ ){
      prim_path[i] << m_SegmentSample.m_vStates[i].ToPose()[3],
                      m_SegmentSample.m_vStates[i].ToPose()[0],
                      m_SegmentSample.m_vStates[i].ToPose()[1],
                      m_SegmentSample.m_vStates[i].ToPose()[5];
    }

    //return cost
    return controlCommandsToCost( m_SegmentSample.m_vCommands );
  }

}

double BulletPrim::controlCommandsToCost(const vector<ControlCommand>& commands){
  double cost(0);

  //some heuristics to convert commands to cost based on force/steering/time/all etc
  std::for_each(commands.begin(), commands.end(), [&](const ControlCommand &c){ cost+=c.m_dForce* c.m_dT; });

  return cost;
}
