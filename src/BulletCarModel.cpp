#include <CarPlanner/CarPlannerCommon.h>
#include <CarPlanner/BulletCarModel.h>
#include "cvars/CVar.h"

BulletCarModel::BulletCarModel()
{
  m_dGravity << 0,0,BULLET_MODEL_GRAVITY;
}

/////////////////////////////////////////////////////////////////////////////////////////
BulletCarModel::~BulletCarModel()
{
  //    close(sockFD);
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::DebugDrawWorld(int worldId)
{
  BulletWorldInstance * pWorld = GetWorldInstance(worldId);
  if( pWorld->m_pDynamicsWorld != NULL ) {
    pWorld->m_pDynamicsWorld->debugDrawWorld();
  	// m_guiHelper->autogenerateGraphicsObjects(pWorld->m_pDynamicsWorld);
  }

}

/////////////////////////////////////////////////////////////////////////////////////////
btVector3 BulletCarModel::GetUpVector(int upAxis,btScalar regularValue,btScalar upValue)
{
  btAssert(upAxis >= 0 && upAxis <= 2 && "bad up axis");

  btVector3 v(regularValue, regularValue, regularValue);
  v[upAxis] = upValue;

  return v;
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::GenerateStaticHull( const struct aiScene *pAIScene, const struct aiNode *pAINode, const aiMatrix4x4 parentTransform, const float flScale, btTriangleMesh &triangleMesh, btVector3& dMin, btVector3& dMax )
{

  aiMesh *pAIMesh;

  aiFace *pAIFace;

  for (size_t x = 0; x < pAINode->mNumMeshes; x++ )
  {
    pAIMesh = pAIScene->mMeshes[pAINode->mMeshes[x]];

    for (size_t y = 0; y < pAIMesh->mNumFaces; y++ )
    {
      pAIFace = &pAIMesh->mFaces[y];

      if ( pAIFace->mNumIndices != 3 )
      {
        continue;
      }


      aiVector3D v1 = parentTransform * pAIMesh->mVertices[pAIFace->mIndices[0]];
      aiVector3D v2 = parentTransform * pAIMesh->mVertices[pAIFace->mIndices[1]];
      aiVector3D v3 = parentTransform * pAIMesh->mVertices[pAIFace->mIndices[2]];

      dMin[0] = std::min((float)dMin[0],std::min(v1.x,std::min(v2.x, v3.x)));
      dMax[0] = std::max((float)dMax[0],std::min(v1.x,std::max(v2.x, v3.x)));

      dMin[1] = std::min((float)dMin[1],std::min(v1.y,std::min(v2.y, v3.y)));
      dMax[1] = std::max((float)dMax[1],std::max(v1.y,std::max(v2.y, v3.y)));

      dMin[2] = std::min((float)dMin[2],std::min(v1.z,std::min(v2.z, v3.z)));
      dMax[2] = std::max((float)dMax[2],std::max(v1.z,std::max(v2.z, v3.z)));

      triangleMesh.addTriangle( btVector3(v1.x * flScale, v1.y * flScale, v1.z * flScale),
      btVector3(v2.x * flScale, v2.y * flScale, v2.z * flScale),
      btVector3(v3.x * flScale, v3.y * flScale, v3.z * flScale),
      false );
    }
  }

  for (size_t x = 0; x < pAINode->mNumChildren; x++ )
  {
    GenerateStaticHull( pAIScene, pAINode->mChildren[x],parentTransform*pAINode->mChildren[x]->mTransformation, flScale, triangleMesh,dMin,dMax );
  }
}


/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::Init(btCollisionShape* pCollisionShape, const btVector3 &dMin, const btVector3 &dMax, CarParameterMap &parameters, unsigned int numWorlds, bool real , bool enableROS)
{
//  if ( real ) {
//    m_poseThreadPub = m_nh.advertise<nav_msgs::Odometry>("pose",1);
//    m_commandThreadSub = m_nh.subscribe<carplanner_msgs::Command>("command", 1, boost::bind(&BulletCarModel::_CommandThreadFunc, this, _1));
//  }

  // reset_mesh_frame = true;

  m_nNumWorlds = numWorlds;
  //initialize a number of worlds
  for(size_t ii = 0 ; ii < m_nNumWorlds ; ii++) {
    BulletWorldInstance *pWorld = new BulletWorldInstance();
    pWorld->m_nIndex = ii;

    _InitWorld(pWorld,pCollisionShape,dMin,dMax,false);

    //initialize the car
    _InitVehicle(pWorld,parameters);

    m_vWorlds.push_back(pWorld);
  }

  m_bEnableROS = enableROS;
  if( m_bEnableROS )
  {
    InitROS();
  }

//  if (real) {
//    m_pPoseThread = new boost::thread( std::bind( &BulletCarModel::_PoseThreadFunc, this ));
//  }
}

///////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::Init(const struct aiScene *pAIScene, CarParameterMap &parameters, unsigned int numWorlds, bool real , bool enableROS)
{
//  if ( real ) {
//    m_poseThreadPub = m_nh.advertise<nav_msgs::Odometry>("pose",1);
//    m_commandThreadSub = m_nh.subscribe<carplanner_msgs::Command>("command", 1, boost::bind(&BulletCarModel::_CommandThreadFunc, this, _1));
//  }

  aiNode *pAINode = pAIScene->mRootNode;

  //generate the triangle mesh
  btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
  btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
  btTriangleMesh *pTriangleMesh = new btTriangleMesh();
  GenerateStaticHull(pAIScene,pAINode,pAINode->mTransformation,1.0,*pTriangleMesh,dMin,dMax);
  btCollisionShape* pCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);

  Init(pCollisionShape, dMin, dMax, parameters, numWorlds, real, enableROS);

  // m_nNumWorlds = numWorlds;
  // //initialize a number of worlds
  // for(size_t ii = 0 ; ii < m_nNumWorlds ; ii++) {
  //   BulletWorldInstance *pWorld = new BulletWorldInstance();
  //   pWorld->m_nIndex = ii;
  //
  //   //initialize the world (heightmap and general physics)
  //   _InitWorld(pWorld,pCollisionShape,dMin,dMax,false);
  //
  //   //initialize the car
  //   _InitVehicle(pWorld,parameters);
  //
  //   m_vWorlds.push_back(pWorld);
  // }
  //
  // m_bEnableROS = enableROS;
  // if( m_bEnableROS )
  // {
  //   InitROS();
  // }

//  if (real) {
//    m_pPoseThread = new boost::thread( std::bind( &BulletCarModel::_PoseThreadFunc, this ));
//  }
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::_InitWorld(BulletWorldInstance* pWorld, btCollisionShape *pGroundShape, btVector3 dMin, btVector3 dMax, bool centerMesh)
{
  //add this to the shapes
  pWorld->m_pTerrainShape = pGroundShape;
  pWorld->m_vCollisionShapes.push_back(pWorld->m_pTerrainShape);
  //pWorld->m_vCollisionShapes.push_back(groundShape);

  pWorld->m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
  pWorld->m_pDispatcher = new btCollisionDispatcher(pWorld->m_pCollisionConfiguration);

  //btVector3 worldMin(pHeightMap->GetMinX(),pHeightMap->GetMinY(),dMin(2)-100);
  //btVector3 worldMax(pHeightMap->GetMaxX(),pHeightMap->GetMaxY(),dMax(2)+100);
  //btVector3 worldMin(-1000,-1000,-1000);
  //btVector3 worldMax(1000,1000,1000);
  pWorld->m_pOverlappingPairCache = new btAxisSweep3(dMin,dMax);
  pWorld->m_pConstraintSolver = new btSequentialImpulseConstraintSolver();
  pWorld->m_pDynamicsWorld = new btDiscreteDynamicsWorld(pWorld->m_pDispatcher,pWorld->m_pOverlappingPairCache,pWorld->m_pConstraintSolver,pWorld->m_pCollisionConfiguration);
  pWorld->m_pDynamicsWorld->setDebugDrawer(&pWorld->m_DebugDrawer);
  pWorld->m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe || btIDebugDraw::DBG_FastWireframe);
  //pWorld->m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawAabb);
  //pWorld->m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_NoDebug);


  //set the gravity vector
  pWorld->m_pDynamicsWorld->setGravity(btVector3(0,0,BULLET_MODEL_GRAVITY));

  btTransform tr;
  tr.setIdentity();
  if(centerMesh == true){
      tr.setOrigin(btVector3((dMax[0] + dMin[0])/2,(dMax[1] + dMin[1])/2,(dMax[2] + dMin[2])/2));
  }

  // if(pWorld->m_pTerrainBody != NULL) {
  //   pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pTerrainBody);
  //   delete pWorld->m_pTerrainBody;
  // }
  pWorld->m_pTerrainBody = _LocalAddRigidBody(pWorld,0,tr,pWorld->m_pTerrainShape,COL_GROUND,COL_RAY|COL_CAR);


  // btAssert((!pWorld->m_pTerrainShape || pWorld->m_pTerrainShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
  // btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
  // btRigidBody::btRigidBodyConstructionInfo cInfo(0.0,myMotionState,pWorld->m_pTerrainShape,btVector3(0,0,0));
  // btRigidBody* body = new btRigidBody(cInfo);
  // body->setContactProcessingThreshold(BT_LARGE_FLOAT);
  // pWorld->m_pTerrainBody = body;
  // pWorld->m_pDynamicsWorld->addRigidBody(body,COL_GROUND,COL_RAY|COL_CAR);

  // pWorld->enableGUI(true);

  //create the ground object
  // _LocalAddRigidBody(pWorld,0,tr,pWorld->m_pTerrainShape,COL_GROUND,COL_RAY|COL_CAR);

  //m_pHeightMap = pHeightMap;


	// m_guiHelper->createPhysicsDebugDrawer(pWorld->m_pDynamicsWorld);
	// m_guiHelper->setUpAxis(2);
	// m_guiHelper->autogenerateGraphicsObjects(pWorld->m_pDynamicsWorld);
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::_InitVehicle(BulletWorldInstance* pWorld, CarParameterMap& parameters)
{
  pWorld->m_Parameters = parameters;

  //delete any previous collision shapes
  for(int ii = 0 ; ii < pWorld->m_vVehicleCollisionShapes.size() ; ii++) {
    delete pWorld->m_vVehicleCollisionShapes[ii];
  }
  pWorld->m_vVehicleCollisionShapes.clear();

  pWorld->m_pVehicleChassisShape = new btBoxShape(btVector3(pWorld->m_Parameters[CarParameters::WheelBase],pWorld->m_Parameters[CarParameters::Width],pWorld->m_Parameters[CarParameters::Height]));
  // pWorld->m_pVehicleChassisShape = new btBoxShape(btVector3(0.5,0.7,0.7)*2);
  pWorld->m_vVehicleCollisionShapes.push_back(pWorld->m_pVehicleChassisShape);

  /*btCompoundShape* compound = new btCompoundShape();
  pWorld->m_vVehicleCollisionShapes.push_back(compound);
  btTransform localTrans;
  localTrans.setIdentity();
  //localTrans effectively shifts the center of mass with respect to the chassis
  localTrans.setOrigin(btVector3(0,0,0));

  compound->addChildShape(localTrans,pWorld->m_pVehicleChassisShape);*/

  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(0,0,0));
  btVector3 vWheelDirectionCS0(0,0,1); //wheel direction is z
  btVector3 vWheelAxleCS(0,1,0); //wheel axle in y direction

  if(pWorld->m_pCarChassis != NULL) {
    pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pCarChassis);
    delete pWorld->m_pCarChassis;
  }

  pWorld->m_pCarChassis = _LocalAddRigidBody(pWorld,pWorld->m_Parameters[CarParameters::Mass],tr,pWorld->m_pVehicleChassisShape, COL_CAR,COL_NOTHING);//chassisShape);
  //pWorld->m_pCarChassis = _LocalAddRigidBody(pWorld,pWorld->m_Parameters.m_dMass,tr,compound, COL_CAR,COL_GROUND);//chassisShape);

  /// create vehicle
  pWorld->m_pVehicleRayCaster = new btDefaultVehicleRaycaster(pWorld->m_pDynamicsWorld);

  if( pWorld->m_pVehicle != NULL ) {
    pWorld->m_pDynamicsWorld->removeVehicle(pWorld->m_pVehicle);
    delete pWorld->m_pVehicle;
  }

  pWorld->m_Tuning.m_frictionSlip = pWorld->m_Parameters[CarParameters::TractionFriction];
  pWorld->m_Tuning.m_suspensionCompression = pWorld->m_Parameters[CarParameters::CompDamping];
  pWorld->m_Tuning.m_suspensionStiffness = pWorld->m_Parameters[CarParameters::Stiffness];
  pWorld->m_Tuning.m_suspensionDamping = pWorld->m_Parameters[CarParameters::ExpDamping];
  pWorld->m_Tuning.m_maxSuspensionForce = pWorld->m_Parameters[CarParameters::MaxSuspForce];
  pWorld->m_Tuning.m_maxSuspensionTravelCm = pWorld->m_Parameters[CarParameters::MaxSuspTravel]*100.0;

  pWorld->m_pVehicle = new RaycastVehicle(pWorld->m_Tuning,pWorld->m_pCarChassis,pWorld->m_pVehicleRayCaster,&pWorld->m_pDynamicsWorld->getSolverInfo());
  pWorld->m_pVehicle->setCoordinateSystem(CAR_RIGHT_AXIS,CAR_UP_AXIS,CAR_FORWARD_AXIS);
  ///never deactivate the vehicle
  pWorld->m_pCarChassis->forceActivationState(DISABLE_DEACTIVATION);
  pWorld->m_pDynamicsWorld->addVehicle(pWorld->m_pVehicle);

  // front right wheel
  bool bIsFrontWheel=true;
  btVector3 connectionPointCS0(pWorld->m_Parameters[CarParameters::WheelBase]/2,pWorld->m_Parameters[CarParameters::Width]/2-(0.3*pWorld->m_Parameters[CarParameters::WheelWidth]), pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
  pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
  // front left wheel
  connectionPointCS0 = btVector3(pWorld->m_Parameters[CarParameters::WheelBase]/2, -pWorld->m_Parameters[CarParameters::Width]/2+(0.3*pWorld->m_Parameters[CarParameters::WheelWidth]),pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
  pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
  // back left wheel
  bIsFrontWheel = false;
  connectionPointCS0 = btVector3(-pWorld->m_Parameters[CarParameters::WheelBase]/2,-pWorld->m_Parameters[CarParameters::Width]/2+(0.3*pWorld->m_Parameters[CarParameters::WheelWidth]), pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
  pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
  // back right wheel
  connectionPointCS0 = btVector3(-pWorld->m_Parameters[CarParameters::WheelBase]/2,pWorld->m_Parameters[CarParameters::Width]/2-(0.3*pWorld->m_Parameters[CarParameters::WheelWidth]), pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
  pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);

  for (size_t i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
  {
    WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(i);
    wheel.m_rollInfluence = pWorld->m_Parameters[CarParameters::RollInfluence];
    Sophus::SE3d wheelTransform(Sophus::SO3d(),
    Eigen::Vector3d(wheel.m_chassisConnectionPointCS[0],wheel.m_chassisConnectionPointCS[1],wheel.m_chassisConnectionPointCS[2] /*+ wheel.getSuspensionRestLength()/2*/));
    pWorld->m_vWheelTransforms.push_back(wheelTransform);
  }

  pWorld->m_pVehicle->SetDynamicFrictionCoefficient(pWorld->m_Parameters[CarParameters::DynamicFrictionCoef]);
  pWorld->m_pVehicle->SetStaticSideFrictionCoefficient(pWorld->m_Parameters[CarParameters::StaticSideFrictionCoef]);
  pWorld->m_pVehicle->SetSlipCoefficient(pWorld->m_Parameters[CarParameters::SlipCoefficient]);
  pWorld->m_pVehicle->SetMagicFormulaCoefficients(pWorld->m_Parameters[CarParameters::MagicFormula_B],
    pWorld->m_Parameters[CarParameters::MagicFormula_C],
    pWorld->m_Parameters[CarParameters::MagicFormula_E]);


  //reset all parameters
  //m_pCarChassis->setCenterOfMassTransform(btTransform::getIdentity());
  pWorld->m_pCarChassis->setLinearVelocity(btVector3(0,0,0));
  pWorld->m_pCarChassis->setAngularVelocity(btVector3(0,0,0));
  pWorld->m_pDynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(pWorld->m_pCarChassis->getBroadphaseHandle(),pWorld->m_pDynamicsWorld->getDispatcher());
  if (pWorld->m_pVehicle)
  {
    pWorld->m_pVehicle->resetSuspension();
    for (size_t i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
    {
      //synchronize the wheels with the (interpolated) chassis worldtransform
      pWorld->m_pVehicle->updateWheelTransform(i,true);
    }
  }


  pWorld->m_vehicleBackup.SaveState(pWorld->m_pVehicle);
}

////////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::InitROS()
{
  m_nh = new ros::NodeHandle("~");
  // m_statePub = m_nh->advertise<carplanner_msgs::VehicleState>("state",1);
  m_terrainMeshPub = m_nh->advertise<mesh_msgs::TriangleMeshStamped>("output_terrain_mesh",1);
  m_chassisMeshPub = m_nh->advertise<mesh_msgs::TriangleMeshStamped>("output_chassis_mesh",1);
  // m_meshSub = m_nh->subscribe<mesh_msgs::TriangleMeshStamped>(m_meshSubTopic, 1, boost::bind(&BulletCarModel::_meshCB, this, _1))
  // m_terrainMeshSub = m_nh->subscribe<mesh_msgs::TriangleMeshStamped>("input_terrain_mesh", 1, boost::bind(&BulletCarModel::_meshCB, this, _1));
  // m_meshSub2 = m_nh->subscribe<mesh_msgs::TriangleMeshStamped>("/fake_mesh_publisher/mesh", 1, boost::bind(&BulletCarModel::_meshCB, this, _1));

  // m_resetmeshSrv = m_nh->advertiseService("reset_mesh", &BulletCarModel::ResetMesh, this);

  m_pPublisherThread = new boost::thread( std::bind( &BulletCarModel::_PublisherFunc, this ));
  // m_pStatePublisherThread = new boost::thread( std::bind( &BulletCarModel::_StatePublisherFunc, this ));
  m_pTerrainMeshPublisherThread = new boost::thread( std::bind( &BulletCarModel::_TerrainMeshPublisherFunc, this ));

}

/////////////////////////////////////////////////////////////////
void BulletCarModel::_pubMesh(btCollisionShape* collisionShape, ros::Publisher* pub)
{
  btTransform* parentTransform = new btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
  _pubMesh(collisionShape, parentTransform, pub);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::_pubMesh(btCollisionShape* collisionShape, btTransform* parentTransform, ros::Publisher* pub)
{
    // ROS_INFO("about to pub mesh with %d faces", dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(collisionShape)->getMeshInterface())->getNumTriangles());

    time_t t0 = std::clock();

    mesh_msgs::TriangleMesh* mesh = new mesh_msgs::TriangleMesh();
    btTransform* transformedToRosParent = new btTransform(btQuaternion(1,0,0,0),btVector3(0,0,0)); // rot 180 x
    (*transformedToRosParent) *= (*parentTransform);
    convertCollisionShape2MeshMsg(collisionShape, transformedToRosParent, &mesh);

    mesh_msgs::TriangleMeshStamped* mesh_stamped = new mesh_msgs::TriangleMeshStamped();
    mesh_stamped->mesh = *mesh;
    mesh_stamped->header.frame_id = "world";
    mesh_stamped->header.stamp = ros::Time::now();

    time_t t1 = std::clock();

    // ROS_INFO("pubbing mesh, %d faces, %d vertices, %.2f sec", mesh->triangles.size(), mesh->vertices.size(), std::difftime(t1,t0)/CLOCKS_PER_SEC); 
    pub->publish(*mesh_stamped);
    ros::spinOnce();
    // ros::Rate(10).sleep();
}

////////////////////////////////////////////////////////////////////////
// bool BulletCarModel::ResetMesh(carplanner_msgs::ResetMesh::Request& req,
//                                carplanner_msgs::ResetMesh::Response& res)
// {
//   // reset_mesh_frame = true; // no longer necessary
//   return true;
// }

////////////////////////////////////////////////////////////////////////////////////////////
// #define CIMM_REPLACE 1
// #define CIMM_APPEND 2
// #define CARPLANNER_INPUT_MESH_MODE CIMM_REPLACE // CIMM_REPLACE, CIMM_APPEND
// void BulletCarModel::_meshCB(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg)
// {
//   static tf::StampedTransform Twm;
//   // static tf2_ros::TransformListener tflistener(m_tfbuffer);

//   try
//   {
//     m_tflistener.waitForTransform("world", "infinitam", ros::Time::now(), ros::Duration(1.0));
//     m_tflistener.lookupTransform("world", "infinitam", ros::Time(0), Twm);
//     // geometry_msgs::TransformStamped Twm_gm = m_tfbuffer.lookupTransform("world", "infinitam", ros::Time(0));
//     // geometry_msg2tf(Twm_gm,&Twm);
//   }
//   catch (tf::TransformException ex)
//   {
//     ROS_ERROR("%s",ex.what());
//     usleep(10000);
//     return;
//   }

//   tf::Transform rot_180_x(tf::Quaternion(1,0,0,0),tf::Vector3(0,0,0));
//   Twm.setData(rot_180_x*Twm/*rot_180_z*/);

//   time_t t0 = std::time(0);

//   btCollisionShape* meshShape;// = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
//   convertMeshMsg2CollisionShape(new mesh_msgs::TriangleMeshStamped(*mesh_msg), &meshShape);
//   // uint new_num_tri = dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(meshShape)->getMeshInterface())->getNumTriangles();

//   // BulletWorldInstance* pWorld = GetWorldInstance(0);
//   // boost::mutex::scoped_lock lock(*pWorld);


//   for (uint i=0; i<m_nNumWorlds; i++)
//   {
//     BulletWorldInstance* pWorld = GetWorldInstance(i);
//     boost::mutex::scoped_lock lock(*pWorld);

//   // #if CARPLANNER_INPUT_MESH_MODE==CIMM_REPLACE
//   //   if(pWorld->m_pTerrainBody != NULL)
//   //   {
//   //     pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pTerrainBody);
//   //   }
//   //   pWorld->m_pTerrainShape = meshShape;
//   //   pWorld->m_pTerrainBody->setCollisionShape(pWorld->m_pTerrainShape);
//   //   pWorld->m_pTerrainBody->setWorldTransform(btTransform(
//   //     btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
//   //     btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
//   //   pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pTerrainBody);

//   // #elif CARPLANNER_INPUT_MESH_MODE==CIMM_APPEND
//     if( pWorld->m_pDynamicsWorld->getCollisionWorld()->getNumCollisionObjects() !< 2 )
//       continue;
//     btTransform tr;
//     tr.setIdentity();
//     btAssert((!meshShape || meshShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
//     btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
//     btRigidBody::btRigidBodyConstructionInfo cInfo(0.0,myMotionState,meshShape,btVector3(0,0,0));
//     btRigidBody* body = new btRigidBody(cInfo);
//     body->setContactProcessingThreshold(BT_LARGE_FLOAT);
//     body->setWorldTransform(btTransform(
//       btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
//       btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
//     pWorld->m_pDynamicsWorld->addRigidBody(body);

//   // #else
//   //   //error
//   // #endif

//   }

//   time_t t1 = std::time(0);
//   ROS_INFO("got mesh, %d faces, %d vertices, %.2f sec", mesh_msg->mesh.triangles.size(), mesh_msg->mesh.vertices.size(), difftime(t1,t0));

// }

void BulletCarModel::replaceMesh(uint worldId, btCollisionShape* meshShape, tf::StampedTransform& Twm)
{
  // btAssert((!meshShape || meshShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

  BulletWorldInstance* pWorld = GetWorldInstance(worldId);
  // boost::unique_lock<boost::mutex> lock(*pWorld);
  pWorld->lock();

  if(pWorld->m_pTerrainBody != NULL)
  {
    pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pTerrainBody);
  }

  pWorld->m_pTerrainShape = meshShape;
  pWorld->m_pTerrainBody->setCollisionShape(pWorld->m_pTerrainShape);
  pWorld->m_pTerrainBody->setWorldTransform(btTransform(
    btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
    btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
  pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pTerrainBody);
  
  // boost::unique_lock<boost::mutex> unlock(*pWorld);
  pWorld->unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::appendMesh(uint worldId, btCollisionShape* meshShape, tf::StampedTransform& Twm)
{
  btAssert((!meshShape || meshShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

  BulletWorldInstance* pWorld = GetWorldInstance(worldId);
  // boost::unique_lock<boost::mutex> lock(*pWorld);
  pWorld->lock();
  
  uint max_num_coll_objs = 3;
  if( pWorld->m_pDynamicsWorld->getCollisionWorld()->getNumCollisionObjects() >= max_num_coll_objs )
  {
    // boost::unique_lock<boost::mutex> unlock(*pWorld);
    pWorld->unlock();
    replaceMesh(worldId, meshShape, Twm);
    return;
  }

  btTransform tr;
  tr.setIdentity();
  btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
  
  btRigidBody::btRigidBodyConstructionInfo cInfo(0.0,myMotionState,meshShape,btVector3(0,0,0));
  btRigidBody* body = new btRigidBody(cInfo);
  body->setContactProcessingThreshold(BT_LARGE_FLOAT);
  body->setWorldTransform(btTransform(
    btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
    btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
  pWorld->m_pTerrainBody = body;
  pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pTerrainBody);

  // boost::unique_lock<boost::mutex> unlock(*pWorld);
  pWorld->unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::_PublisherFunc()
{
  while( ros::ok() )
  {
    BulletWorldInstance* pWorld = GetWorldInstance(0);
    _pubMesh(pWorld->m_pCarChassis->getCollisionShape(), &(pWorld->m_pCarChassis->getWorldTransform()), &m_chassisMeshPub);

    ros::Rate(10).sleep();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::_TerrainMeshPublisherFunc()
{
  while( ros::ok() )
  {
    BulletWorldInstance* pWorld = GetWorldInstance(0);
    time_t t0 = clock();

    _pubMesh(pWorld->m_pTerrainBody->getCollisionShape(), &(pWorld->m_pTerrainBody->getWorldTransform()), &m_terrainMeshPub);
    
    time_t t1 = clock();
    uint num_tri = dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(pWorld->m_pTerrainBody->getCollisionShape())->getMeshInterface())->getNumTriangles();
    ROS_INFO("pubbing mesh, %d triangles, %.2f sec", num_tri, std::difftime(t1,t0)/CLOCKS_PER_SEC); 
     
    // btCollisionObjectArray objarr = pWorld->m_pDynamicsWorld->getCollisionObjectArray();
    // for(uint i=0; i<objarr.size(); i++)
    // {
    //   if(objarr[i] != pWorld->m_pCarChassis)
    //   {
    //     time_t t0 = clock();

    //     _pubMesh(objarr[i]->getCollisionShape(), &(objarr[i]->getWorldTransform()), &m_terrainMeshPub);
    //     
    //     time_t t1 = clock();
    //     uint num_tri = dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(objarr[i]->getCollisionShape())->getMeshInterface())->getNumTriangles();
    //     ROS_INFO("pubbing mesh, %d triangles, %.2f sec", num_tri, std::difftime(t1,t0)/CLOCKS_PER_SEC); 
    //   }
    // }
    
    ros::Rate(100).sleep();
  }
}

//////////////////////////////////////////////s///////////////////////////////////////////
void BulletCarModel::PushDelayedControl(int worldId, ControlCommand& delayedCommands)
{
  BulletWorldInstance* pWorld = GetWorldInstance(worldId);
  //lock the world to prevent changes
  boost::mutex::scoped_lock lock(*pWorld);
  pWorld->m_lPreviousCommands.push_front(delayedCommands);
  //add to the total command time
  pWorld->m_dTotalCommandTime += delayedCommands.m_dT;
  //int s = pWorld->m_lPreviousCommands.size();

  //if this time is over the maximum amount, pop them off at the back
  while(pWorld->m_dTotalCommandTime > MAX_CONTROL_DELAY &&
    (pWorld->m_dTotalCommandTime - pWorld->m_lPreviousCommands.back().m_dT) > MAX_CONTROL_DELAY)
    {
      pWorld->m_dTotalCommandTime -= pWorld->m_lPreviousCommands.back().m_dT;
      pWorld->m_lPreviousCommands.pop_back();
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::_GetDelayedControl(int worldId, double timeDelay, ControlCommand& delayedCommands)
  {
    //clamp the time delay to be > 0
    timeDelay = std::max(timeDelay,0.0);

    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    //lock the world to prevent changes
    boost::mutex::scoped_lock lock(*pWorld);
    CommandList& previousCommands = pWorld->m_lPreviousCommands;
    //if there is control delay, get commands from a previous time
    double currentDelay = 0;
    CommandList::iterator it = previousCommands.begin();
    ControlCommand* pCurrentCommand = &(*it);
    //currentDelay = (*it).m_dT;


    int count = 0;
    if(previousCommands.size() > 1) {
      it++; //move to the first element
      for(; it != previousCommands.end() ; it++) {
        count++;
        if( currentDelay + (*it).m_dT >= timeDelay ) {

          //interpolate between the current and next commands
          double r2 = (timeDelay - currentDelay)/(*it).m_dT;
          double r1 = 1-r2;

          delayedCommands.m_dForce = r1*pCurrentCommand->m_dForce + r2*(*it).m_dForce;
          delayedCommands.m_dCurvature = r1*pCurrentCommand->m_dCurvature + r2*(*it).m_dCurvature;
          delayedCommands.m_dPhi = r1*pCurrentCommand->m_dPhi + r2*(*it).m_dPhi;
          delayedCommands.m_dTorque = r1*pCurrentCommand->m_dTorque + r2*(*it).m_dTorque;

          it++;
          return;
        }else {
          pCurrentCommand = &(*it);
          currentDelay += pCurrentCommand->m_dT;

          if(currentDelay == timeDelay) {
            delayedCommands = *pCurrentCommand;
            return;
          }
        }
      }
    }else if(previousCommands.size() > 0){
      DLOG(INFO) << "Command history list size < 2, using first command.";
      delayedCommands = previousCommands.front();
    }else{
      DLOG(INFO) << "Command history list size == 0. Passing empty command";
      delayedCommands.m_dForce = pWorld->m_Parameters[CarParameters::AccelOffset]*SERVO_RANGE;
      delayedCommands.m_dCurvature = 0;
      delayedCommands.m_dPhi = pWorld->m_Parameters[CarParameters::SteeringOffset]*SERVO_RANGE;
      delayedCommands.m_dTorque << 0,0,0;
    }


  }

  /////////////////////////////////////////////////////////////////////////////////////////
  double BulletCarModel::GetCorrectedSteering(double& dCurvature, int index)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(index);
    double phi = atan(dCurvature*pWorld->m_Parameters[CarParameters::WheelBase]);
    phi = SoftMinimum(pWorld->m_Parameters[CarParameters::MaxSteering],
      SoftMaximum(phi,-pWorld->m_Parameters[CarParameters::MaxSteering],50),50);
      dCurvature  = (tan(phi)/pWorld->m_Parameters[CarParameters::WheelBase]);
    return phi;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  double BulletCarModel::GetSteeringAngle(const double dcurvature, double& dCorrectedCurvature, int index, double steeringCoef /*= 1*/)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(index);
    double phi = atan(dcurvature*pWorld->m_Parameters[CarParameters::WheelBase])*steeringCoef;
    //double phi = 1.0/atan((1-powi(pWorld->m_Parameters[CarParameters::WheelBase]/2,2)*powi(dcurvature,2))/(powi(dcurvature,2)*powi(pWorld->m_Parameters[CarParameters::WheelBase],2)));

    dCorrectedCurvature  = (tan(phi)/pWorld->m_Parameters[CarParameters::WheelBase]);
    //dCorrectedCurvature = 1/sqrt(powi(pWorld->m_Parameters[CarParameters::WheelBase]/2,2) + powi(pWorld->m_Parameters[CarParameters::WheelBase],2)*powi(1.0/tan(phi),2));
    return phi;
  }


  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::UpdateParameters(const std::vector<RegressionParameter>& vNewParams)
  {
    for(size_t ii = 0 ; ii < m_nNumWorlds ; ii++) {
      UpdateParameters(vNewParams,ii);
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::UpdateParameters(const std::vector<RegressionParameter>& vNewParams,int index) {
    UpdateParameters(vNewParams,GetWorldInstance(index));
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::UpdateParameters(const std::vector<RegressionParameter>& vNewParams,BulletWorldInstance* pWorld)
  {
    boost::mutex::scoped_lock lock(*pWorld);
    //update the parameter map and learning parameter list with the new params
    for(size_t ii = 0; ii < vNewParams.size() ; ii++) {
      //pWorld->m_vLearningParameters[ii].m_dVal = vNewParams[ii].m_dVal;
      pWorld->m_Parameters[vNewParams[ii].m_nKey] = vNewParams[ii].m_dVal;
      //DLOG(INFO) << "Updating parameter with key " << vNewParams[ii].m_nKey << " to " << pWorld->m_Parameters[vNewParams[ii].m_nKey];
    }
    _InternalUpdateParameters(pWorld);
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::_InternalUpdateParameters(BulletWorldInstance* pWorld)
  {
    //DLOG(INFO) << "updating parameters to " << pWorld->m_vParameters;

    // boost::mutex::scoped_lock lock(*pWorld);

    pWorld->m_bParametersChanged = true;

    //dyanmic friction is slightly less
    pWorld->m_pVehicle->SetDynamicFrictionCoefficient(pWorld->m_Parameters[CarParameters::DynamicFrictionCoef]);
    //set side friction (for drifting)
    pWorld->m_pVehicle->SetStaticSideFrictionCoefficient(pWorld->m_Parameters[CarParameters::StaticSideFrictionCoef]);
    pWorld->m_pVehicle->SetSlipCoefficient(pWorld->m_Parameters[CarParameters::SlipCoefficient]);
    pWorld->m_pVehicle->SetMagicFormulaCoefficients(pWorld->m_Parameters[CarParameters::MagicFormula_B],
      pWorld->m_Parameters[CarParameters::MagicFormula_C],
      pWorld->m_Parameters[CarParameters::MagicFormula_E]);

    //set the mass and wheelbase of the car
    //pWorld->m_Parameters[CarParameters::WheelBase] = pWorld->m_vParameters[eWheelBase];
    //pWorld->m_Parameters.m_dMass = pWorld->m_vParameters[eMass];

    // btCollisionObjectArray objarr = pWorld->m_pDynamicsWorld->getCollisionObjectArray();
    // printf("%d collision objects before remove\n", objarr.size());

    pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pVehicle->getRigidBody());


    // objarr = pWorld->m_pDynamicsWorld->getCollisionObjectArray();
    // printf("%d collision objects after remove\n", objarr.size());

    //change rigid body dimensions
    btBoxShape *pBoxShape =  (btBoxShape *)pWorld->m_pVehicle->getRigidBody()->getCollisionShape();
    pBoxShape->setImplicitShapeDimensions(btVector3(pWorld->m_Parameters[CarParameters::WheelBase],pWorld->m_Parameters[CarParameters::Width],pWorld->m_Parameters[CarParameters::Height]));
    //calculate new inertia
    btVector3 localInertia(0,0,0);
    pBoxShape->calculateLocalInertia(pWorld->m_Parameters[CarParameters::Mass],localInertia);
    pWorld->m_pVehicle->getRigidBody()->setMassProps(pWorld->m_Parameters[CarParameters::Mass],localInertia);
    pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pVehicle->getRigidBody(),COL_CAR,COL_NOTHING);

    // objarr = pWorld->m_pDynamicsWorld->getCollisionObjectArray();
    // printf("%d collision objects after add\n", objarr.size());

    //change the position of the wheels
    pWorld->m_pVehicle->getWheelInfo(0).m_chassisConnectionPointCS[0] = pWorld->m_Parameters[CarParameters::WheelBase]/2;
    pWorld->m_pVehicle->getWheelInfo(1).m_chassisConnectionPointCS[0] = pWorld->m_Parameters[CarParameters::WheelBase]/2;
    pWorld->m_pVehicle->getWheelInfo(2).m_chassisConnectionPointCS[0] = -pWorld->m_Parameters[CarParameters::WheelBase]/2;
    pWorld->m_pVehicle->getWheelInfo(3).m_chassisConnectionPointCS[0] = -pWorld->m_Parameters[CarParameters::WheelBase]/2;

    for (size_t i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
    {
      WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(i);
      pWorld->m_pVehicle->updateWheelTransformsWS(wheel);
      pWorld->m_pVehicle->updateWheelTransform(i);
      wheel.m_suspensionRestLength1 = pWorld->m_Parameters[CarParameters::SuspRestLength];
      wheel.m_suspensionStiffness = pWorld->m_Parameters[CarParameters::Stiffness];
      wheel.m_wheelsDampingCompression = pWorld->m_Parameters[CarParameters::CompDamping];
      wheel.m_wheelsDampingRelaxation = pWorld->m_Parameters[CarParameters::ExpDamping];
    }

    pWorld->m_pVehicle->updateSuspension();

  }

  /////////////////////////////////////////////////////////////////////////////////////////
  double BulletCarModel::GetTotalWheelFriction(int worldId, double dt)
  {
    double totalForce = 0;
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    for(size_t ii = 0; ii < pWorld->m_pVehicle->getNumWheels() ; ii++) {
      totalForce += _CalculateWheelFriction((ii),pWorld,dt);
    }
    return totalForce;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  std::pair<double,double> BulletCarModel::GetSteeringRequiredAndMaxForce(const int nWorldId, const int nWheelId, const double dPhi, const double dt)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(nWorldId);
    return pWorld->m_pVehicle->GetSteeringRequiredAndMaxForce(nWheelId,dPhi,dt);
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  double BulletCarModel::_CalculateWheelFriction(int wheelNum, BulletWorldInstance* pInstance, double dt)
  {
    bool bDynamic;
    double maxImpulse = pInstance->m_pVehicle->CalculateMaxFrictionImpulse(wheelNum,dt,bDynamic);
    return maxImpulse / dt;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  double BulletCarModel::GetTotalGravityForce(BulletWorldInstance* pWorld)
  {
    btVector3 gravityForce(0,0,-10*pWorld->m_Parameters[CarParameters::Mass]);
    for(size_t ii = 0; ii < pWorld->m_pVehicle->getNumWheels() ; ii++) {
      WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(ii);
      if(wheel.m_raycastInfo.m_isInContact)
      {
        gravityForce -= wheel.m_raycastInfo.m_contactNormalWS*wheel.m_wheelsSuspensionForce;
      }
    }
    //now get the component in the direction of the vehicle
    const btTransform& chassisTrans = pWorld->m_pVehicle->getChassisWorldTransform();
    btVector3 carFwd (
      chassisTrans.getBasis()[0][CAR_FORWARD_AXIS],
      chassisTrans.getBasis()[1][CAR_FORWARD_AXIS],
      chassisTrans.getBasis()[2][CAR_FORWARD_AXIS]);
    double force = gravityForce.dot(carFwd);
    return -force;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::UpdateState(  const int& worldId,
    const ControlCommand command,
    const double forceDt /*= -1*/,
    const bool bNoDelay /* = false */,
    const bool bNoUpdate /* = false */)
  {
    ControlCommand delayedCommands;
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);

    if(pWorld->m_dTime == -1 && forceDt == -1 )   {
      pWorld->m_dTime = Tic();
      return;
    }

    //calculate the time since the last iteration
    double dT = Toc( pWorld->m_dTime );
    pWorld->m_dTime = Tic();

    if( forceDt != -1 ){
      dT = forceDt;
    }

    delayedCommands = command;
    delayedCommands.m_dT = dT;
    if(bNoDelay == false){
      PushDelayedControl(worldId,delayedCommands);
      //get the delayed command
      _GetDelayedControl(worldId, pWorld->m_Parameters[CarParameters::ControlDelay],delayedCommands);
    }
    //get the delayed commands for execution and remove the offsets
    double dCorrectedForce = delayedCommands.m_dForce- pWorld->m_Parameters[CarParameters::AccelOffset]*SERVO_RANGE;
    double dCorrectedPhi = delayedCommands.m_dPhi-pWorld->m_Parameters[CarParameters::SteeringOffset]*SERVO_RANGE;

    //D.C. motor equations:
    //torque = Pwm*Ts - slope*V
    //TODO: make this velocity in the direction of travel
    const double stallTorque = dCorrectedForce*pWorld->m_Parameters[CarParameters::StallTorqueCoef];
    dCorrectedForce = sgn(stallTorque)*std::max(0.0,fabs(stallTorque) -
    pWorld->m_Parameters[CarParameters::TorqueSpeedSlope]*fabs(pWorld->m_state.m_dV.norm()));

    //now apply the offset and scale values to the force and steering commands
    dCorrectedPhi = dCorrectedPhi/(pWorld->m_Parameters[CarParameters::SteeringCoef]*SERVO_RANGE);

    //clamp the steering
    dCorrectedPhi = SoftMinimum(pWorld->m_Parameters[CarParameters::MaxSteering],
      SoftMaximum(dCorrectedPhi,-pWorld->m_Parameters[CarParameters::MaxSteering],10),10);

    //steering needs to be flipped due to the way RayCastVehicle works
    dCorrectedPhi *= -1;

    //rate-limit the steering
    double dCurrentSteering = pWorld->m_pVehicle->GetAckermanSteering();
    double dRate = (dCorrectedPhi-dCurrentSteering)/dT;
    //clamp the rate
    dRate = sgn(dRate) * std::min(fabs(dRate),pWorld->m_Parameters[CarParameters::MaxSteeringRate]);
    //apply the steering
    dCorrectedPhi = dCurrentSteering+dRate*dT;

    double wheelForce = dCorrectedForce/2;

    int wheelIndex = 2;
    pWorld->m_pVehicle->applyEngineForce(wheelForce,wheelIndex);
    wheelIndex = 3;
    pWorld->m_pVehicle->applyEngineForce(wheelForce,wheelIndex);

    wheelIndex = 0;
    //set the steering value
    pWorld->m_pVehicle->SetAckermanSteering(dCorrectedPhi);

    // Simulation mode -> this causes the car to move on the screen
    // Experiment mode + SIL -> this causes the car to go +z forever and spin on it's y-axis (through the length of the car)
    // if bNoUpdate is true, then car does not move in both modes
    // printf("OK 0\n");
    if (pWorld->m_pDynamicsWorld && bNoUpdate==false)
    {
      // boost::mutex::scoped_lock lock(*pWorld);
      Eigen::Vector3d T_w = pWorld->m_state.m_dTwv.so3()*command.m_dTorque;
      btVector3 bTorques( T_w[0], T_w[1], T_w[2] );
      pWorld->m_pVehicle->getRigidBody()->applyTorque( bTorques );
      //DLOG(INFO) << "Sending torque vector " << T_w.transpose() << " to car.";
      // printf("OK 1\n");
      pWorld->m_pDynamicsWorld->stepSimulation(dT,1,dT);
    }
    // printf("OK 2\n");

    //do this in a critical section
    {
      boost::mutex::scoped_lock lock(*pWorld);
      //get chassis data from bullet
      Eigen::Matrix4d Twv;
      pWorld->m_pVehicle->getChassisWorldTransform().getOpenGLMatrix(Twv.data());
      pWorld->m_state.m_dTwv = Sophus::SE3d(Twv);

      if(pWorld->m_state.m_vWheelStates.size() != pWorld->m_pVehicle->getNumWheels()) {
        pWorld->m_state.m_vWheelStates.resize(pWorld->m_pVehicle->getNumWheels());
        pWorld->m_state.m_vWheelContacts.resize(pWorld->m_pVehicle->getNumWheels());
      }

      for(size_t ii = 0; ii < pWorld->m_pVehicle->getNumWheels() ; ii++) {
        //m_pVehicle->updateWheelTransform(ii,true);
        pWorld->m_pVehicle->getWheelInfo(ii).m_worldTransform.getOpenGLMatrix(Twv.data());
        pWorld->m_state.m_vWheelStates[ii] = Sophus::SE3d(Twv);
        pWorld->m_state.m_vWheelContacts[ii] = pWorld->m_pVehicle->getWheelInfo(ii).m_raycastInfo.m_isInContact;
      }

      //get the velocity
      pWorld->m_state.m_dV << pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[0], pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[1], pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[2];
      pWorld->m_state.m_dW << pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[0], pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[1], pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[2];

      //set the steering
      pWorld->m_state.m_dSteering = pWorld->m_pVehicle->GetAckermanSteering();
    }

  }

  /////////////////////////////////////////////////////////////////////////////////////////

  Eigen::Vector3d BulletCarModel::GetVehicleLinearVelocity(int worldId)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    btVector3 v = pWorld->m_pVehicle->getRigidBody()->getLinearVelocity();
    Eigen::Vector3d dV;
    dV << v.x(), v.y(), v.z();
    return dV;
  }

  /////////////////////////////////////////////////////////////////////////////////////////

  Eigen::Vector3d BulletCarModel::GetVehicleAngularVelocity(int worldId)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    btVector3 v = pWorld->m_pVehicle->getRigidBody()->getAngularVelocity();
    Eigen::Vector3d dV;
    dV << v.x(), v.y(), v.z();
    return dV;
  }

  /////////////////////////////////////////////////////////////////////////////////////////

  Eigen::Vector3d BulletCarModel::GetVehicleInertiaTensor(int worldId)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);

    btVector3 bVec = pWorld->m_pVehicle->getRigidBody()->getInvInertiaDiagLocal();
    Eigen::Vector3d res;
    for(int ii = 0 ; ii < 3 ; ii++) {
      res(ii) = (bVec[ii] == 0 ? 0 : 1/bVec[ii]);
    }
    return res;
  }


  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::GetVehicleState(int worldId,VehicleState& stateOut)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    boost::mutex::scoped_lock lock(*pWorld);
    stateOut = pWorld->m_state;
    stateOut.m_dTwv.translation() += GetBasisVector(stateOut.m_dTwv,2)*
    (pWorld->m_Parameters[CarParameters::SuspRestLength] +
      pWorld->m_Parameters[CarParameters::WheelRadius]+
      pWorld->m_Parameters[CarParameters::SuspConnectionHeight]-0.01);
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::SetStateNoReset( BulletWorldInstance *pWorld , const Sophus::SE3d& Twv)
  {
    btTransform trans;
    trans.setFromOpenGLMatrix(Twv.matrix().data());
    pWorld->m_pCarChassis->setAngularVelocity(btVector3(0,0,0));
    pWorld->m_pCarChassis->setLinearVelocity(btVector3(0,0,0));
    pWorld->m_pCarChassis->setCenterOfMassTransform(trans);
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::SetState( int nWorldId,  const VehicleState& state )
  {
    BulletWorldInstance *pWorld = GetWorldInstance(nWorldId);
    boost::mutex::scoped_lock lock(*pWorld);
    //load the backup onto the vehicle
    pWorld->m_vehicleBackup.LoadState(pWorld->m_pVehicle);

    //set the wheel positions and contact
    for(size_t ii = 0; ii < state.m_vWheelStates.size() ; ii++) {
      //m_pVehicle->updateWheelTransform(ii,true);
      pWorld->m_pVehicle->getWheelInfo(ii).m_worldTransform.setFromOpenGLMatrix(state.m_vWheelStates[ii].data());
      pWorld->m_pVehicle->getWheelInfo(ii).m_raycastInfo.m_isInContact = state.m_vWheelContacts[ii];
    }

    //update the parameters since they will have been overwritten
    _InternalUpdateParameters(pWorld);
    btVector3 vel(state.m_dV[0], state.m_dV[1], state.m_dV[2]);
    btVector3 w(state.m_dW[0], state.m_dW[1], state.m_dW[2]);

    //set the state 4x4 matrix, however offset the body up to account for the wheel columns
    Sophus::SE3d T = state.m_dTwv;
    T.translation() -= GetBasisVector(T,2)*
      (pWorld->m_Parameters[CarParameters::SuspRestLength] +
      pWorld->m_Parameters[CarParameters::WheelRadius]+
      pWorld->m_Parameters[CarParameters::SuspConnectionHeight]-0.01);
    SetStateNoReset(pWorld,T);

    pWorld->m_state = state;
    pWorld->m_state.m_dTwv = T;

    //set the linear velocity of the car
    pWorld->m_pVehicle->getRigidBody()->setLinearVelocity(vel);
    pWorld->m_pVehicle->getRigidBody()->setAngularVelocity(w);


    //set the steering
    pWorld->m_pVehicle->SetAckermanSteering(state.m_dSteering);


    //raycast all wheels so they are correctly positioned
    //    for (int i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
    //    {
    //        WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(i);
    //        pWorld->m_pVehicle->rayCast(wheel);
    //    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  btRigidBody*	BulletCarModel::_LocalAddRigidBody(BulletWorldInstance *pWorld, double mass, const btTransform& startTransform, btCollisionShape* shape, short group, short mask)
  {
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
      shape->calculateLocalInertia(mass,localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(BT_LARGE_FLOAT);

    pWorld->m_pDynamicsWorld->addRigidBody(body,group,mask);

    return body;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::_LocalRemoveRigidBody(BulletWorldInstance *pWorld, btCollisionShape* shape)
  {
    std::cout << "removing rigid body" << std::endl;

    btRigidBody* body = dynamic_cast<btRigidBody*>(shape);
    // btCollisionObject* body = dynamic_cast<btCollisionObject*>(shape);
    // body->setContactProcessingThreshold(BT_LARGE_FLOAT);

    pWorld->m_pDynamicsWorld->removeRigidBody(body);
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  std::vector<Sophus::SE3d> BulletCarModel::GetWheelTransforms(const int worldIndex){
    BulletWorldInstance *pWorld = GetWorldInstance(worldIndex);
    return pWorld->m_vWheelTransforms;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::ResetCommandHistory(int worldId)
  {
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    boost::mutex::scoped_lock lock(*pWorld);
    pWorld->m_lPreviousCommands.clear();
    pWorld->m_dTotalCommandTime = 0;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::GetCommandHistory(int worldId,CommandList &previousCommandsOut)
  {
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    boost::mutex::scoped_lock lock(*pWorld);
    previousCommandsOut = pWorld->m_lPreviousCommands;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  CommandList&        BulletCarModel::GetCommandHistoryRef(int worldId)
  {
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    return pWorld->m_lPreviousCommands;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void BulletCarModel::SetCommandHistory(const int& worldId, const CommandList &previousCommands)
  {
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    boost::mutex::scoped_lock lock(*pWorld);
    //find out the total time of the commands
    pWorld->m_dTotalCommandTime = 0;
    for(const ControlCommand& command : previousCommands ){
      pWorld->m_dTotalCommandTime += command.m_dT;
    }

    pWorld->m_lPreviousCommands = previousCommands;
  }

/////////////////////////////////////////////////////////////////////////////////////////
bool BulletCarModel::RayCast(const Eigen::Vector3d& dSource,const Eigen::Vector3d& dRayVector, Eigen::Vector3d& dIntersect, const bool& biDirectional, int index /*= 0*/)
{
  btVector3 source(dSource[0],dSource[1],dSource[2]);
  btVector3 vec(dRayVector[0],dRayVector[1],dRayVector[2]);
  btVector3 target = source + vec;
  BulletWorldInstance*pInstance = GetWorldInstance(index);

  btVehicleRaycaster::btVehicleRaycasterResult results,results2;
  if( biDirectional ){
    source = source - vec;
  }

  if(pInstance->m_pVehicleRayCaster->castRay(source,target,results) == 0){
    return false;
  }else{
    Eigen::Vector3d dNewSource(source[0],source[1],source[2]);
    dIntersect = dNewSource + results.m_distFraction* (biDirectional ? (Eigen::Vector3d)(dRayVector*2) : dRayVector);
    return true;
  }
}

