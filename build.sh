mkdir -p build
cd build
cmake .. -DEXPORT_CarPlanner=ON -DBULLET_COLLISION_LIBRARY=../../bullet3/build/src/BulletCollision/libBulletCollision.so -DBULLET_DYNAMICS_LIBRARY=../../bullet3/build/src/BulletDynamics/libBulletDynamics.so -DBULLET_INCLUDE_DIR=../../bullet3/src -DBULLET_MATH_LIBRARY=../../bullet3/build/src/LinearMath/libLinearMath.so -DBULLET_SOFTBODY_LIBRARY=../../bullet3/build/src/BulletSoftBody/libBulletSoftBody.so -Dassimp_INCLUDE_DIRS=/usr/include/assimp/  
make 
