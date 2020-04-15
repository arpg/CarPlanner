mkdir -p build
cd build
cmake .. -DEXPORT_carplanner=ON -DBULLET_COLLISION_LIBRARY=/home/mike/code/bullet3/build/src/BulletCollision/libBulletCollision.so -DBULLET_DYNAMICS_LIBRARY=/home/mike/code/bullet3/build/src/BulletDynamics/libBulletDynamics.so -DBULLET_INCLUDE_DIR=/home/mike/code/bullet3/src -DBULLET_MATH_LIBRARY=/home/mike/code/bullet3/build/src/LinearMath/libLinearMath.so -DBULLET_SOFTBODY_LIBRARY=/home/mike/code/bullet3/build/src/BulletSoftBody/libBulletSoftBody.so
make -j12

