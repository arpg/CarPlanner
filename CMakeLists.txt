project( CarPlanner )
cmake_minimum_required(VERSION 2.8)

# Add to module path, so we can find our cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
include(def_library)
include(export_package)

message( STATUS "CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}" )

add_definitions(-DBT_USE_DOUBLE_PRECISION)
add_definitions(-DBT_NO_PROFILE)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall") #-DBOOST_NO_CXX11_NUMERIC_LIMITS")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

# the boost libraries that we need
#set( BOOST_LIBS thread date_time system timer )

find_package( Eigen3 REQUIRED )
FIND_PACKAGE( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( CVARS REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Bullet REQUIRED )
#find_package( Boost COMPONENTS ${BOOST_LIBS} REQUIRED )
find_package( QUATLIB REQUIRED )
find_package( Sophus REQUIRED )
find_package( VRPN REQUIRED ) # requires some work!

set( _LIBRARIES  ${BULLET_LIBRARIES}  ${BOOST_LIBRARIES} ${VRPN_LIBRARY} ${Sophus_LIBRARIES} ${VRPN_SERVER_LIBRARY}) # ${MVL_LIBRARIES})
set( _INCLUDE_DIRS
        ${CMAKE_BINARY_DIR}
        ${Sophus_INCLUDE_DIR}
        ${VRPN_INCLUDES}
        ${EIGEN3_INCLUDE_DIR}
        #${MVL_INCLUDE_DIRS}
        ${CVARS_INCLUDE_DIR}
        ${BULLET_INCLUDE_DIR}
        ${SceneGraph_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

include_directories( ${_INCLUDE_DIRS} )


include_directories( ${CAR_PLANNER_INCLUDE_DIRS} )

set( SRCS
        src/ApplyVelocitiesFunctor.cpp
        src/BulletCarModel.cpp
        src/CarRegressor.cpp
        src/LocalPlanner.cpp
        src/WheelInfo.cpp
        src/CarParameters.cpp
        src/GLDebugDrawer.cpp
        src/RaycastVehicle.cpp
        src/BezierBoundarySolver.cpp
        src/MochaException.cpp
        src/CarController.cpp
        src/Vicon.cpp
        src/DirectBoundarySolver.cpp
   )

set( HDRS
        include/CarPlanner/ApplyVelocitiesFunctor.h
        include/CarPlanner/CarPlannerCommon.h
        include/CarPlanner/BulletCarModel.h
        include/CarPlanner/Vicon.h
        include/CarPlanner/CarWheel.h
        include/CarPlanner/CarRegressor.h
        include/CarPlanner/CarController.h
        include/CarPlanner/GLDebugDrawer.h
        include/CarPlanner/CarParameters.h
        include/CarPlanner/DirectBoundarySolver.h
        include/CarPlanner/CVarHelpers.h
   )

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

def_library(${LIBRARY_NAME}
  SOURCES ${HDRS} ${SRCS}
  LINK_LIBS ${_LIBRARIES}
  PACKAGE CarPlanner
  )

export_package( CarPlanner
  TARGETS ${LIBRARY_NAME}
  LIBRARY ${LIBRARY_NAME}
  LIBRARIES ${LIBRARY_NAME}
  INCLUDE_DIRS ${_INCLUDE_DIRS}
  )

