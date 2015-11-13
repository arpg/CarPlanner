project( CarPlanner )
cmake_minimum_required(VERSION 2.8)
set( VERSION 2.0 )

if( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add to module path, so we can find our cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

add_definitions(-DBT_USE_DOUBLE_PRECISION)
add_definitions(-DBT_NO_PROFILE)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

find_package( Eigen3 REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Bullet REQUIRED )
find_package( Sophus REQUIRED )

set( REQUIRED_LIBRARIES ${BULLET_LIBRARIES}  ${BOOST_LIBRARIES} ${EIGEN3_LIBRARIES} )

set( REQUIRED_INCLUDE_DIRS
     ${CMAKE_BINARY_DIR}
     ${Sophus_INCLUDE_DIRS}
     ${EIGEN3_INCLUDE_DIR}
     ${BULLET_INCLUDE_DIR}
     )

set( SRCS
        src/ApplyVelocitiesFunctor.cpp
        src/BulletCarModel.cpp
        src/CarRegressor.cpp
        src/LocalPlanner.cpp
        src/WheelInfo.cpp
        src/CarParameters.cpp
        src/RaycastVehicle.cpp
        src/BezierBoundarySolver.cpp
        src/MochaException.cpp
        src/CarController.cpp
        src/DirectBoundarySolver.cpp
        src/RpgUtils.h
   )

set( HDRS
        include/CarPlanner/ApplyVelocitiesFunctor.h
        include/CarPlanner/CarPlannerCommon.h
        include/CarPlanner/BulletCarModel.h
        include/CarPlanner/LocalPlanner.h
        include/CarPlanner/CarWheel.h
        include/CarPlanner/CarRegressor.h
        include/CarPlanner/CarController.h
        include/CarPlanner/CarParameters.h
        include/CarPlanner/DirectBoundarySolver.h
        include/CarPlanner/RaycastVehicle.h
        include/CarPlanner/ThreadPool.h
   )

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

include_directories( ${REQUIRED_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include)

add_library(${LIBRARY_NAME} ${SRCS} ${HDRS})
target_link_libraries(${LIBRARY_NAME} ${REQUIRED_LIBRARIES})

include(install_package)

install_package(
    PKG_NAME ${PROJECT_NAME}
    LIB_NAME ${LIBRARY_NAME}
    VERSION ${VERSION}
    DESCRIPTION "Statically typed library depending on bullet for SITL Control."
    INSTALL_INCLUDE_DIR true
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    INCLUDE_DIRS ${REQUIRED_INCLUDE_DIRS}
    LINK_LIBS ${REQUIRED_LIBRARIES}
    )
