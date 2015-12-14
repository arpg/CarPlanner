project( CarPlanner )
cmake_minimum_required(VERSION 2.8)
enable_testing()

# Add to module path, so we can find our cmake modules
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules )

STRING( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

include(install_package)

add_definitions(-DBT_USE_DOUBLE_PRECISION)
add_definitions(-DBT_NO_PROFILE)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -DBOOST_NO_CXX11_NUMERIC_LIMITS")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()


#the boost libraries that we need
set(BOOST_LIBS thread date_time system timer)


find_package( Eigen3 REQUIRED )
find_package(Pangolin REQUIRED)
find_package( Node REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( CVars REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Bullet REQUIRED )
find_package( HAL REQUIRED )
find_package( Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package( QUATLIB REQUIRED )
find_package(Sophus REQUIRED)

set( _LIBRARIES ${BULLET_LIBRARIES}  ${Boost_LIBRARIES} ${Sophus_LIBRARIES} ${SceneGraph_LIBRARIES} ${Node_LIBRARIES} ${CVars_LIBRARIES} ${HAL_LIBRARIES} )
set( _INCLUDE_DIRS
        ${CMAKE_BINARY_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Node_INCLUDE_DIRS}
        ${CVars_INCLUDE_DIR}
        ${BULLET_INCLUDE_DIR}
        ${SceneGraph_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${HAL_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        )

include_directories( ${_INCLUDE_DIRS} )

#link_libraries( CarPlanner   OpenGLSupport )

# expose these for third parties
set( CAR_PLANNER_INCLUDE_DIRS ${_INCLUDE_DIRS}  CACHE INTERNAL "" FORCE )
set( CAR_PLANNER_LIBRARIES  ${_LIBRARIES} CACHE INTERNAL "" FORCE )

include_directories( ${CAR_PLANNER_INCLUDE_DIRS} )

set( SRCS
        ApplyVelocitiesFunctor.cpp
        BulletCarModel.cpp
        CarRegressor.cpp
        LocalPlanner.cpp
        WheelInfo.cpp
        CarParameters.cpp
        GLDebugDrawer.cpp
        RaycastVehicle.cpp
        BezierBoundarySolver.cpp
        MochaException.cpp
        CarController.cpp
        Localizer.cpp
        DirectBoundarySolver.cpp
   )

set( HDRS
        ApplyVelocitiesFunctor.h
        CarPlannerCommon.h
        BulletCarModel.h
        Localizer.h
        CarWheel.h
        CarRegressor.h
        CarController.h
        GLDebugDrawer.h
        CarParameters.h
        DirectBoundarySolver.h
        CVarHelpers.h
   )


add_library( ${LIBRARY_NAME} SHARED ${SRCS} ${HDRS} )
target_link_libraries( ${LIBRARY_NAME} ${_LIBRARIES} )

install_package(
    PKG_NAME ${PROJECT_NAME}
    LIB_NAME ${LIBRARY_NAME}
    VERSION 2.1
    DESCRIPTION "CarPlanner library."
    INSTALL_INCLUDE_DIR true
    INSTALL_GENERATED_HEADERS ${GENERATED_HEADERS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    INCLUDE_DIRS ${_INCLUDE_DIRS}
    LINK_LIBS ${_LIBRARIES}
    )

###########################

include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
###########################
