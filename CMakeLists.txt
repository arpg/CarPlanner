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
set(BOOST_LIBS thread date_time system timer filesystem)

find_package(PkgConfig REQUIRED)
find_package( Eigen3 REQUIRED )
find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( CVars REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Bullet REQUIRED )
find_package( HAL REQUIRED )
find_package( Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package( QUATLIB REQUIRED )
find_package(Sophus REQUIRED)
find_package( assimp REQUIRED )
find_package( yaml-cpp REQUIRED )
find_package( gflags REQUIRED )

set( _LIBRARIES ${BULLET_LIBRARIES}  ${Boost_LIBRARIES} ${Sophus_LIBRARIES} ${SceneGraph_LIBRARIES} ${CVars_LIBRARIES} ${HAL_LIBRARIES} ${assimp_LIBRARIES} ${YAML_CPP_LIBRARIES} ${GFLAGS_LIBRARIES})
set( _INCLUDE_DIRS
        ${CMAKE_BINARY_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
				${CVars_INCLUDE_DIRS}
        ${BULLET_INCLUDE_DIR}
        ${assimp_INCLUDE_DIRS}
        ${SceneGraph_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${HAL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        ${GFLAGS_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

#link_libraries( CarPlanner   OpenGLSupport )

# expose these for third parties
set( CarPlanner_INCLUDE_DIRS ${_INCLUDE_DIRS}  CACHE INTERNAL "" FORCE )
set( CarPlanner_LIBRARIES  ${_LIBRARIES} CACHE INTERNAL "" FORCE )

include_directories( ${CarPlanner_INCLUDE_DIRS} )

set( SRCS
        src/ApplyVelocitiesFunctor.cpp
        src/BulletCarModel.cpp
        src/CarRegressor.cpp
        src/LocalPlanner.cpp
        src/LocalPlannerNoCVar.cpp
        src/WheelInfo.cpp
        src/CarParameters.cpp
        src/GLDebugDrawer.cpp
        src/RaycastVehicle.cpp
        src/BezierBoundarySolver.cpp
        src/MochaException.cpp
        src/CarController.cpp
        src/Localizer.cpp
        src/DirectBoundarySolver.cpp
        src/BulletPrim.cpp
   )

set( HDRS
        include/CarPlanner/ApplyVelocitiesFunctor.h
        include/CarPlanner/CarPlannerCommon.h
        include/CarPlanner/BulletCarModel.h
        include/CarPlanner/Localizer.h
        include/CarPlanner/CarWheel.h
        include/CarPlanner/CarRegressor.h
        include/CarPlanner/CarController.h
        include/CarPlanner/GLDebugDrawer.h
        include/CarPlanner/CarParameters.h
        include/CarPlanner/DirectBoundarySolver.h
        include/CarPlanner/CVarHelpers.h
        include/CarPlanner/RpgUtils.h
        include/CarPlanner/BulletPrim.h
        include/CarPlanner/yaml_eig_conv.h
   )

add_library( ${LIBRARY_NAME} SHARED ${SRCS} ${HDRS} )
target_link_libraries( ${LIBRARY_NAME} ${_LIBRARIES} )


add_subdirectory(tests)

install_package(
    PKG_NAME ${PROJECT_NAME}
    LIB_NAME ${LIBRARY_NAME}
    VERSION 2.1
    DESCRIPTION "CarPlanner library."
    INSTALL_INCLUDE_DIR true
    INSTALL_GENERATED_HEADERS ${GENERATED_HEADERS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    INCLUDE_DIRS ${CarPlanner_INCLUDE_DIRS}
    LINK_LIBS ${CarPlanner_LIBRARIES}
    )

###########################

include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
###########################
