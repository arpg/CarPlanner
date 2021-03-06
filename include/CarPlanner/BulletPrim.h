﻿/**
 * @Author Subhransu Mishra (subhransu.kumar.mishra@gmail.com)
 * @date   July, 2016
 * @brief  header file to generate control primitives for bullet car
 *
 * Detailed description of file.
 */

#ifndef BULLET_PRIM_H
#define BULLET_PRIM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "LocalPlannerNoCVar.h"
#include "BulletCarModel.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include <memory>
#include <atomic>

using namespace std;
using namespace Eigen;

class BulletPrimConfig{
public:
  BulletPrimConfig(int g_nIterationLimit=10,bool g_bInertialControl=true, double time_interval = 0.01);

  int iteration_limit_;
  bool inertial_control_;
  double time_interval_;
};

class BulletPrim{
public:

    BulletPrim();

    BulletPrim(const BulletPrim&);

    BulletPrim(const std::string& sMeshFile, const std::string& sParamsFile, const BulletPrimConfig& cfg = BulletPrimConfig() );

    void init(const std::string& sMeshFile, const std::string& sParamsFile, const BulletPrimConfig& cfg);

    ~BulletPrim();

    /**
     * Returns the control pimitive/s that will take the car from start to goal
     * @param axyv_start The start as yaw, x, y and v(forward vel)
     * @param axyv_goal The goal as yaw, x, y and v(forward vel)
     * @param prim_ctrl control pimitive/s a set of torques and steering angles
     * @param prim_path path primitive/s as a set of yaw, x, y and v(forward vel)
     * @return cost of the path given some metric(e.g. more torque more cost). -ve cost if path can't be found
     */
    double getPrim( const Vector4d& axyv_start, const Vector4d& axyv_goal, vector<Vector2d>& prim_ctrl, vector<Vector4d>& prim_path);


    /**
     * Solves the boundary value problem given a start state and goal state.
     * @param axyv_start Start state as yaw, x, y and forward velocity.
     * @param axyv_goal Goal state as yaw, x, y and forward velocity.
     * @param motion_sample The solution to the BVP as vector of controls and trajectories
     * @return Returns true if a solution was found
     */
    bool solveBVP(const Vector4d& axyv_start, const Vector4d& axyv_goal, MotionSample& motion_sample);

private:
    /**
    * Converts a vector of ControlCommands to a cost
    * @param commands The command
    * @return The cost
    */
    double controlCommandsToCost(const vector<ControlCommand>& commands);

private:
    CarParameterMap m_mDefaultParameters; //! BulletCarModel car parameters

    BulletCarModel m_PlanCarModel; //! BulletCarModel for simulating planned paths for feasibility

    LocalPlannerNoCVar m_Planner; // Car planner

    BulletPrimConfig cfg_;

    bool initialized;
};

#endif //BULLET_PRIM_H

