/**
 * @Author Subhransu Mishra (subhransu.kumar.mishra@gmail.com),
 *         Corin Sandford (corin.sandford@colorado.edu),
 *         Christoffer Heckman (christoffer.heckman@colorado.edu)
 * @date   July, 2016
 * @brief  cpp file to generate control primitives for bullet car
 *
 * Detailed description of file.
 */

#include "CarPlanner/LocalPlannerNoCVar.h"
#include <SceneGraph/GLWaypoint.h>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace SceneGraph;

int main(int argc, char** argv){

//  LocalPlannerNoCVar planner1;
//  cout<<"planner 1 created"<<endl;
//
//  LocalPlannerNoCVar planner2;
//  cout<<"planner 2 created"<<endl;


  Vector6d xyzrpy_start; xyzrpy_start<< 10,2,3,0.6,0.7,0.5;

  GLWayPoint glwp_start;
  glwp_start.SetPose(xyzrpy_start);
  glwp_start.SetVelocity(1);

  cout<<"glwp_start.GetPose5d():"<< glwp_start.GetPose5d()<<endl;
  cout<<"glwp_start.GetPose4x4_po()"<<glwp_start.GetPose4x4_po()<<endl;
  cout<<"velocity:"<<glwp_start.GetVelocity()<<endl;

  return 0;
}
