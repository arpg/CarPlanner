/**
 * @Author Subhransu Mishra (subhransu.kumar.mishra@gmail.com),
 * 				 Corin Sandford (corin.sandford@colorado.edu),
 *         Christoffer Heckman (christoffer.heckman@colorado.edu)
 * @date   July, 2016
 * @brief  cpp file to run the bullet_prim program
 *
 * Usage: BulletPrimTest -mesh <meshfile.ply> -params <params.csv> -waypoints <waypoints.csv>
 */

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include <CarPlanner/BulletPrim.h>
#include <scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>
#include <CarPlanner/yaml_eig_conv.h>


DEFINE_string(mesh, "", "File for ground mesh.");
DEFINE_string(params, "", "CSV file for car parameters.");
DEFINE_string(waypoints, "", "CSV file for waypoints in x1,y1,yaw1,x2,y2,yaw2");

using boost::filesystem::path;
using boost::filesystem::absolute;
using boost::filesystem::exists;

int main( int argc, char* argv[] ) 
{
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  google::SetUsageMessage("BulletPrimTest -mesh <meshfile.ply> -params <params.csv> -waypoints <waypoints.yaml>");

  if(FLAGS_mesh.empty() || FLAGS_params.empty() || FLAGS_waypoints.empty()) {
    LOG(FATAL) << "One of the files(mesh, params, waypoints) missing \n"
        << "Usage: BulletPrimTest -mesh <meshfile.ply> -params <params.csv> -waypoints <waypoints.yaml>"<<endl;
    return -1;
  }

  //Get absolute path for different files
  path path_waypoints(absolute(path(FLAGS_waypoints)));

  //Check if the file exists and the right type
  bool file_probs(false);
  if(!exists(path_waypoints)){
    file_probs = true;
    cout<<"path_waypoints doesn't exist"<<endl;
  }else if(path_waypoints.extension().string().compare(".yaml") ){
    file_probs = true;
    cout<<"path_waypoints has the wrong extension:"<<endl;
  }else{
    cout<<"path_waypoints: "<<path_waypoints<<endl;
  }

  cout << endl;

  if(file_probs){
    cout<<"file probs"<<endl;
    return -1;
  }

  //Initialize the bullet primitive object
  BulletPrim bullet_prim( FLAGS_mesh, FLAGS_params );

  // Read waypoints from file
  YAML::Node yaml_node_ = YAML::LoadFile(path_waypoints.string());

  Matrix<double,Dynamic,8> axyvs_start_goal = yaml_node_["axyvs_start_goal"].as< Matrix<double,Dynamic,8> >();

  // For each waypoint pair(of start and goal) find the ctrl and path that takes from start to goal
  for( int i = 0; i < axyvs_start_goal.rows(); i++ ) {
    Vector4d axyv_start = axyvs_start_goal.block<1,4>( i, 0 );
    Vector4d axyv_goal  = axyvs_start_goal.block<1,4>( i, 4 );
    vector<Vector2d> prim_ctrl;
    vector<Vector4d> prim_path;
    double cost = bullet_prim.getPrim(axyv_start, axyv_goal,prim_ctrl, prim_path);
    cout << "Waypoint: yaw start, x start, y start, v start, yaw goal, x goal, y goal, v goal:" << i << endl;
    cout << "\t" << axyvs_start_goal.block<1,8>( i, 0 ) << endl;
    cout << "cost: "<<cost<<endl;
    cout<<" # of control points:"<<prim_ctrl.size()<<" and  # of path points"<<prim_path.size()<<endl;
    for(size_t j=0; j <prim_ctrl.size(); j++){
      cout<<j<<": tau phi:"<<prim_ctrl[j].transpose()<<": axyv:"<<prim_path[j].transpose()<<endl;
    }
    cout << endl;
  }
  return 0;
}
