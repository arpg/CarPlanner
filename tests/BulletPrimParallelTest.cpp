/**
 * @Author Subhransu Mishra (subhransu.kumar.mishra@gmail.com),
 * 				 Corin Sandford (corin.sandford@colorado.edu),
 *         Christoffer Heckman (christoffer.heckman@colorado.edu)
 * @date   July, 2016
 * @brief  cpp file to run the bullet_prim program
 *
 * Usage: BulletPrimParallelTest -mesh <meshfile.ply> -params <params.csv> -waypoints <waypoints.csv>
 */

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include <CarPlanner/BulletPrim.h>
#include <scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>
#include <yaml_eig_conv.h>
#include <thread>
#include <mutex>


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

  google::SetUsageMessage("bprim -mesh <meshfile.ply> -params <params.csv> -waypoints <waypoints.yaml>");

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

  //BulletPrim configuration
  BulletPrimConfig bulletprim_cfg;
  bulletprim_cfg.iteration_limit_ = 100;
  bulletprim_cfg.inertial_control_ = true;
  bulletprim_cfg.time_interval_ = 0.01; //default is 0.01

  size_t nthreads = 4;
  //Initialize the bullet primitive object
  vector<BulletPrim> bullet_prims;
  bullet_prims.resize(nthreads);
  for_each(bullet_prims.begin(), bullet_prims.end(),[&](BulletPrim& bp){bp.init(FLAGS_mesh, FLAGS_params , bulletprim_cfg);});

  // Read waypoints from file
  YAML::Node yaml_node_ = YAML::LoadFile(path_waypoints.string());
  Matrix<double,Dynamic,8> axyvs_start_goal = yaml_node_["axyvs_start_goal"].as< Matrix<double,Dynamic,8> >();

  // For each waypoint pair(of start and goal) just call the getPrim function from each thread
  std::vector<std::thread> threads(nthreads);
  std::mutex bp_mutex;
  int nwp = axyvs_start_goal.rows();
  cout<<"number of waypoints:"<<nwp<<endl;

  for(size_t t = 0;t<nthreads;t++){
    threads[t] = std::thread(std::bind(
        [&](const int bi, const int ei, const int t)
        {
          // loop over all items
          for(int i = bi;i<ei;i++){
            Vector4d axyv_start = axyvs_start_goal.block<1,4>( i, 0 );
            Vector4d axyv_goal  = axyvs_start_goal.block<1,4>( i, 4 );
            vector<Vector2d> prim_ctrl;
            vector<Vector4d> prim_path;
            bullet_prims[t].getPrim(axyv_start,axyv_goal,prim_ctrl,prim_path);
            {
              std::lock_guard<std::mutex> lock(bp_mutex);
              cout<<"computed waypoint:"<<i<<endl;
            }

          }
        },t*nwp/nthreads,(t+1)==nthreads?nwp:(t+1)*nwp/nthreads,t));
  }
  std::for_each(threads.begin(),threads.end(),[](std::thread& x){x.join();});


  return 0;
}
