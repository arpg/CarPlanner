#ifndef _TF_CONVERSION_TOOLS
#define	_TF_CONVERSION_TOOLS

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/tf.h"
#include <sophus/se3.hpp>

// tf::Transform rot_270_x(tf::Quaternion(-0.707,0,0,0.707),tf::Vector3(0,0,0));
// tf::Transform rot_90_y(tf::Quaternion(0,0.707,0,0.707),tf::Vector3(0,0,0));
// tf::Transform rot_180_y(tf::Quaternion(0,1,0,0),tf::Vector3(0,0,0));
// tf::Transform rot_270_y(tf::Quaternion(0,-0.707,0,0.707),tf::Vector3(0,0,0));
// tf::Transform rot_90_z(tf::Quaternion(0,0,0.707,0.707),tf::Vector3(0,0,0));
// tf::Transform rot_180_z(tf::Quaternion(0,0,1,0),tf::Vector3(0,0,0));
// tf::Transform rot_270_z(tf::Quaternion(0,0,-0.707,0.707),tf::Vector3(0,0,0));
// tf::Transform link_to_optical(tf::Quaternion(-0.5,0.5,-0.5,0.5),tf::Vector3(0,0,0));

// inline Sophus::SE3d& GLWaypoint2SE3d(GLWaypoint pt) {
// 	return Sophus::SE3d( pt->GetPose4x4_po() );
// }

inline Sophus::SE3d& PoseMsg2SE3d(geometry_msgs::PoseStamped waypoint) {
	Sophus::SE3d pose;
	pose.translation() = *(new Eigen::Vector3d(waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z));
	pose.setQuaternion(Eigen::Quaterniond(waypoint.pose.orientation.w, waypoint.pose.orientation.x, waypoint.pose.orientation.y, waypoint.pose.orientation.z));
	return pose;
}

inline Eigen::Matrix4d XYZRPY2TMatrix(
		double x,
		double y,
		double z,
		double r,
		double p,
		double q) {
	Eigen::Matrix4d T;
	// psi = roll, th = pitch, phi = yaw
	double cq, cp, cr, sq, sp, sr;
	cr = cos(r);
	cp = cos(p);
	cq = cos(q);

	sr = sin(r);
	sp = sin(p);
	sq = sin(q);

	T(0, 0) = cp * cq;
	T(0, 1) = -cr * sq + sr * sp * cq;
	T(0, 2) = sr * sq + cr * sp * cq;

	T(1, 0) = cp * sq;
	T(1, 1) = cr * cq + sr * sp * sq;
	T(1, 2) = -sr * cq + cr * sp * sq;

	T(2, 0) = -sp;
	T(2, 1) = sr * cp;
	T(2, 2) = cr * cp;

	T(0, 3) = x;
	T(1, 3) = y;
	T(2, 3) = z;
	T.row(3) = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
	return T;
}

inline Eigen::Matrix4d XYZRPY2TMatrix(const Eigen::Matrix<double, 6, 1>& x) {
	return XYZRPY2TMatrix(x(0), x(1), x(2), x(3), x(4), x(5));
}

inline Eigen::Vector3d RMatrix2RPY(const Eigen::Matrix3d& R) {
  Eigen::Vector3d rpq;
  // roll
  rpq[0] = atan2(R(2, 1), R(2, 2));

  // pitch
  double det = -R(2, 0) * R(2, 0) + 1.0;
  if (det <= 0) {
    if (R(2, 0) > 0) {
      rpq[1] = -M_PI / 2.0;
    } else {
      rpq[1] = M_PI / 2.0;
    }
  } else {
    rpq[1] = -asin(R(2, 0));
  }
  // yaw
  rpq[2] = atan2(R(1, 0), R(0, 0));
  return rpq;
}

inline Eigen::Matrix<double, 6, 1> TMatrix2XYZRPY(const Eigen::Matrix4d& T) {
  Eigen::Matrix<double, 6, 1> Cart;
  Eigen::Vector3d rpq = RMatrix2RPY(T.block<3, 3>(0, 0));
  Cart[0] = T(0, 3);
  Cart[1] = T(1, 3);
  Cart[2] = T(2, 3);
  Cart[3] = rpq[0];
  Cart[4] = rpq[1];
  Cart[5] = rpq[2];
  return Cart;
}

inline void tf2geometry_msg(tf::Transform& tf, geometry_msgs::Transform* gm)
{
	gm->translation.x = tf.getOrigin().getX(); 
	gm->translation.y = tf.getOrigin().getY();
	gm->translation.z = tf.getOrigin().getZ();
	gm->rotation.x = tf.getRotation().getX();
	gm->rotation.y = tf.getRotation().getY();
	gm->rotation.z = tf.getRotation().getZ();
	gm->rotation.w = tf.getRotation().getW();
}

inline void tf2geometry_msg(tf::StampedTransform& tf, geometry_msgs::TransformStamped* gm)
{
	geometry_msgs::Transform gm_temp;
	tf2geometry_msg(tf, &gm_temp);
	gm->transform = gm_temp;
	gm->header.frame_id = tf.frame_id_;
	gm->child_frame_id = tf.child_frame_id_;
	gm->header.stamp = tf.stamp_;
}

inline void geometry_msg2tf(geometry_msgs::Transform& gm, tf::Transform* tf)
{
	tf->setOrigin(tf::Vector3(gm.translation.x, gm.translation.y, gm.translation.z));
	tf->setRotation(tf::Quaternion(gm.rotation.x, gm.rotation.y, gm.rotation.z, gm.rotation.w));
}

inline void geometry_msg2tf(geometry_msgs::TransformStamped& gm, tf::StampedTransform* tf)
{
	tf::Transform tf_temp;
	geometry_msg2tf(gm.transform, &tf_temp);
	tf->setData(tf_temp);
	tf->frame_id_ = gm.header.frame_id;
	tf->child_frame_id_ = gm.child_frame_id;
	tf->stamp_ = gm.header.stamp;
}

#endif
