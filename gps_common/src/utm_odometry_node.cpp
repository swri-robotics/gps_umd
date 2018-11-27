/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <novatel_gps_msgs/Inspva.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

using namespace gps_common;

ros::Publisher odom_pub;
ros::Subscriber gps_source_sub;
std::string frame_id, child_frame_id;
double rot_cov, y_offset, x_offset, z_offset;
double roll_offset, pitch_offset, yaw_offset;
bool odom_init, publish_odom_tf;


void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_DEBUG_THROTTLE(60,"No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = easting;
    odom.pose.pose.position.y = northing;
    odom.pose.pose.position.z = fix->altitude;
    
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    odom_pub.publish(odom);
  }
}

void insCallBack(const novatel_gps_msgs::InspvaConstPtr& ins) {

  double northing, easting;
  std::string zone;
  static tf::TransformBroadcaster odom_tf_broadcaster;
  static geometry_msgs::TransformStamped odom_tf;
  static geometry_msgs::PoseStamped imu_pose;

  tf::TransformListener tfListener;
  LLtoUTM(ins->latitude, ins->longitude, northing, easting, zone);
  if(!odom_init)
  {
	  x_offset = easting;
	  y_offset = northing;
	  z_offset = ins->height;

      roll_offset = ins->roll;
      pitch_offset = ins->pitch;
      yaw_offset = ins->azimuth;
      odom_init = true;
  }
  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ins->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = ins->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = easting - x_offset;
    odom.pose.pose.position.y = northing - y_offset;
    odom.pose.pose.position.z = 0;

    double yaw =   90 - ins->azimuth; //need to do this 90deg offset per novatel requirements.
    double pitch = ins->pitch;
    double roll =  ins->roll;

    roll = roll * 0.01745329252;
    pitch = pitch * 0.01745329252;
    yaw = yaw * 0.01745329252; // pi/180 convert to radians

    tf::Quaternion quat_imu;
    quat_imu.setRPY(roll, pitch, yaw);

    odom.pose.pose.orientation.x = quat_imu[0];
    odom.pose.pose.orientation.y = quat_imu[1];
    odom.pose.pose.orientation.z = quat_imu[2];
    odom.pose.pose.orientation.w = quat_imu[3];


    double velocity = sqrt(pow(ins->east_velocity,2)+pow(ins->north_velocity,2)+pow(ins->up_velocity,2));

    odom.twist.twist.linear.x = velocity;
    odom.twist.twist.linear.y = 0; //TODO:
    odom.twist.twist.linear.z = 0; //TODO:

    odom.pose.covariance = {1,0,0,0,0,0,
			 	 	 	 	0,1,0,0,0,0,
							0,0,1,0,0,0,
							0,0,0,1,0,0,
							0,0,0,0,1,0,
							0,0,0,0,0,1};

    odom_pub.publish(odom);
    if(publish_odom_tf)
    {
    	odom_tf.header.stamp = odom.header.stamp;
        odom_tf.header.frame_id = frame_id;
        odom_tf.child_frame_id = child_frame_id;
        odom_tf.transform.translation.x = odom.pose.pose.position.x;
        odom_tf.transform.translation.y = odom.pose.pose.position.y;
        odom_tf.transform.translation.z = odom.pose.pose.position.z;

        odom_tf.transform.rotation.x = quat_imu[0];
        odom_tf.transform.rotation.y = quat_imu[1];
        odom_tf.transform.rotation.z = quat_imu[2];
        odom_tf.transform.rotation.w = quat_imu[3];

    	odom_tf_broadcaster.sendTransform(odom_tf);
    }
  }
}


int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  bool use_ins = true;
  odom_init = false; //initialization flag for odom.
  publish_odom_tf = false;
  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
  if(!priv_node.getParam("use_ins", use_ins))
	  ROS_WARN_STREAM("No use_ins param provided, using default: "<<use_ins);

  if(!priv_node.getParam("publish_odom_tf", publish_odom_tf))
	  ROS_WARN_STREAM("No publish_odom_tf param provided, using default: "<<publish_odom_tf);

  if(use_ins)
  {
	  odom_pub = node.advertise<nav_msgs::Odometry>("/odom", 10);
	  gps_source_sub = node.subscribe("/inspva", 10, insCallBack);
  }
  else
  {
	  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);
	  gps_source_sub = node.subscribe("fix", 10, callback);
  }

  ros::spin();
}

