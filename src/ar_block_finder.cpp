#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <tf2_kdl/tf2_kdl.h>

class BaseLinkToBlockTransformer {
public:
  BaseLinkToBlockTransformer(
      ros::NodeHandle& nh,
      std::string& camera_link,
      std::string& ar_marker_base_link_frame,
      std::string& ar_marker_block_frame,
      std::string& block_frame,
      std::string& base_link_frame,
      bool print_camera_frame
    ) : 
    nh_(nh),
    camera_link_(camera_link),
    ar_marker_base_link_frame_(ar_marker_base_link_frame), 
    ar_marker_block_frame_(ar_marker_block_frame), 
    block_frame_(block_frame), 
    base_link_frame_(base_link_frame), 
    tf_listener(buffer_), 
    print_camera_frame_(print_camera_frame),
    rate(10)
  {
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ar_block_finder/pose", 1);
  }

  bool spin() {
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    geometry_msgs::TransformStamped camera_to_base_link_marker;
    camera_to_base_link_marker.header.frame_id = camera_link_;
    camera_to_base_link_marker.child_frame_id = ar_marker_base_link_frame_;
    geometry_msgs::Transform camera_to_base_link_marker_cached_transform;
    bool camera_to_base_link_marker_cached_transform_available = false;

    geometry_msgs::TransformStamped base_link_marker_to_base_link;
    base_link_marker_to_base_link.header.frame_id = ar_marker_base_link_frame_;
    base_link_marker_to_base_link.child_frame_id = base_link_frame_;
    base_link_marker_to_base_link.transform.translation.x = -0.03;
    base_link_marker_to_base_link.transform.translation.y = 0.0;
    base_link_marker_to_base_link.transform.translation.z = 0.0;
    base_link_marker_to_base_link.transform.rotation.x = q.x();
    base_link_marker_to_base_link.transform.rotation.y = q.y();
    base_link_marker_to_base_link.transform.rotation.z = q.z();
    base_link_marker_to_base_link.transform.rotation.w = q.w();

    geometry_msgs::TransformStamped block_marker_to_block;
    block_marker_to_block.header.frame_id = ar_marker_block_frame_;
    block_marker_to_block.child_frame_id = block_frame_;
    block_marker_to_block.transform.translation.x = 0.0;
    block_marker_to_block.transform.translation.y = 0.0;
    block_marker_to_block.transform.translation.z = -0.015;
    block_marker_to_block.transform.rotation.x = q.x();
    block_marker_to_block.transform.rotation.y = q.y();
    block_marker_to_block.transform.rotation.z = q.z();
    block_marker_to_block.transform.rotation.w = q.w();

    static_broadcaster.sendTransform(block_marker_to_block);
    static_broadcaster.sendTransform(base_link_marker_to_base_link);

    block_pose.header.frame_id = block_frame_;
    block_pose.pose.orientation.x = q.x();
    block_pose.pose.orientation.y = q.y();
    block_pose.pose.orientation.z = q.z();
    block_pose.pose.orientation.w = q.w();

    geometry_msgs::TransformStamped base_link_to_block;

    while (nh_.ok()) {
      ros::Time time(ros::Time::now());
      block_pose.header.stamp = time;

      try {
        camera_to_base_link_marker = buffer_.lookupTransform(camera_link_, ar_marker_base_link_frame_, time, ros::Duration(1.0));
        camera_to_base_link_marker_cached_transform_available = true;
        camera_to_base_link_marker_cached_transform = camera_to_base_link_marker.transform; 
      } catch (tf2::TransformException &ex) {
        if (camera_to_base_link_marker_cached_transform_available) {
          camera_to_base_link_marker.header.stamp = time;
          camera_to_base_link_marker.transform = camera_to_base_link_marker_cached_transform;
          broadcaster.sendTransform(camera_to_base_link_marker);
        }
      }

      try {
        base_link_to_block = buffer_.lookupTransform(base_link_frame_, block_frame_, time, ros::Duration(1.0));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      geometry_msgs::Vector3 &translation = base_link_to_block.transform.translation;
      block_pose.pose.position.x = translation.x;
      block_pose.pose.position.y = translation.y;
      block_pose.pose.position.z = translation.z;

      pub_.publish(block_pose);

      if (print_camera_frame_) {
        try {
          geometry_msgs::TransformStamped camera_to_base_link;
          camera_to_base_link = buffer_.lookupTransform(base_link_frame_, camera_link_, time, ros::Duration(2.0));
          geometry_msgs::Transform &transform = camera_to_base_link.transform;
          geometry_msgs::Transform transform2 = camera_to_base_link.transform;
          double roll, pitch, yaw;
          tf2::transformToKDL(camera_to_base_link).M.GetRPY(roll, pitch, yaw);
          ROS_INFO("Camera translation x, y, z and rotation roll, pitch, yaw: %f, %f, %f, %f, %f, %f", transform.translation.x, transform.translation.y, transform.translation.z, roll, pitch, yaw);
        } catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
      }

      rate.sleep();
    }
  }

private:
  ros::NodeHandle& nh_;
  message_filters::Subscriber<geometry_msgs::TransformStamped> sub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  tf2_ros::TransformBroadcaster broadcaster;
  ros::Rate rate;
  ros::Publisher pub_;
  std::string& camera_link_;
  std::string& ar_marker_base_link_frame_;
  std::string& ar_marker_block_frame_;
  std::string& block_frame_;
  std::string& base_link_frame_;
  bool print_camera_frame_;
  geometry_msgs::PoseStamped block_pose;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_link_to_block");
  ros::NodeHandle nh;
  std::string camera_link(nh.param("camera_link_frame", std::string("camera_link")));
  std::string ar_marker_base_link_frame(nh.param("ar_marker_base_link_frame_id", std::string("ar_marker_4")));
  std::string ar_marker_block_frame(nh.param("ar_marker_block_frame_id", std::string("ar_marker_9")));
  std::string block_frame(nh.param("block_frame", std::string("block")));
  std::string base_link_frame(nh.param("base_link_frame", std::string("base_link")));
  bool print_camera_frame(nh.param("print_camera_transform", false));
  BaseLinkToBlockTransformer node(nh, camera_link, ar_marker_base_link_frame, ar_marker_block_frame, block_frame, base_link_frame, print_camera_frame);
  node.spin();
  return 0;
}
