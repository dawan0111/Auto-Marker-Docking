#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_eigen/tf2_eigen.h>

#include "auto_marker_docking/aruco_marker_detector.hpp"
#include "auto_marker_docking/aruco_kalman_filter.hpp"
#include "auto_marker_docking/utils.hpp"

class AutoMarkerDockingNode : public rclcpp::Node {
public:
  AutoMarkerDockingNode() : Node("auto_marker_docking_node") {
    RCLCPP_INFO(this->get_logger(), "auto_marker_docking_node");
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&AutoMarkerDockingNode::image_callback_, this,
                  std::placeholders::_1));

    camera_intrinsic_ = (cv::Mat_<float>(3, 3) << 1696.80268, 0.0, 960.5, 0.0,
                         1696.80268, 540.5, 0.0, 0.0, 1.0);
    camera_dist_coeffs_ = (cv::Mat_<float>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
    aruco_marker_size_ = 0.1;
    aurco_marker_dictionary_id_ = cv::aruco::DICT_6X6_250;
    detect_marker_pose_ = Eigen::Isometry3d::Identity();

    T_world_image_ = Eigen::Isometry3d::Identity(); // image to world
    Eigen::Matrix3d R_image_world = Eigen::Matrix3d::Zero();

    R_image_world(0, 2) = 1.0;
    R_image_world(1, 0) = -1.0;
    R_image_world(2, 1) = -1.0;

    T_world_image_.rotate(R_image_world);

    aruco_marker_detector_ptr_ = std::make_shared<ArucoMarkerDetector>(
      camera_intrinsic_,
      camera_dist_coeffs_,
      aurco_marker_dictionary_id_
    );

    Eigen::Vector3d initial_pose;    
    Eigen::Matrix3d initial_conv;    
    Eigen::Matrix3d predict_noise;    
    Eigen::Matrix3d measure_noise;

    initial_pose << 0, 0, 0;
    initial_conv << 0.01, 0, 0,
                    0, 0.01, 0,
                    0, 0, 0.005;

    predict_noise << 0.005, 0, 0,
                    0, 0.005, 0,
                    0, 0, 0.001;

    measure_noise << 0.005, 0, 0,
                    0, 0.005, 0,
                    0, 0, 0.18;

    aruco_kalman_filter_ptr_ = std::make_shared<ArucoKalmanFilter>(initial_pose, initial_conv, predict_noise, measure_noise);

    has_detected_marker_ = false;

    timer_ = this->create_wall_timer(
            std::chrono::microseconds(30), 
            std::bind(&AutoMarkerDockingNode::timer_callback, this));
  }

private:
  void image_callback_(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat cv_image = cv_ptr->image;
      detect_aruco_marker(cv_image);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.",
                   msg->encoding.c_str());
    }
  }

  void timer_callback()
  {
    if (has_detected_marker_) {
      Eigen::Vector3d state_predict = Eigen::Vector3d::Zero();
      aruco_kalman_filter_ptr_->predict(state_predict);
      Eigen::Isometry3d aruco_waypoint_pose = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d filter_marker_pose = aruco_kalman_filter_ptr_->get_state();

      Eigen::Vector3d euler = filter_marker_pose.rotation().eulerAngles(2, 1, 0);
      Eigen::Vector3d translation = filter_marker_pose.translation();

      double yaw = euler[0];

      aruco_waypoint_pose.translate(Eigen::Vector3d(0, 0.5, 0));
      // std::cout << "filter: " << filter_marker_pose << std::endl;
      // aruco_waypoint.translate(Eigen::Vector3d(0, waypoint_translation[1] * -1, 0));
      // Convert Eigen::Isometry3d to geometry_msgs::msg::TransformStamped
      send_aruco_transform(filter_marker_pose, "camera_link", "filter_aruco_link");
      send_aruco_transform(detect_marker_pose_, "camera_link", "aruco_link");
      send_aruco_transform(aruco_waypoint_pose, "aruco_link", "aruco_link_3m");
      // send_aruco_transform(aruco_waypoint_pose, "aruco_link", "aruco_waypoint");
      
    }
      // 타이머 콜백에서 수행할 작업을 여기에 추가하세요.
  }

  void detect_aruco_marker(const cv::Mat& image) {
    auto is_success = aruco_marker_detector_ptr_->detectMarkers(image);

    if (is_success) {
      detect_marker_pose_ = Eigen::Isometry3d::Identity();

      auto marker_rotates = aruco_marker_detector_ptr_->getRotationVectors();
      auto marker_translates = aruco_marker_detector_ptr_->getTranslationVectors();

      auto marker_translate = marker_translates[0];
      auto marker_rotate = marker_rotates[0];

      Eigen::Vector3d translation(marker_translate[0], 0.0, marker_translate[2]);
      Eigen::AngleAxisd rotation(radian_normalization(marker_rotate[1]), Eigen::Vector3d::UnitZ());

      detect_marker_pose_.translate(T_world_image_ * translation);
      detect_marker_pose_.rotate(rotation);

      detect_marker_pose_ = detect_marker_pose_;
      has_detected_marker_ = true;

      Eigen::Vector3d update_state(detect_marker_pose_.translation().x(), detect_marker_pose_.translation().y(), marker_rotate[1]);
      aruco_kalman_filter_ptr_->update(update_state);
    } else {
      has_detected_marker_ = false;
      RCLCPP_INFO(this->get_logger(), "no detect aruco marker!!");
    }
  }

  void send_aruco_transform(const Eigen::Isometry3d& T, std::string header_frame_id, std::string child_frame_id)
  {
    geometry_msgs::msg::TransformStamped tfs = tf2::eigenToTransform(T);
    // Set the frame_id and child_frame_id
    tfs.header.frame_id = header_frame_id;
    tfs.child_frame_id = child_frame_id;
    tfs.header.stamp = this->get_clock()->now();

    // Broadcast the transform
    tf_broadcaster_->sendTransform(tfs);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_detected_marker_;

  cv::Mat camera_intrinsic_;
  cv::Mat camera_dist_coeffs_;
  double aruco_marker_size_;
  int aurco_marker_dictionary_id_;

  Eigen::Isometry3d detect_marker_pose_;
  Eigen::Isometry3d T_world_image_;

  std::shared_ptr<ArucoMarkerDetector> aruco_marker_detector_ptr_;
  std::shared_ptr<ArucoKalmanFilter> aruco_kalman_filter_ptr_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoMarkerDockingNode>());
  rclcpp::shutdown();

  return 0;
}
