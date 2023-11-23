#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <cstdio>
#include <cmath>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_eigen/tf2_eigen.h>

#include "auto_marker_docking_interface/action/marker_docking.hpp"

#include "auto_marker_docking/aruco_marker_detector.hpp"
#include "auto_marker_docking/aruco_kalman_filter.hpp"
#include "auto_marker_docking/PD_controller.hpp"
#include "auto_marker_docking/utils.hpp"

class AutoMarkerDockingActionServer : public rclcpp::Node {
public:
  enum Step {
      DETECTION,
      WAYPOINT_1,
      WAYPOINT_2,
      ARUCO_1,
      ARUCO_2,
      END
  };

  using MarkerDocking = auto_marker_docking_interface::action::MarkerDocking;
  using GoalHandleMarkerDocking = rclcpp_action::ServerGoalHandle<MarkerDocking>;

  AutoMarkerDockingActionServer() : Node("auto_marker_docking_action_server") {
    RCLCPP_INFO(this->get_logger(), "===== auto_marker_docking_action_server configure =====");

    configure();

    current_step_ = DETECTION;

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, 10,
        std::bind(&AutoMarkerDockingActionServer::image_callback_, this,
                  std::placeholders::_1));
    
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<MarkerDocking>(
      this,
      "aruco_marker_docking",
      std::bind(&AutoMarkerDockingActionServer::handle_goal, this, _1, _2),
      std::bind(&AutoMarkerDockingActionServer::handle_cancel, this, _1),
      std::bind(&AutoMarkerDockingActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<MarkerDocking>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MarkerDocking::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with marker robot gap %f m", goal->marker_gap);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMarkerDocking> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMarkerDocking> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&AutoMarkerDockingActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMarkerDocking> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing docking");
    detected_count_ = 0;
    current_step_ = DETECTION;
    rclcpp::Rate loop_rate(std::chrono::milliseconds(30));
    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<MarkerDocking::Feedback>();
    auto result = std::make_shared<MarkerDocking::Result>();

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Docking canceled");

        return;
      }

      if (has_detected_marker_) {
        Eigen::Vector3d state_predict = get_predict_vector(0.03);
        aruco_kalman_filter_ptr_->predict(state_predict);

        Eigen::Isometry3d filter_marker_pose = aruco_kalman_filter_ptr_->get_state();
        Eigen::Isometry3d T_aruco_waypoint = Eigen::Isometry3d::Identity();

        T_aruco_waypoint.translate(Eigen::Vector3d(0.4, 0.0, 0.0));
        Eigen::Isometry3d aruco_waypoint_pose  = filter_marker_pose * T_aruco_waypoint;

        auto translation = aruco_waypoint_pose.translation();
        auto euler = aruco_waypoint_pose.rotation().eulerAngles(2, 1, 0);

        if (current_step_ == ARUCO_1 || current_step_ == ARUCO_2) {
          translation = filter_marker_pose.translation();
          euler = filter_marker_pose.rotation().eulerAngles(2, 1, 0);
        }

        Eigen::Vector3d output = PD_controller_ptr_->update(
          Eigen::Vector3d(translation(0) * -1.0, translation(1) * -1.0, euler[0]),
          30
        );
        Eigen::Vector2d velocity = xy_to_polar_coordinates(output(0), output(1));

        if (velocity(0) < max_linear_vel_ * -1) {
          velocity(0) = max_linear_vel_ * -1;
        }
        if (velocity(0) > max_linear_vel_) {
          velocity(0) = max_linear_vel_;
        }

        if (velocity(1) < max_anguler_vel_ * -1) {
          velocity(1) = max_anguler_vel_ * -1;
        }
        if (velocity(1) > max_anguler_vel_) {
          velocity(1) = max_anguler_vel_;
        }
        

        if (current_step_ == DETECTION) {
          if (detected_count_ >= 100) {
            current_step_ = WAYPOINT_1;
            RCLCPP_INFO(this->get_logger(), "change step: DETECTION -> WAYPOINT_1");

          }
        } else if (current_step_ == WAYPOINT_1) {
          robot_velocity_input_(0) = 0.0;
          robot_velocity_input_(1) = velocity(1);

          if (std::abs(translation(1)) <= 0.02) {
            current_step_ = WAYPOINT_2;
            RCLCPP_INFO(this->get_logger(), "change step: WAYPOINT_1 -> WAYPOINT_2");
          }
        } else if (current_step_ == WAYPOINT_2) {
          robot_velocity_input_(0) = velocity(0);
          robot_velocity_input_(1) = 0.0;

          if (std::abs(translation(0)) <= 0.02) {
            current_step_ = ARUCO_1;
            RCLCPP_INFO(this->get_logger(), "change step: WAYPOINT_2 -> ARUCO_1");
          }
        } else if (current_step_ == ARUCO_1) {
          robot_velocity_input_(0) = 0.0;
          robot_velocity_input_(1) = velocity(1);

          if (std::abs(translation(1)) <= 0.02) {
            current_step_ = ARUCO_2;
            RCLCPP_INFO(this->get_logger(), "change step: ARUCO_1 -> ARUCO_2");
          }
        } else if (current_step_ == ARUCO_2) {
          bool linear_finish = false;
          bool anguler_finish = false;
          robot_velocity_input_(0) = velocity(0);
          robot_velocity_input_(1) = velocity(1);

          if (std::abs(translation(0)) <= goal->marker_gap) {
            linear_finish = true;
            robot_velocity_input_(0) = 0.0;
          }

          if (std::abs(translation(1)) <= 0.02) {
            anguler_finish = true;
            robot_velocity_input_(1) = 0.0;
          }

          if (linear_finish && anguler_finish) {
            current_step_ = END;
            RCLCPP_INFO(this->get_logger(), "change step: ARUCO_2 -> END");
          }
        } else {
          robot_velocity_input_(0) = 0.0;
          robot_velocity_input_(1) = 0.0;
          publish_cmd_vel();
          break;
        }

        publish_cmd_vel();
        send_aruco_transform(filter_marker_pose, camera_link_name_, "filter_aruco_link");
        send_aruco_transform(aruco_waypoint_pose, camera_link_name_, "aruco_waypoint1"); 
      }

      feedback->step = static_cast<int>(current_step_);
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

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

  void configure() {
    std::vector<double> camera_intrinsic_values = {
      1696.80268, 0.0, 960.5,
      0.0, 1696.80268, 540.5,
      0.0, 0.0, 1.0
    };;
    std::vector<double> camera_dist_coeffs_values = {0.0, 0.0, 0.0, 0.0, 0.0};

    std::vector<double> initial_covariance = {
      0.01, 0.0, 0.0,
      0.0, 0.01, 0.0,
      0.0, 0.0, 0.005
    };
    std::vector<double> predict_covariance = {
      0.005, 0.0, 0.0,
      0.0, 0.005, 0.0,
      0.0, 0.0, 0.001
    };
    std::vector<double> measure_covariance = {
      0.005, 0.0, 0.0,
      0.0, 0.005, 0.0,
      0.0, 0.0, 0.18
    };;

    std::vector<double> P_gain = {0.25, 0.25, 0.25};
    std::vector<double> D_gain = {0.01, 0.01, 0.01};

    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->get_parameter("camera_topic", camera_topic_);

    std::cout  <<  "camera_topic: " << camera_topic_ << std::endl;

    this->declare_parameter("camera_link_name", "camera_link");
    this->get_parameter("camera_link_name", camera_link_name_);

    this->declare_parameter("camera_intrinsic_matrix", camera_intrinsic_values);
    this->get_parameter("camera_intrinsic_matrix", camera_intrinsic_values);

    this->declare_parameter("camera_dist_coeffs", camera_dist_coeffs_values);
    this->get_parameter("camera_dist_coeffs", camera_dist_coeffs_values);
    
    this->declare_parameter("aruco_marker_size", 0.1);
    this->get_parameter("aruco_marker_size", aruco_marker_size_);

    this->declare_parameter("aurco_marker_dictionary_id", 10);
    this->get_parameter("aurco_marker_dictionary_id", aurco_marker_dictionary_id_);

    this->declare_parameter("initial_covariance", initial_covariance);
    this->get_parameter("initial_covariance", initial_covariance);

    this->get_parameter("predict_covariance", predict_covariance);
    this->declare_parameter("predict_covariance", predict_covariance);

    this->get_parameter("measure_covariance", measure_covariance);
    this->declare_parameter("measure_covariance", measure_covariance);

    this->declare_parameter("cmd_vel_topic", "cmd_vel");
    this->get_parameter("cmd_vel_topic", cmd_vel_topic_);

    this->declare_parameter("max_anguler_vel", 0.25);
    this->get_parameter("max_anguler_vel", max_anguler_vel_);

    this->declare_parameter("max_linear_vel", 0.3);
    this->get_parameter("max_linear_vel", max_linear_vel_);

    this->declare_parameter("P_gain", P_gain);
    this->get_parameter("P_gain", P_gain);

    this->declare_parameter("D_gain", D_gain);
    this->get_parameter("D_gain", D_gain);

    camera_intrinsic_ = cv::Mat(3, 3, CV_64F, camera_intrinsic_values.data());
    camera_dist_coeffs_ = cv::Mat(1, 5, CV_64F, camera_dist_coeffs_values.data());
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

    initial_pose << 0.0, 0.0, 0.0;
    initial_conv << initial_covariance[0], 0.0, 0.0,
                    0.0, initial_covariance[4], 0.0,
                    0.0, 0.0, initial_covariance[8];

    predict_noise << predict_covariance[0], 0.0, 0.0,
                    0.0, predict_covariance[4], 0.0,
                    0.0, 0.0, predict_covariance[8];

    measure_noise << measure_covariance[0], 0.0, 0.0,
                    0.0, measure_covariance[4], 0.0,
                    0.0, 0.0, measure_covariance[8];

    aruco_kalman_filter_ptr_ = std::make_shared<ArucoKalmanFilter>(initial_pose, initial_conv, predict_noise, measure_noise);

    // PD Conotroller setup
    Eigen::Vector3d p_gain;    
    Eigen::Vector3d d_gain;

    p_gain << P_gain[0], P_gain[1], P_gain[2];    
    d_gain << D_gain[0], D_gain[1], D_gain[2];

    PD_controller_ptr_ = std::make_shared<PDController>(p_gain, d_gain);
    PD_controller_ptr_->set_target(Eigen::Vector3d(0.0, 0.0, 0.0));

    robot_pose_ = Eigen::Isometry3d();
    robot_velocity_input_ = Eigen::Vector2d::Zero();

    has_detected_marker_ = false;
    detected_count_ = 0;
  }

  void publish_cmd_vel()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = robot_velocity_input_(0);
    message.angular.z = robot_velocity_input_(1);

    cmd_vel_publisher_->publish(message);
  }

  void detect_aruco_marker(const cv::Mat& image) {
    auto is_success = aruco_marker_detector_ptr_->detectMarkers(image);

    if (is_success) {
      detected_count_ += 1;
      detect_marker_pose_ = Eigen::Isometry3d::Identity();

      auto marker_rotates = aruco_marker_detector_ptr_->getRotationVectors();
      auto marker_translates = aruco_marker_detector_ptr_->getTranslationVectors();

      auto marker_translate = marker_translates[0];
      auto marker_rotate = marker_rotates[0];

      marker_rotate[1] = marker_rotate[1] * -1;

      Eigen::Vector3d translation(marker_translate[0], 0.0, marker_translate[2]);
      Eigen::AngleAxisd rotation(radian_normalization(marker_rotate[1]), Eigen::Vector3d::UnitZ());

      detect_marker_pose_.translate(T_world_image_ * translation);
      detect_marker_pose_.rotate(rotation);

      detect_marker_pose_ = detect_marker_pose_;
      has_detected_marker_ = true;

      Eigen::Vector3d update_state(detect_marker_pose_.translation().x(), detect_marker_pose_.translation().y(), marker_rotate[1]);
      aruco_kalman_filter_ptr_->update(update_state);
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

  Eigen::Vector3d get_predict_vector(double dt)
  {
    Eigen::Isometry3d pose_p = aruco_kalman_filter_ptr_->get_state();
    auto euler = pose_p.rotation().eulerAngles(2, 1, 0);
    Eigen::Vector2d measure_polar = xy_to_polar_coordinates(
      pose_p.translation().x(), pose_p.translation().y()
    );

    Eigen::Vector3d state_predict = Eigen::Vector3d::Zero();
    double r = measure_polar(0);
    double delta = measure_polar(1);
    double linear = robot_velocity_input_(0);
    double anguler = robot_velocity_input_(1);
    double yaw = euler[0];

    if (yaw < Half_PI) {
      yaw = yaw + PI;
    }
    if (yaw < 0 && Half_PI * -1 < yaw) {
      yaw = yaw - PI;
    }

    double new_delta = delta + anguler * dt;
    state_predict(0) = (r * std::cos(new_delta) - pose_p.translation().x()) + (linear * dt * std::cos(0));
    state_predict(1) = (r * std::sin(new_delta) - pose_p.translation().y()) + (linear * dt * std::sin(0));
    state_predict(2) = robot_velocity_input_(1) * dt;

    return state_predict;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string camera_topic_;
  std::string camera_link_name_;
  bool has_detected_marker_;
  int detected_count_;

  std::string cmd_vel_topic_;
  double max_anguler_vel_;
  double max_linear_vel_;

  cv::Mat camera_intrinsic_;
  cv::Mat camera_dist_coeffs_;
  double aruco_marker_size_;
  int aurco_marker_dictionary_id_;

  Eigen::Isometry3d detect_marker_pose_;
  Eigen::Isometry3d T_world_image_;
  Eigen::Isometry3d robot_pose_;
  Eigen::Vector2d robot_velocity_input_;

  std::shared_ptr<ArucoMarkerDetector> aruco_marker_detector_ptr_;
  std::shared_ptr<ArucoKalmanFilter> aruco_kalman_filter_ptr_;
  std::shared_ptr<PDController> PD_controller_ptr_;

  Step current_step_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoMarkerDockingActionServer>());
  rclcpp::shutdown();

  return 0;
}
