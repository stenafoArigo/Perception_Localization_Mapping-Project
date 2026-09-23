/*
 * stageros ROS 2 compatibility port
 *
 * This file is a ROS 2-oriented port of the classic ROS 1 stage_ros/stageros
 * bridge behavior. The original stageros implementation was released under
 * GPL-2.0-or-later terms; keep this file under the same license if you publish
 * derived work.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// Include ROS 2 headers before Stage/FLTK/Xlib.
// Xlib defines a global macro named `None`, which collides with
// rclcpp::SignalHandlerOptions::None if rclcpp is included afterwards.
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <stage.hh>

// Keep the Xlib macro from leaking into the rest of this translation unit.
#ifdef None
#undef None
#endif

namespace stage_ros2_compat
{

constexpr const char * kImage = "image";
constexpr const char * kDepth = "depth";
constexpr const char * kCameraInfo = "camera_info";
constexpr const char * kOdom = "odom";
constexpr const char * kBaseScan = "base_scan";
constexpr const char * kGroundTruth = "base_pose_ground_truth";
constexpr const char * kCmdVel = "cmd_vel";
constexpr double kPi = 3.141592653589793238462643383279502884;

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

void flip_image_rows(std::vector<uint8_t> & data, size_t row_bytes, size_t height)
{
  if (row_bytes == 0 || height < 2 || data.size() < row_bytes * height) {
    return;
  }

  std::vector<uint8_t> tmp(row_bytes);
  for (size_t y = 0; y < height / 2; ++y) {
    uint8_t * top = data.data() + y * row_bytes;
    uint8_t * bottom = data.data() + (height - 1 - y) * row_bytes;
    std::memcpy(tmp.data(), top, row_bytes);
    std::memcpy(top, bottom, row_bytes);
    std::memcpy(bottom, tmp.data(), row_bytes);
  }
}

bool truthy(const std::string & value)
{
  return value == "1" || value == "true" || value == "TRUE" || value == "True" ||
         value == "yes" || value == "YES" || value == "on" || value == "ON";
}

class StageRos2 final : public rclcpp::Node
{
public:
  StageRos2(
    int stage_argc, char ** stage_argv, bool gui, const std::string & world_file,
    bool use_model_names_from_cli)
  : rclcpp::Node("stageros"),
    base_last_cmd_(0, 0, RCL_ROS_TIME),
    sim_time_(0, 0, RCL_ROS_TIME),
    base_last_globalpos_time_(0, 0, RCL_ROS_TIME)
  {
    const double watchdog_seconds = this->declare_parameter<double>("base_watchdog_timeout", 0.2);
    base_watchdog_timeout_ = rclcpp::Duration::from_seconds(watchdog_seconds);
    is_depth_canonical_ = this->declare_parameter<bool>("is_depth_canonical", true);
    use_model_names_ = this->declare_parameter<bool>("use_model_names", use_model_names_from_cli);
    delay_odom_tf_by_one_update_ = this->declare_parameter<bool>("delay_odom_tf_by_one_update", true);
    if (delay_odom_tf_by_one_update_) {
      RCLCPP_WARN(
        this->get_logger(),
        "delay_odom_tf_by_one_update is enabled: odom/base TF will use the previous Stage pose to match one-update-delayed LaserScan data");
    }
    if (!std::filesystem::exists(world_file)) {
      throw std::runtime_error("Stage world file does not exist: " + world_file);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    Stg::Init(&stage_argc, &stage_argv);
    if (gui) {
      world_.reset(new Stg::WorldGui(600, 400, "Stage (ROS 2)"));
    } else {
      world_.reset(new Stg::World());
    }

    world_->Load(world_file.c_str());
    world_->AddUpdateCallback(reinterpret_cast<Stg::world_callback_t>(StageRos2::stage_update_cb), this);
    world_->ForEachDescendant(reinterpret_cast<Stg::model_callback_t>(StageRos2::find_model_cb), this);
  }

  ~StageRos2() override = default;

  Stg::World * world()
  {
    return world_.get();
  }

  bool subscribe_models()
  {
    if (positionmodels_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "No Stage position models found in the world file");
      return false;
    }

    for (size_t r = 0; r < positionmodels_.size(); ++r) {
      auto robot = std::make_unique<StageRobot>();
      robot->positionmodel = positionmodels_[r];
      robot->positionmodel->Subscribe();

      RCLCPP_INFO(
        this->get_logger(), "Subscribed to Stage position model '%s'",
        robot->positionmodel->Token());

      for (auto * laser : lasermodels_) {
        if (laser != nullptr && laser->Parent() == robot->positionmodel) {
          robot->lasermodels.push_back(laser);
          laser->Subscribe();
          RCLCPP_INFO(this->get_logger(), "Subscribed to Stage ranger '%s'", laser->Token());
        }
      }

      for (auto * camera : cameramodels_) {
        if (camera != nullptr && camera->Parent() == robot->positionmodel) {
          robot->cameramodels.push_back(camera);
          camera->Subscribe();
          RCLCPP_INFO(this->get_logger(), "Subscribed to Stage camera '%s'", camera->Token());
        }
      }

      RCLCPP_INFO(
        this->get_logger(), "Robot '%s' has %zu ranger(s) and %zu camera(s)",
        robot->positionmodel->Token(), robot->lasermodels.size(), robot->cameramodels.size());

      robot->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        map_name(kOdom, r, robot->positionmodel), rclcpp::QoS(10));
      robot->ground_truth_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        map_name(kGroundTruth, r, robot->positionmodel), rclcpp::QoS(10));
      robot->cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        map_name(kCmdVel, r, robot->positionmodel), rclcpp::QoS(10),
        [this, r](geometry_msgs::msg::Twist::SharedPtr msg) {cmdvel_received(r, msg);});

      for (size_t s = 0; s < robot->lasermodels.size(); ++s) {
        const std::string topic = robot->lasermodels.size() == 1 ?
          map_name(kBaseScan, r, robot->positionmodel) :
          map_name(kBaseScan, r, s, robot->positionmodel);
        robot->laser_pubs.push_back(
          this->create_publisher<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS()));
      }

      for (size_t s = 0; s < robot->cameramodels.size(); ++s) {
        if (robot->cameramodels.size() == 1) {
          robot->image_pubs.push_back(
            this->create_publisher<sensor_msgs::msg::Image>(
              map_name(kImage, r, robot->positionmodel), rclcpp::SensorDataQoS()));
          robot->depth_pubs.push_back(
            this->create_publisher<sensor_msgs::msg::Image>(
              map_name(kDepth, r, robot->positionmodel), rclcpp::SensorDataQoS()));
          robot->camera_info_pubs.push_back(
            this->create_publisher<sensor_msgs::msg::CameraInfo>(
              map_name(kCameraInfo, r, robot->positionmodel), rclcpp::QoS(10)));
        } else {
          robot->image_pubs.push_back(
            this->create_publisher<sensor_msgs::msg::Image>(
              map_name(kImage, r, s, robot->positionmodel), rclcpp::SensorDataQoS()));
          robot->depth_pubs.push_back(
            this->create_publisher<sensor_msgs::msg::Image>(
              map_name(kDepth, r, s, robot->positionmodel), rclcpp::SensorDataQoS()));
          robot->camera_info_pubs.push_back(
            this->create_publisher<sensor_msgs::msg::CameraInfo>(
              map_name(kCameraInfo, r, s, robot->positionmodel), rclcpp::QoS(10)));
        }
      }

      robotmodels_.push_back(std::move(robot));
    }

    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset_positions",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        reset_positions(request, response);
      });

    if (lasermodels_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No Stage ranger models found; no LaserScan topics will be published");
    }
    if (cameramodels_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No Stage camera models found; no image topics will be published");
    }

    return true;
  }

private:
  struct StageRobot
  {
    Stg::ModelPosition * positionmodel {nullptr};
    std::vector<Stg::ModelCamera *> cameramodels;
    std::vector<Stg::ModelRanger *> lasermodels;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_pubs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pubs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> laser_pubs;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;

    // Some Stage configurations expose LaserScan data one world update later than
    // the current position model pose. When delay_odom_tf_by_one_update is true,
    // this cached pose is used for odom/base TF so the scan and TF describe the
    // same robot orientation.
    bool have_previous_pose_for_odom_tf {false};
    Stg::Pose previous_pose_for_odom_tf;
  };

  static bool stage_update_cb(Stg::World *, StageRos2 * node)
  {
    node->world_callback();
    return false;
  }

  static void find_model_cb(Stg::Model * mod, StageRos2 * node)
  {
    if (auto * ranger = dynamic_cast<Stg::ModelRanger *>(mod)) {
      node->lasermodels_.push_back(ranger);
    }
    if (auto * position = dynamic_cast<Stg::ModelPosition *>(mod)) {
      node->positionmodels_.push_back(position);
      node->initial_poses_.push_back(position->GetGlobalPose());
    }
    if (auto * camera = dynamic_cast<Stg::ModelCamera *>(mod)) {
      node->cameramodels_.push_back(camera);
    }
  }

  std::string model_token(const Stg::Model * mod) const
  {
    if (mod == nullptr) {
      return {};
    }
    auto * ancestor = reinterpret_cast<Stg::Ancestor *>(const_cast<Stg::Model *>(mod));
    const char * token = ancestor->Token();
    return token == nullptr ? std::string() : std::string(token);
  }

  std::string map_name(const std::string & name, size_t robot_id, const Stg::Model * mod) const
  {
    if (positionmodels_.size() <= 1 && !use_model_names_) {
      return name;
    }

    const std::string token = model_token(mod);
    if (use_model_names_ && !token.empty() && token.find(':') == std::string::npos) {
      return token + "/" + name;
    }

    return "robot_" + std::to_string(robot_id) + "/" + name;
  }

  std::string map_name(
    const std::string & name, size_t robot_id, size_t device_id, const Stg::Model * mod) const
  {
    if (positionmodels_.size() <= 1 && !use_model_names_) {
      return name + "_" + std::to_string(device_id);
    }

    const std::string token = model_token(mod);
    if (use_model_names_ && !token.empty() && token.find(':') == std::string::npos) {
      return token + "/" + name + "_" + std::to_string(device_id);
    }

    return "robot_" + std::to_string(robot_id) + "/" + name + "_" + std::to_string(device_id);
  }

  void reset_positions(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    std::lock_guard<std::mutex> lock(msg_lock_);
    RCLCPP_INFO(this->get_logger(), "Resetting Stage robot poses");
    for (size_t r = 0; r < positionmodels_.size(); ++r) {
      positionmodels_[r]->SetPose(initial_poses_[r]);
      positionmodels_[r]->SetStall(false);
    }
    for (auto & robot : robotmodels_) {
      robot->have_previous_pose_for_odom_tf = false;
    }
    base_last_globalpos_.clear();
  }

  void cmdvel_received(size_t idx, const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(msg_lock_);
    if (idx >= positionmodels_.size()) {
      return;
    }
    positionmodels_[idx]->SetSpeed(msg->linear.x, msg->linear.y, msg->angular.z);
    base_last_cmd_ = sim_time_;
  }

  void world_callback()
  {
    if (!rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "rclcpp is shutting down; stopping Stage");
      world_->QuitAll();
      return;
    }

    std::lock_guard<std::mutex> lock(msg_lock_);

    const int64_t sim_nsec = static_cast<int64_t>(world_->SimTimeNow()) * 1000LL;
    sim_time_ = rclcpp::Time(sim_nsec, RCL_ROS_TIME);

    // ROS treats zero time specially; mimic the ROS 1 bridge and skip it.
    if (sim_time_.nanoseconds() == 0) {
      return;
    }

    if (base_watchdog_timeout_.seconds() > 0.0 &&
      (sim_time_ - base_last_cmd_) >= base_watchdog_timeout_)
    {
      for (auto * position : positionmodels_) {
        position->SetSpeed(0.0, 0.0, 0.0);
      }
    }

    for (size_t r = 0; r < robotmodels_.size(); ++r) {
      StageRobot * robot = robotmodels_[r].get();
      publish_base_frames_and_odom(*robot, r);
      publish_lasers(*robot, r);
      publish_cameras(*robot, r);
    }

    for (auto & robot : robotmodels_) {
      update_previous_pose_for_odom_tf(*robot);
    }

    base_last_globalpos_time_ = sim_time_;

    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = sim_time_;
    clock_pub_->publish(clock_msg);
  }

  void publish_lasers(StageRobot & robot, size_t robot_id)
  {
    for (size_t s = 0; s < robot.lasermodels.size(); ++s) {
      Stg::ModelRanger * laser = robot.lasermodels[s];
      const std::vector<Stg::ModelRanger::Sensor> & sensors = laser->GetSensors();
      if (sensors.empty()) {
        continue;
      }
      if (sensors.size() > 1) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "This compatibility bridge supports only the first sensor of a Stage ranger");
      }

      const Stg::ModelRanger::Sensor & sensor = sensors[0];
      const std::string laser_frame = robot.lasermodels.size() > 1 ?
        map_name("base_laser_link", robot_id, s, robot.positionmodel) :
        map_name("base_laser_link", robot_id, robot.positionmodel);

      Stg::Pose lp = laser->GetPose();
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, lp.a);
      q.normalize();
      send_transform(
        map_name("base_link", robot_id, robot.positionmodel), laser_frame,
        lp.x, lp.y, robot.positionmodel->GetGeom().size.z + lp.z, q);

      if (!sensor.ranges.empty()) {
        sensor_msgs::msg::LaserScan msg;
        msg.header.stamp = sim_time_;
        msg.header.frame_id = laser_frame;
        msg.angle_min = -sensor.fov / 2.0;
        msg.angle_max = sensor.fov / 2.0;
        msg.angle_increment = sensor.sample_count > 1 ?
          sensor.fov / static_cast<double>(sensor.sample_count - 1) : 0.0;
        msg.range_min = sensor.range.min;
        msg.range_max = sensor.range.max;
        msg.ranges.assign(sensor.ranges.begin(), sensor.ranges.end());
        msg.intensities.assign(sensor.intensities.begin(), sensor.intensities.end());
        robot.laser_pubs[s]->publish(msg);
      }
    }
  }

  void publish_base_frames_and_odom(StageRobot & robot, size_t robot_id)
  {
    const std::string odom_frame = map_name("odom", robot_id, robot.positionmodel);
    const std::string base_footprint_frame = map_name("base_footprint", robot_id, robot.positionmodel);
    const std::string base_link_frame = map_name("base_link", robot_id, robot.positionmodel);
    const Stg::Pose pose_for_odom_tf = get_pose_for_odom_tf(robot);

    tf2::Quaternion identity;
    identity.setRPY(0.0, 0.0, 0.0);
    identity.normalize();
    send_transform(base_footprint_frame, base_link_frame, 0.0, 0.0, 0.0, identity);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = sim_time_;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_footprint_frame;
    odom_msg.pose.pose.position.x = pose_for_odom_tf.x;
    odom_msg.pose.pose.position.y = pose_for_odom_tf.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = yaw_to_quaternion(pose_for_odom_tf.a);

    Stg::Velocity v = robot.positionmodel->GetVelocity();
    odom_msg.twist.twist.linear.x = v.x;
    odom_msg.twist.twist.linear.y = v.y;
    odom_msg.twist.twist.angular.z = v.a;
    robot.odom_pub->publish(odom_msg);

    tf2::Quaternion odom_q;
    odom_q.setRPY(0.0, 0.0, pose_for_odom_tf.a);
    odom_q.normalize();
    send_transform(
      odom_frame, base_footprint_frame,
      pose_for_odom_tf.x, pose_for_odom_tf.y, 0.0, odom_q);

    publish_ground_truth(robot, robot_id, odom_frame);
  }

  Stg::Pose get_pose_for_odom_tf(StageRobot & robot)
  {
    const Stg::Pose current_pose = robot.positionmodel->GetGlobalPose();
    if (!delay_odom_tf_by_one_update_ || !robot.have_previous_pose_for_odom_tf) {
      return current_pose;
    }
    return robot.previous_pose_for_odom_tf;
  }

  void update_previous_pose_for_odom_tf(StageRobot & robot)
  {
    robot.previous_pose_for_odom_tf = robot.positionmodel->GetGlobalPose();
    robot.have_previous_pose_for_odom_tf = true;
  }

  void publish_ground_truth(StageRobot & robot, size_t robot_id, const std::string & odom_frame)
  {
    Stg::Pose gpose = robot.positionmodel->GetGlobalPose();

    Stg::Velocity gvel(0, 0, 0, 0);
    if (base_last_globalpos_.size() > robot_id) {
      const Stg::Pose prevpose = base_last_globalpos_.at(robot_id);
      const double dt = (sim_time_ - base_last_globalpos_time_).seconds();
      if (dt > 0.0) {
        gvel = Stg::Velocity(
          (gpose.x - prevpose.x) / dt,
          (gpose.y - prevpose.y) / dt,
          (gpose.z - prevpose.z) / dt,
          Stg::normalize(gpose.a - prevpose.a) / dt);
      }
      base_last_globalpos_.at(robot_id) = gpose;
    } else {
      base_last_globalpos_.push_back(gpose);
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, gpose.a);
    q.normalize();

    nav_msgs::msg::Odometry ground_truth_msg;
    ground_truth_msg.header.stamp = sim_time_;
    ground_truth_msg.header.frame_id = odom_frame;
    ground_truth_msg.child_frame_id = map_name("base_footprint", robot_id, robot.positionmodel);
    ground_truth_msg.pose.pose.position.x = gpose.x;
    ground_truth_msg.pose.pose.position.y = gpose.y;
    ground_truth_msg.pose.pose.position.z = gpose.z;
    ground_truth_msg.pose.pose.orientation = tf2::toMsg(q);
    ground_truth_msg.twist.twist.linear.x = gvel.x;
    ground_truth_msg.twist.twist.linear.y = gvel.y;
    ground_truth_msg.twist.twist.linear.z = gvel.z;
    ground_truth_msg.twist.twist.angular.z = gvel.a;
    robot.ground_truth_pub->publish(ground_truth_msg);
  }

  void publish_cameras(StageRobot & robot, size_t robot_id)
  {
    for (size_t s = 0; s < robot.cameramodels.size(); ++s) {
      Stg::ModelCamera * camera = robot.cameramodels[s];
      const std::string camera_frame = robot.cameramodels.size() > 1 ?
        map_name("camera", robot_id, s, robot.positionmodel) :
        map_name("camera", robot_id, robot.positionmodel);

      const bool has_color_subscribers = robot.image_pubs[s]->get_subscription_count() > 0;
      const bool has_depth_subscribers = robot.depth_pubs[s]->get_subscription_count() > 0;
      const bool has_info_subscribers = robot.camera_info_pubs[s]->get_subscription_count() > 0;

      if (has_color_subscribers && camera->FrameColor() != nullptr) {
        publish_color_image(*robot.image_pubs[s], *camera, camera_frame);
      }

      if (has_depth_subscribers && camera->FrameDepth() != nullptr) {
        publish_depth_image(*robot.depth_pubs[s], *camera, camera_frame);
      }

      if ((has_color_subscribers && camera->FrameColor() != nullptr) ||
        (has_depth_subscribers && camera->FrameDepth() != nullptr) || has_info_subscribers)
      {
        publish_camera_tf_and_info(
          *robot.camera_info_pubs[s], *camera, camera_frame,
          map_name("base_link", robot_id, robot.positionmodel), robot.positionmodel);
      }
    }
  }

  void publish_color_image(
    rclcpp::Publisher<sensor_msgs::msg::Image> & pub, Stg::ModelCamera & camera,
    const std::string & camera_frame)
  {
    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = sim_time_;
    image_msg.header.frame_id = camera_frame;
    image_msg.height = camera.getHeight();
    image_msg.width = camera.getWidth();
    image_msg.encoding = "rgba8";
    image_msg.step = image_msg.width * 4;
    image_msg.data.resize(static_cast<size_t>(image_msg.step) * image_msg.height);
    std::memcpy(image_msg.data.data(), camera.FrameColor(), image_msg.data.size());
    flip_image_rows(image_msg.data, image_msg.step, image_msg.height);
    pub.publish(image_msg);
  }

  void publish_depth_image(
    rclcpp::Publisher<sensor_msgs::msg::Image> & pub, Stg::ModelCamera & camera,
    const std::string & camera_frame)
  {
    sensor_msgs::msg::Image depth_msg;
    depth_msg.header.stamp = sim_time_;
    depth_msg.header.frame_id = camera_frame;
    depth_msg.height = camera.getHeight();
    depth_msg.width = camera.getWidth();

    const float * depth = camera.FrameDepth();
    const size_t pixel_count = static_cast<size_t>(depth_msg.width) * depth_msg.height;

    if (is_depth_canonical_) {
      depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depth_msg.step = depth_msg.width * sizeof(float);
      depth_msg.data.resize(pixel_count * sizeof(float));
      std::memcpy(depth_msg.data.data(), depth, depth_msg.data.size());

      const float near_clip = static_cast<float>(camera.getCamera().nearClip());
      const float far_clip = static_cast<float>(camera.getCamera().farClip());
      float * data = reinterpret_cast<float *>(depth_msg.data.data());
      for (size_t i = 0; i < pixel_count; ++i) {
        if (data[i] <= near_clip) {
          data[i] = -std::numeric_limits<float>::infinity();
        } else if (data[i] >= far_clip) {
          data[i] = std::numeric_limits<float>::infinity();
        }
      }
    } else {
      depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      depth_msg.step = depth_msg.width * sizeof(uint16_t);
      depth_msg.data.resize(pixel_count * sizeof(uint16_t));

      const int near_clip_mm = static_cast<int>(camera.getCamera().nearClip() * 1000.0);
      const int far_clip_mm = static_cast<int>(camera.getCamera().farClip() * 1000.0);
      uint16_t * out = reinterpret_cast<uint16_t *>(depth_msg.data.data());
      for (size_t i = 0; i < pixel_count; ++i) {
        int value_mm = static_cast<int>(depth[i] * 1000.0);
        if (value_mm <= near_clip_mm || value_mm >= far_clip_mm) {
          value_mm = 0;
        }
        value_mm = std::clamp(value_mm, 0, static_cast<int>(std::numeric_limits<uint16_t>::max()));
        out[i] = static_cast<uint16_t>(value_mm);
      }
    }

    flip_image_rows(depth_msg.data, depth_msg.step, depth_msg.height);
    pub.publish(depth_msg);
  }

  void publish_camera_tf_and_info(
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo> & pub, Stg::ModelCamera & camera,
    const std::string & camera_frame, const std::string & base_link_frame,
    Stg::ModelPosition * position)
  {
    Stg::Pose lp = camera.GetPose();
    tf2::Quaternion q;
    q.setRPY(
      camera.getCamera().pitch() * kPi / 180.0 - kPi,
      0.0,
      lp.a + camera.getCamera().yaw() * kPi / 180.0 - position->GetPose().a);
    q.normalize();

    send_transform(
      base_link_frame, camera_frame,
      lp.x, lp.y, position->GetGeom().size.z + lp.z, q);

    sensor_msgs::msg::CameraInfo camera_msg;
    camera_msg.header.stamp = sim_time_;
    camera_msg.header.frame_id = camera_frame;
    camera_msg.height = camera.getHeight();
    camera_msg.width = camera.getWidth();
    camera_msg.distortion_model = "plumb_bob";

    const double cx = camera_msg.width / 2.0;
    const double cy = camera_msg.height / 2.0;
    const double fov_h = camera.getCamera().horizFov() * kPi / 180.0;
    const double fov_v = camera.getCamera().vertFov() * kPi / 180.0;
    const double fx = camera.getWidth() / (2.0 * std::tan(fov_h / 2.0));
    const double fy = camera.getHeight() / (2.0 * std::tan(fov_v / 2.0));

    camera_msg.d.assign(4, 0.0);
    camera_msg.k.fill(0.0);
    camera_msg.r.fill(0.0);
    camera_msg.p.fill(0.0);
    camera_msg.k[0] = fx;
    camera_msg.k[2] = cx;
    camera_msg.k[4] = fy;
    camera_msg.k[5] = cy;
    camera_msg.k[8] = 1.0;
    camera_msg.r[0] = 1.0;
    camera_msg.r[4] = 1.0;
    camera_msg.r[8] = 1.0;
    camera_msg.p[0] = fx;
    camera_msg.p[2] = cx;
    camera_msg.p[5] = fy;
    camera_msg.p[6] = cy;
    camera_msg.p[10] = 1.0;

    pub.publish(camera_msg);
  }

  void send_transform(
    const std::string & parent_frame, const std::string & child_frame,
    double x, double y, double z, const tf2::Quaternion & q)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = sim_time_;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(transform);
  }

  std::unique_ptr<Stg::World> world_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex msg_lock_;

  std::vector<Stg::ModelCamera *> cameramodels_;
  std::vector<Stg::ModelRanger *> lasermodels_;
  std::vector<Stg::ModelPosition *> positionmodels_;
  std::vector<std::unique_ptr<StageRobot>> robotmodels_;
  std::vector<Stg::Pose> initial_poses_;
  std::vector<Stg::Pose> base_last_globalpos_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  bool is_depth_canonical_ {true};
  bool use_model_names_ {false};
  bool delay_odom_tf_by_one_update_ {true};
  rclcpp::Time base_last_cmd_;
  rclcpp::Duration base_watchdog_timeout_ {0, 0};
  rclcpp::Time sim_time_;
  rclcpp::Time base_last_globalpos_time_;
};

}  // namespace stage_ros2_compat

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() < 2) {
    std::fprintf(stderr, "Usage: stageros [-g] [-u] <worldfile>\n");
    std::fprintf(stderr, "  -g: run without Stage GUI, matching ROS 1 stageros behavior\n");
    std::fprintf(stderr, "  -u: use Stage model names for ROS topic/frame prefixes\n");
    rclcpp::shutdown();
    return 1;
  }

  bool gui = true;
  bool use_model_names = false;
  std::string world_file;

  std::vector<std::string> stage_args_storage;
  stage_args_storage.push_back(args.front());

  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "-g") {
      gui = false;
      stage_args_storage.push_back(args[i]);
    } else if (args[i] == "-u") {
      use_model_names = true;
    } else {
      world_file = args[i];
      stage_args_storage.push_back(args[i]);
    }
  }

  if (world_file.empty()) {
    std::fprintf(stderr, "Usage: stageros [-g] [-u] <worldfile>\n");
    rclcpp::shutdown();
    return 1;
  }

  std::vector<char *> stage_argv;
  stage_argv.reserve(stage_args_storage.size());
  for (auto & arg : stage_args_storage) {
    stage_argv.push_back(arg.data());
  }
  int stage_argc = static_cast<int>(stage_argv.size());

  try {
    auto node = std::make_shared<stage_ros2_compat::StageRos2>(
      stage_argc, stage_argv.data(), gui, world_file, use_model_names);

    if (!node->subscribe_models()) {
      rclcpp::shutdown();
      return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    node->world()->Start();
    Stg::World::Run();

    executor.cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "stageros failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
