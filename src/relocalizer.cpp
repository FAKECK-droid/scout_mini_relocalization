#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "scout_mini_relocalization/registration.hpp"

namespace scout_mini_relocalization
{
using Steady = std::chrono::steady_clock;
using Cloud = sensor_msgs::msg::PointCloud2;
using PoseSeed = geometry_msgs::msg::PoseWithCovarianceStamped;

Eigen::Isometry3d from_pose(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6 ||
    std::abs(q.norm() - 1.0) > 0.01)
  {
    throw std::invalid_argument("invalid quaternion");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = q.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  if (!valid_transform(result)) {throw std::invalid_argument("invalid pose");}
  return result;
}

Eigen::Isometry3d from_tf(const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return from_pose(pose);
}

geometry_msgs::msg::Pose to_pose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond q(transform.linear());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

class Relocalizer : public rclcpp::Node
{
public:
  Relocalizer() : Node("small_gicp_relocalizer"), buffer_(get_clock()), listener_(buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "body");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
    const std::vector<std::string> frames{map_frame_, odom_frame_, base_frame_, cloud_frame_, lidar_frame_};
    for (size_t i = 0; i < frames.size(); ++i) {
      if (frames[i].empty() || frames[i].front() == '/') {
        throw std::invalid_argument("frame names must be nonempty without a leading slash");
      }
      for (size_t j = 0; j < i; ++j) {
        if (frames[i] == frames[j]) {throw std::invalid_argument("frame roles must be distinct");}
      }
    }
    imu_lidar_ = make_transform(
      declare_parameter<std::vector<double>>("t_imu_lidar", std::vector<double>{}),
      declare_parameter<std::vector<double>>("R_imu_lidar", std::vector<double>{}));
    registration_rate_ = positive("registration_rate", 1.0);
    const double tf_rate = positive("tf_publish_rate", 20.0);
    map_leaf_ = positive("map_leaf_size", 0.30);
    scan_leaf_ = positive("scan_leaf_size", 0.30);
    neighbors_ = positive_int("num_neighbors", 20);
    settings_.num_threads = positive_int("num_threads", 2);
    settings_.max_iterations = positive_int("max_iterations", 40);
    settings_.max_correspondence_distance = positive("max_correspondence_distance", 1.0);
    settings_.translation_eps = positive("translation_eps", 0.001);
    settings_.rotation_eps = positive("rotation_eps", 0.001745);
    min_source_ = positive_int("min_source_points", 200);
    min_inliers_ = positive_int("min_inliers", 100);
    min_ratio_ = positive("min_inlier_ratio", 0.50);
    if (min_ratio_ > 1.0 || neighbors_ < 3 || min_source_ < neighbors_) {
      throw std::invalid_argument("invalid point counts or inlier ratio");
    }
    max_rmse_ = positive("max_euclidean_rmse", 0.30);
    init_translation_ = positive("init_max_translation", 1.0);
    init_rotation_ = positive("init_max_rotation", 0.261799);
    initialpose_check_correction_ = declare_parameter<bool>("initialpose_check_correction", false);
    track_translation_ = positive("track_max_translation", 0.50);
    track_rotation_ = positive("track_max_rotation", 0.174533);
    confirm_translation_ = positive("confirmation_translation", 0.20);
    confirm_rotation_ = positive("confirmation_rotation", 0.052360);
    confirmation_frames_ = positive_int("confirmation_frames", 2);
    if (confirmation_frames_ < 2) {throw std::invalid_argument("initialization requires two or more frames");}
    max_scan_age_ = positive("max_scan_age", 0.50);
    tf_wait_ = positive("tf_wait_timeout", 0.20);
    initialpose_use_latest_tf_ = declare_parameter<bool>("initialpose_use_latest_tf", true);
    max_job_age_ = positive("max_job_age", 1.0);
    degraded_after_ = positive("degraded_after", 3.0);
    failures_limit_ = positive_int("failures_before_degraded", 3);
    odom_jump_translation_ = positive("odom_jump_translation", 2.0);
    odom_jump_rotation_ = positive("odom_jump_rotation", 1.570796);
    max_linear_speed_ = positive("max_expected_linear_speed", 3.0);
    max_angular_speed_ = positive("max_expected_angular_speed", 3.0);
    use_base_z_ = declare_parameter<bool>("use_initial_base_z", false);
    initial_z_ = declare_parameter<double>("initial_base_z", 0.0);
    if (!std::isfinite(initial_z_)) {throw std::invalid_argument("initial_base_z must be finite");}
    aligned_enabled_ = declare_parameter<bool>("publish_aligned_cloud", false);

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/relocalization/pose", 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/relocalization/status", 10);
    map_pub_ = create_publisher<Cloud>(
      "/relocalization/map_cloud", rclcpp::QoS(1).reliable().transient_local());
    if (aligned_enabled_) {
      aligned_pub_ = create_publisher<Cloud>("/relocalization/aligned_cloud", rclcpp::SensorDataQoS());
    }
    load_map(declare_parameter<std::string>("prior_pcd_file", ""));
    cloud_sub_ = create_subscription<Cloud>(
      declare_parameter<std::string>("cloud_topic", "/cloud_registered_body"),
      rclcpp::SensorDataQoS().keep_last(1),
      [this](Cloud::ConstSharedPtr cloud) {on_cloud(std::move(cloud));});
    seed_sub_ = create_subscription<PoseSeed>(
      declare_parameter<std::string>("initialpose_topic", "/initialpose"), 1,
      [this](PoseSeed::ConstSharedPtr pose) {on_seed(*pose);});
    next_job_ = next_status_ = Steady::now();
    worker_ = std::thread([this]() {work();});
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / tf_rate)),
      [this]() {tick();});
    RCLCPP_INFO(get_logger(), "Map loaded (%zu points). Waiting for initialpose and static %s <- %s TF",
      target_->size(), base_frame_.c_str(), lidar_frame_.c_str());
  }

  ~Relocalizer() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      ++generation_;
      job_.reset();
    }
    condition_.notify_one();
    if (worker_.joinable()) {worker_.join();}
  }

private:
  struct Job
  {
    Cloud::ConstSharedPtr cloud;
    Eigen::Isometry3d odom_base, base_body, guess;
    int64_t stamp;
    uint64_t generation;
    bool initializing;
    Steady::time_point started;
  };

  double positive(const std::string & name, double fallback)
  {
    const auto value = declare_parameter<double>(name, fallback);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  int positive_int(const std::string & name, int fallback)
  {
    const auto value = declare_parameter<int>(name, fallback);
    if (value <= 0) {throw std::invalid_argument(name + " must be positive");}
    return value;
  }

  void load_map(const std::string & path)
  {
    pcl::PointCloud<pcl::PointXYZ> map;
    if (path.empty() || path.front() != '/' ||
      pcl::io::loadPCDFile(path, map) != 0)
    {
      throw std::runtime_error("prior_pcd_file must be an existing absolute PCD path");
    }
    std::vector<Eigen::Vector3d> points;
    points.reserve(map.size());
    for (const auto & p : map) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        points.emplace_back(p.x, p.y, p.z);
      }
    }
    if (points.size() < static_cast<size_t>(min_source_)) {
      throw std::runtime_error("insufficient finite map points");
    }
    auto preprocessed = small_gicp::preprocess_points(
      points, map_leaf_, neighbors_, settings_.num_threads);
    target_ = preprocessed.first;
    tree_ = preprocessed.second;
    if (target_->size() < static_cast<size_t>(min_source_)) {
      throw std::runtime_error("insufficient downsampled map points");
    }
    pcl::PointCloud<pcl::PointXYZ> display;
    display.reserve(target_->size());
    for (size_t i = 0; i < target_->size(); ++i) {
      const auto p = target_->point(i);
      display.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
    Cloud message;
    pcl::toROSMsg(display, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = now();
    map_pub_->publish(message);
  }

  void on_cloud(Cloud::ConstSharedPtr cloud)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cloud->header.frame_id != cloud_frame_) {reason_ = "wrong_cloud_frame"; return;}
    const int64_t stamp = rclcpp::Time(cloud->header.stamp).nanoseconds();
    if (stamp <= 0) {reason_ = "invalid_cloud_stamp"; return;}
    if (stamp <= dispatched_stamp_ ||
      (latest_cloud_ && stamp <= rclcpp::Time(latest_cloud_->header.stamp).nanoseconds()))
    {
      reason_ = "out_of_order_cloud";
      return;
    }
    latest_cloud_ = std::move(cloud);
    cloud_received_ = Steady::now();
  }

  void on_seed(const PoseSeed & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (message.header.frame_id != map_frame_) {
      reason_ = "wrong_initialpose_frame";
      RCLCPP_WARN(get_logger(), "Initial pose frame '%s' must be '%s'",
        message.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    try {
      auto pose = from_pose(message.pose.pose);
      if (use_base_z_) {pose.translation().z() = initial_z_;}
      const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
      pending_seed_stamp_ = stamp == 0 ? now().nanoseconds() : stamp;
      if (!initialpose_use_latest_tf_ && pending_seed_stamp_ <= 0) {
        throw std::invalid_argument("invalid initialpose stamp");
      }
      // Keep the manual map pose until confirmation, including across odom resets.
      requested_pose_ = pending_pose_ = pose;
      seed_.reset();
      confirmed_.reset();
      confirmations_ = failures_ = 0;
      ++generation_;
      // A running task will be discarded by its generation; cancel any queued task.
      if (job_) {job_.reset(); busy_ = false;}
      latest_cloud_.reset();
      previous_odom_.reset();
      previous_odom_stamp_ = dispatched_stamp_ = 0;
      next_job_ = Steady::now();
      reason_ = "waiting_initialpose_tf";
      RCLCPP_INFO(get_logger(), "Initial pose received in %s; waiting for %s -> %s TF",
        map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
    } catch (const std::exception & error) {
      reason_ = "invalid_initialpose";
      RCLCPP_WARN(get_logger(), "Invalid initial pose: %s", error.what());
    }
  }

  bool lookup(const std::string & target, const std::string & source,
    int64_t stamp, Eigen::Isometry3d & output)
  {
    try {
      output = from_tf(buffer_.lookupTransform(
        target, source, rclcpp::Time(stamp, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(0.0)).transform);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void invalidate(const std::string & reason)
  {
    ++generation_;
    correction_.reset();
    seed_.reset();
    // Retry an unconfirmed manual request after the TF/odom context changes.
    pending_pose_ = requested_pose_;
    confirmed_.reset();
    latest_cloud_.reset();
    if (job_) {busy_ = false;}
    job_.reset();
    previous_odom_.reset();
    dispatched_stamp_ = accepted_stamp_ = 0;
    confirmations_ = failures_ = 0;
    reason_ = reason;
    if (pending_pose_) {
      RCLCPP_INFO(get_logger(), "Initial pose retained after %s; waiting for odometry TF",
        reason.c_str());
    }
  }

  void fail(const std::string & reason)
  {
    ++failures_;
    confirmations_ = 0;
    confirmed_.reset();
    reason_ = reason;
    if (reason == "correction_jump") {
      constexpr double rad_to_deg = 57.29577951308232;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Relocalization observation rejected: correction_jump "
        "[mode=%s, translation %.3f/%.3fm, rotation %.1f/%.1fdeg, cloud age %.3fs]",
        last_job_initializing_ ? "initialization" : "tracking",
        quality_.correction_translation,
        last_job_initializing_ ? init_translation_ : track_translation_,
        quality_.correction_rotation * rad_to_deg,
        (last_job_initializing_ ? init_rotation_ : track_rotation_) * rad_to_deg,
        last_cloud_age_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Relocalization observation rejected: %s (cloud age %.3fs, limit %.3fs)",
        reason.c_str(), last_cloud_age_, max_scan_age_);
    }
  }

  void tick()
  {
    const auto wall = Steady::now();
    const int64_t clock_now = now().nanoseconds();
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {return;}
    if (last_clock_ > 0 && clock_now < last_clock_) {
      invalidate("ros_time_backwards");
      buffer_.clear();
    }
    last_clock_ = clock_now;
    if (clock_now <= 0) {return;}

    if (!base_body_) {
      try {
        const auto mount = buffer_.lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
        // A path through odom/body would include motion and have a nonzero stamp.
        if (rclcpp::Time(mount.header.stamp).nanoseconds() != 0) {
          reason_ = "mount_tf_must_be_static";
        } else {
          base_body_ = from_tf(mount.transform) * imu_lidar_.inverse();
        }
      } catch (const std::exception &) {
        reason_ = "waiting_static_mount_tf";
      }
    }
    if (pending_pose_) {
      Eigen::Isometry3d odom_base;
      // A manual map pose is a coarse seed. Latest odometry avoids rejecting a
      // fresh RViz click simply because the corresponding EKF TF arrives later.
      const int64_t seed_stamp = initialpose_use_latest_tf_ ? 0 : pending_seed_stamp_;
      if (lookup(odom_frame_, base_frame_, seed_stamp, odom_base)) {
        seed_ = map_from_odom(*pending_pose_, odom_base);
        pending_pose_.reset();
        next_job_ = wall;
        reason_ = "initializing";
        RCLCPP_INFO(get_logger(), "Initial pose paired with odometry TF; starting registration");
      } else {
        reason_ = "waiting_initialpose_tf";
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting indefinitely for %s -> %s TF; initial pose retained",
          odom_frame_.c_str(), base_frame_.c_str());
      }
    }
    // Manual initialization has no deadline; keep trying fresh scans until it
    // succeeds or a newer /initialpose replaces it.
    if (!pending_pose_ && !busy_ && !job_ && base_body_ && (seed_ || correction_) &&
      latest_cloud_ && wall >= next_job_)
    {
      const auto stamp = rclcpp::Time(latest_cloud_->header.stamp).nanoseconds();
      const double age = (clock_now - stamp) * 1e-9;
      last_cloud_age_ = age;
      Eigen::Isometry3d odom_base;
      if (stamp <= dispatched_stamp_) {
        latest_cloud_.reset();
      } else if (age > max_scan_age_ || age < 0.0) {
        latest_cloud_.reset();
        fail("stale_or_future_cloud");
        next_job_ = wall + period();
      } else if (!lookup(odom_frame_, base_frame_, stamp, odom_base)) {
        reason_ = "waiting_cloud_tf";
        if (std::chrono::duration<double>(wall - cloud_received_).count() > tf_wait_) {
          latest_cloud_.reset();
          fail("cloud_tf_timeout");
          next_job_ = wall + period();
        }
      } else {
        const double dt = std::max(0.0, (stamp - previous_odom_stamp_) * 1e-9);
        if (previous_odom_ && !nearby(*previous_odom_, odom_base,
          odom_jump_translation_ + max_linear_speed_ * dt,
          odom_jump_rotation_ + max_angular_speed_ * dt))
        {
          invalidate("odom_discontinuity");
        } else {
          previous_odom_ = odom_base;
          previous_odom_stamp_ = stamp;
          const bool initializing = seed_.has_value();
          const Eigen::Isometry3d predicted = (initializing ? *seed_ : *correction_) * odom_base;
          job_ = Job{latest_cloud_, odom_base, *base_body_, predicted, stamp,
            generation_, initializing, wall};
          latest_cloud_.reset();
          dispatched_stamp_ = stamp;
          next_job_ = wall + period();
          busy_ = true;
          condition_.notify_one();
        }
      }
    }

    const double correction_age = accepted_stamp_ ? (clock_now - accepted_stamp_) * 1e-9 : -1.0;
    // Broadcast the last accepted correction at the current time, even while degraded.
    if (correction_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = rclcpp::Time(clock_now, get_clock()->get_clock_type());
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = odom_frame_;
      const auto pose = to_pose(*correction_);
      tf.transform.translation.x = pose.position.x;
      tf.transform.translation.y = pose.position.y;
      tf.transform.translation.z = pose.position.z;
      tf.transform.rotation = pose.orientation;
      broadcaster_->sendTransform(tf);
    }
    if (wall >= next_status_) {
      publish_status(clock_now, correction_age);
      next_status_ = wall + std::chrono::milliseconds(500);
    }
  }

  Steady::duration period() const
  {
    return std::chrono::duration_cast<Steady::duration>(
      std::chrono::duration<double>(1.0 / registration_rate_));
  }

  std::vector<Eigen::Vector3d> scan_points(const Job & job)
  {
    const auto & cloud = *job.cloud;
    if (!cloud.width || !cloud.height || !cloud.point_step ||
      static_cast<uint64_t>(cloud.row_step) < static_cast<uint64_t>(cloud.width) * cloud.point_step ||
      cloud.data.size() < static_cast<uint64_t>(cloud.row_step) * cloud.height)
    {
      throw std::invalid_argument("invalid_cloud_layout");
    }
    for (const auto * name : {"x", "y", "z"}) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [name](const auto & f) {return f.name == name;});
      if (field == cloud.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        field->count != 1 || static_cast<uint64_t>(field->offset) + sizeof(float) > cloud.point_step)
      {
        throw std::invalid_argument("cloud_requires_float32_xyz");
      }
    }
    pcl::PointCloud<pcl::PointXYZ> raw;
    pcl::fromROSMsg(cloud, raw);
    std::vector<Eigen::Vector3d> points;
    points.reserve(raw.size());
    for (const auto & p : raw) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
        points.push_back(job.base_body * Eigen::Vector3d(p.x, p.y, p.z));
      }
    }
    return points;
  }

  void work()
  {
    while (true) {
      std::optional<Job> task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {return stopping_ || job_.has_value();});
        if (stopping_) {return;}
        task = std::move(job_);
        job_.reset();
      }
      Quality quality;
      small_gicp::RegistrationResult result;
      std::string rejected;
      Cloud aligned;
      try {
        auto points = scan_points(*task);
        if (points.size() < static_cast<size_t>(min_source_)) {
          throw std::invalid_argument("insufficient_source_points");
        }
        auto preprocessed = small_gicp::preprocess_points(
          points, scan_leaf_, neighbors_, settings_.num_threads);
        if (preprocessed.first->size() < static_cast<size_t>(min_source_)) {
          throw std::invalid_argument("insufficient_downsampled_source");
        }
        result = small_gicp::align(*target_, *preprocessed.first, *tree_, task->guess, settings_);
        quality = measure_quality(*preprocessed.first, *tree_, result,
          settings_.max_correspondence_distance);
        if (valid_transform(result.T_target_source)) {
          quality.correction_translation =
            (result.T_target_source.translation() - task->guess.translation()).norm();
          quality.correction_rotation = rotation_distance(task->guess, result.T_target_source);
        }
        // A manual pose starts a new localization context. Its coarse error is
        // checked by registration quality and consecutive-frame confirmation;
        // normal tracking still rejects jumps relative to its accepted context.
        rejected = rejection_reason(result, quality, task->guess, min_source_, min_inliers_,
          min_ratio_, max_rmse_, task->initializing ? init_translation_ : track_translation_,
          task->initializing ? init_rotation_ : track_rotation_,
          !task->initializing || initialpose_check_correction_);
        if (rejected.empty() && aligned_enabled_) {
          pcl::PointCloud<pcl::PointXYZ> display;
          display.reserve(preprocessed.first->size());
          for (size_t i = 0; i < preprocessed.first->size(); ++i) {
            const Eigen::Vector4d p = result.T_target_source.matrix() * preprocessed.first->point(i);
            display.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
          }
          pcl::toROSMsg(display, aligned);
          aligned.header = task->cloud->header;
          aligned.header.frame_id = map_frame_;
        }
      } catch (const std::exception & exception) {
        rejected = exception.what();
      }
      quality.processing_ms = std::chrono::duration<double, std::milli>(
        Steady::now() - task->started).count();
      std::lock_guard<std::mutex> lock(mutex_);
      busy_ = false;
      if (stopping_ || task->generation != generation_) {continue;}
      quality_ = quality;
      last_job_initializing_ = task->initializing;
      if (quality.processing_ms * 1e-3 > max_job_age_ ||
        (now().nanoseconds() - task->stamp) * 1e-9 > max_job_age_)
      {
        rejected = "job_timeout";
      }
      if (now().nanoseconds() < task->stamp) {rejected = "time_changed_during_job";}
      if (task->stamp <= accepted_stamp_) {rejected = "out_of_order_result";}
      if (!rejected.empty()) {fail(rejected); continue;}
      const auto candidate = map_from_odom(result.T_target_source, task->odom_base);
      if (task->initializing) {
        if (confirmed_ && nearby(*confirmed_, candidate, confirm_translation_, confirm_rotation_)) {
          ++confirmations_;
        } else {
          confirmations_ = 1;
        }
        confirmed_ = candidate;
        reason_ = "confirming_initialization";
        if (confirmations_ < confirmation_frames_) {continue;}
        seed_.reset();
        requested_pose_.reset();
        confirmed_.reset();
        RCLCPP_INFO(get_logger(),
          "Relocalization initialized; publishing %s -> %s TF "
          "(seed correction %.3fm, %.1fdeg)",
          map_frame_.c_str(), odom_frame_.c_str(), quality.correction_translation,
          quality.correction_rotation * 57.29577951308232);
      }
      correction_ = candidate;
      accepted_stamp_ = task->stamp;
      failures_ = 0;
      reason_ = "accepted";
      geometry_msgs::msg::PoseStamped message;
      message.header = task->cloud->header;
      message.header.frame_id = map_frame_;
      message.pose = to_pose(result.T_target_source);
      pose_pub_->publish(message);
      if (aligned_enabled_) {aligned_pub_->publish(aligned);}
    }
  }

  void publish_status(int64_t clock_now, double correction_age)
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = rclcpp::Time(clock_now, get_clock()->get_clock_type());
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "small_gicp_relocalizer";
    status.hardware_id = "software";
    const bool degraded = correction_ &&
      (failures_ >= failures_limit_ || correction_age > degraded_after_);
    status.message = pending_pose_ || seed_ ? "INITIALIZING" :
      correction_ ? (degraded ? "DEGRADED" : "TRACKING") : "WAIT_INIT";
    status.level = status.message == "TRACKING" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    auto value = [&status](const std::string & name, const std::string & text) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = name;
        kv.value = text;
        status.values.push_back(kv);
      };
    auto number = [&value](const std::string & name, auto v) {value(name, std::to_string(v));};
    value("reason", reason_);
    number("generation", generation_);
    number("confirmation_count", confirmations_);
    number("consecutive_failures", failures_);
    number("job_in_flight", busy_ ? 1 : 0);
    number("source_points", quality_.source_points);
    number("gicp_inliers", quality_.gicp_inliers);
    number("euclidean_inliers", quality_.euclidean_inliers);
    number("inlier_ratio", quality_.source_points ?
      static_cast<double>(quality_.euclidean_inliers) / quality_.source_points : 0.0);
    number("euclidean_rmse_m", quality_.rmse);
    number("gicp_error", quality_.gicp_error);
    number("iterations", quality_.iterations);
    number("hessian_min_eigenvalue", quality_.hessian_min_eigenvalue);
    number("hessian_condition", quality_.hessian_condition);
    number("processing_ms", quality_.processing_ms);
    number("cloud_age_s", last_cloud_age_);
    value("registration_mode", last_job_initializing_ ? "initialization" : "tracking");
    value("initialpose_check_correction", initialpose_check_correction_ ? "true" : "false");
    number("correction_translation_m", quality_.correction_translation);
    number("correction_rotation_rad", quality_.correction_rotation);
    number("last_success_stamp_ns", accepted_stamp_);
    number("correction_age_s", correction_age);
    value("tf_refreshing", correction_ ? "true" : "false");
    message.status.push_back(std::move(status));
    status_pub_->publish(message);
  }

  std::string map_frame_, odom_frame_, base_frame_, cloud_frame_, lidar_frame_;
  Eigen::Isometry3d imu_lidar_;
  std::optional<Eigen::Isometry3d> base_body_, requested_pose_, pending_pose_, seed_;
  std::optional<Eigen::Isometry3d> confirmed_, correction_, previous_odom_;
  double registration_rate_, map_leaf_, scan_leaf_, min_ratio_, max_rmse_;
  double init_translation_, init_rotation_, track_translation_, track_rotation_;
  double confirm_translation_, confirm_rotation_, max_scan_age_, tf_wait_, max_job_age_;
  double degraded_after_, odom_jump_translation_, odom_jump_rotation_;
  double max_linear_speed_, max_angular_speed_, initial_z_;
  double last_cloud_age_ = 0.0;
  int neighbors_, min_source_, min_inliers_, confirmation_frames_, confirmations_ = 0;
  int failures_limit_, failures_ = 0;
  bool use_base_z_, aligned_enabled_, initialpose_use_latest_tf_, initialpose_check_correction_;
  bool last_job_initializing_ = false;
  bool stopping_ = false, busy_ = false;
  int64_t last_clock_ = 0, pending_seed_stamp_ = 0, dispatched_stamp_ = 0;
  int64_t accepted_stamp_ = 0, previous_odom_stamp_ = 0;
  uint64_t generation_ = 0;
  small_gicp::RegistrationSetting settings_;
  small_gicp::PointCloud::Ptr target_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> tree_;
  Quality quality_;
  std::string reason_ = "waiting_initialpose";
  Cloud::ConstSharedPtr latest_cloud_;
  std::optional<Job> job_;
  Steady::time_point next_job_, next_status_, cloud_received_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr map_pub_, aligned_pub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Subscription<PoseSeed>::SharedPtr seed_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace scout_mini_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<scout_mini_relocalization::Relocalizer>());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("small_gicp_relocalizer"), "%s", exception.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
