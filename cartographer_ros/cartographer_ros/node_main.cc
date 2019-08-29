/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/msg_conversion.h"
#include <cartographer_ros/tf_class.h>
#include <cartographer_ros/second_order_filter.h>
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include <mutex>

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

class MapServer {
 public:
  bool start_map_;
  bool terminate_map_, terminate_thread_;
  ::ros::Publisher accumul_odom_drift_pub_, filtered_odom_drift_pub_, fake_gps_pub_;
  ::ros::ServiceServer new_map_srv_, terminate_map_srv_, publish_drift_srv_;
  ::geometry_msgs::TransformStamped accumul_odom_drift_;
  uint drift_estimation_rate_;
  double lowpass_time_constant_;

  // Mutex-protected variables
  ::geometry_msgs::TransformStamped cur_odom_drift_;
  ::geometry_msgs::TransformStamped tf_odom_;
  std::mutex est_drift_mutex_, tf_odom_mutex_;

  MapServer() {
    ::ros::NodeHandle node_handle("~");
    node_handle.param("map_at_launch", start_map_, true);
    node_handle.param("lowpass_time_constant", lowpass_time_constant_, 0.25);
    terminate_map_ = false;
    terminate_thread_ = false;
    drift_estimation_rate_ = 50; // 100 hz

    accumul_odom_drift_.transform = ZeroTransform();
    accumul_odom_drift_.header.frame_id = "vislam_corrected";
    accumul_odom_drift_.child_frame_id = "vislam";
    cur_odom_drift_ = accumul_odom_drift_;
  }

  ~MapServer() { }

  void TfTask(const std::string &frame_id, const std::string &child_frame_id) {
    ROS_DEBUG("[cartographer_ros] tf Thread started with rate %f: ", drift_estimation_rate_);
    tf_listener::TfClass obj_tf;
    ros::Rate loop_rate(drift_estimation_rate_);
    geometry_msgs::TransformStamped transform;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped local_accumul_odom_drift, fake_gps;
    geometry_msgs::PoseStamped fake_gps_pose;
    geometry_msgs::TransformStamped filtered_accumul_odom_drift;
    bool initialize_filter = true;

    ROS_INFO("[cartographer_ros]: tf task started for tf from %s to %s.",
             frame_id.c_str(), child_frame_id.c_str());

    // Variable initialization
    accumul_odom_drift_.header.frame_id = frame_id;
    accumul_odom_drift_.child_frame_id = child_frame_id;
    transform.transform = ZeroTransform();

    // Low-pass filter for drift estimation
    lpf::SecondOrderFilter3d lpf;

    ros::Time t0 = ros::Time::now();
    while (!terminate_thread_ && ros::ok()) {
        // Get the transforms
        if(obj_tf.GetTransform(child_frame_id, frame_id)) {
          transform = tf_to_tf2(obj_tf.transform_);

          if (initialize_filter) {  // Synchronize initial timestamp
            initialize_filter = false;
            accumul_odom_drift_.header.stamp = transform.header.stamp;
            lpf.Initialize(accumul_odom_drift_, lowpass_time_constant_);            
          }

          // Publish accumulated drift
          local_accumul_odom_drift.transform = 
            ComposeTransforms(accumul_odom_drift_.transform, transform.transform);
          local_accumul_odom_drift.header.stamp = transform.header.stamp;
          local_accumul_odom_drift.header.frame_id = frame_id;
          local_accumul_odom_drift.child_frame_id = child_frame_id + "_drift";
          // geometry_msgs::TransformStamped drift = cur_odom_drift_;
          tf_broadcaster.sendTransform(local_accumul_odom_drift);

          // drift.header.stamp = ros::Time::now();
          // drift.transform.translation.z = 0.0;
          // drift.transform.rotation = ZeroQuaternion();
          // ROS_WARN("[cartographer] Drift: x = %f, y = %f", 
          //          drift.transform.translation.x, drift.transform.translation.y);
          // filtered_odom_drift_pub_.publish(drift);

          // Compute fake GPS
          ::geometry_msgs::TransformStamped tf_odom;
          tf_odom_mutex_.lock();
          tf_odom = tf_odom_;
          tf_odom_mutex_.unlock();
          if ((tf_odom_.header.stamp.toSec() != 0) && 
              (transform.header.stamp.toSec() != fake_gps.header.stamp.toSec())) {
            fake_gps.header.stamp = transform.header.stamp;
            fake_gps.header.frame_id = "vislam_odom";
            fake_gps.child_frame_id = "fake_gps";
            fake_gps.transform = ComposeTransforms(local_accumul_odom_drift.transform, tf_odom.transform);
            tf_broadcaster.sendTransform(fake_gps);

            fake_gps_pose = ToGeometryMsgPose(fake_gps.transform, "store", fake_gps.header.stamp);
            fake_gps_pose.pose.position.x = -fake_gps_pose.pose.position.x;
            fake_gps_pose.pose.position.y = -fake_gps_pose.pose.position.y;
            fake_gps_pub_.publish(fake_gps_pose);
          }

          // Filter the drift estimator
          filtered_accumul_odom_drift = lpf.Filter(local_accumul_odom_drift);

          // Save current estimated drift
          est_drift_mutex_.lock();
          cur_odom_drift_ = filtered_accumul_odom_drift;
          cur_odom_drift_.header.frame_id = "vislam_corrected";
          cur_odom_drift_.child_frame_id = "vislam";
          est_drift_mutex_.unlock();
          // filtered_odom_drift_pub_.publish(filtered_accumul_odom_drift);

          filtered_accumul_odom_drift.header.frame_id = frame_id;
          filtered_accumul_odom_drift.child_frame_id = child_frame_id + "_drift_filtered";
          tf_broadcaster.sendTransform(filtered_accumul_odom_drift);
        }

        loop_rate.sleep();
    }
    // Update the accumulated drift pose
    accumul_odom_drift_.transform = 
            ComposeTransforms(accumul_odom_drift_.transform, transform.transform);

    ROS_DEBUG("[cartographer_ros] Exiting tf Thread...");
}

  bool StartNewMapService(std_srvs::Trigger::Request  &req,
                          std_srvs::Trigger::Response &res) {
    ROS_INFO("[cartographer] Starting new map!");
    start_map_ = true;
    return true;
  }

  bool StopMappingService(std_srvs::Trigger::Request  &req,
                          std_srvs::Trigger::Response &res) {
    ROS_INFO("[cartographer] Stopping map!");
    terminate_map_ = true;
    terminate_thread_ = true;
    return true;
  }

  bool PublishDrift(std_srvs::Trigger::Request  &req,
                    std_srvs::Trigger::Response &res) {
    ROS_INFO("[cartographer] Publishing current estimated drift!");
    est_drift_mutex_.lock();
    geometry_msgs::TransformStamped drift = cur_odom_drift_;
    est_drift_mutex_.unlock();

    // Set to XY drift only
    drift.header.stamp = ros::Time::now();
    drift.transform.translation.z = 0.0;
    drift.transform.rotation = ZeroQuaternion();
    ROS_WARN("[cartographer] Drift: x = %f, y = %f", 
             drift.transform.translation.x, drift.transform.translation.y);
    
    // debug mode (not publishing any drift)
    drift.transform.translation.x = 0.0;
    drift.transform.translation.y = 0.0;

    // filtered_odom_drift_pub_.publish(drift);
    return true;
  }

  void StartNewMap() {
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);
    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

    auto map_builder =
        cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
            node_options.map_builder_options);
    Node node(node_options, std::move(map_builder), &tf_buffer);
    if (!FLAGS_load_state_filename.empty()) {
      node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    }

    if (FLAGS_start_trajectory_with_default_topics) {
      node.StartTrajectoryWithDefaultTopics(trajectory_options);
    }

    ::ros::Rate rate(100);
    while(::ros::ok()) {
      ::ros::spinOnce();

      // Check if mapping needs to stop
      if (terminate_map_) {
        terminate_map_ = false;

        // Break loop
        break;
      } else {
        tf_odom_mutex_.lock();
        tf_odom_ = node.GetLastOdom();
        tf_odom_mutex_.unlock();
      }
      rate.sleep();
    }

    node.FinishAllTrajectories();
    node.RunFinalOptimization();

    if (!FLAGS_save_state_filename.empty()) {
      node.SerializeState(FLAGS_save_state_filename);
    }
  }

  void Run() {
    ::ros::NodeHandle node_handle;
    new_map_srv_ = node_handle.advertiseService(
        kStartMappingServiceName, &MapServer::StartNewMapService, this);
    terminate_map_srv_ = node_handle.advertiseService(
        kStopMappingServiceName,  &MapServer::StopMappingService, this);
    publish_drift_srv_ = node_handle.advertiseService(
        kPublishDriftServiceName, &MapServer::PublishDrift, this);
    accumul_odom_drift_pub_ = node_handle.advertise<::geometry_msgs::TransformStamped>(
        kAccumulOdomDriftTopic, kLatestOnlyPublisherQueueSize);
    filtered_odom_drift_pub_ = node_handle.advertise<::geometry_msgs::TransformStamped>(
        kFilteredOdomDriftTopic, kLatestOnlyPublisherQueueSize);
    fake_gps_pub_ = node_handle.advertise<::geometry_msgs::PoseStamped>(
        kFakeGPSTopic, kLatestOnlyPublisherQueueSize);

    std::string odom_frame = "vislam_odom";
    std::string init_odom_frame = "first_" + odom_frame;

    ::ros::Rate rate(20);
    while(::ros::ok()) {
      if (start_map_) {
        // Start thread that estimates drift
        std::thread h_drift_estimator_task = 
          std::thread(&MapServer::TfTask, this, init_odom_frame, odom_frame);

        // The function below blocks until it finishes mapping
        this->StartNewMap();

        // Wait until thread returns
        h_drift_estimator_task.join();
        terminate_thread_ = false;

        // We don't start mapping the next time until a service request
        start_map_ = false;
      } else {  // Spins and sleeps
        ::ros::spinOnce();
        rate.sleep();
      }
    }
  }

};

}  // namespace
}  // namespace cartographer_ros


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  cartographer_ros::MapServer map_obj;
  map_obj.Run();

  // cartographer_ros::Run(true);

  ::ros::shutdown();
}
