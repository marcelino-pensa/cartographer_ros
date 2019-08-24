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
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

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
  ::ros::Publisher odom_drift_publisher_, accumul_odom_drift_publisher_;
  ::ros::ServiceServer new_map_srv, terminate_map_srv;
  ::geometry_msgs::TransformStamped accumul_odom_drift_;
  ::geometry_msgs::PoseStamped cur_odom_drift_, cur_odom_drift_map_;
  uint drift_estimation_rate_;

  MapServer() {
    ::ros::NodeHandle node_handle("~");
    node_handle.param("map_at_launch", start_map_, true);
    terminate_map_ = false;
    terminate_thread_ = false;
    accumul_odom_drift_.transform = ZeroTransform();
    drift_estimation_rate_ = 100; // 100 hz
  }

  ~MapServer() {

  }

  void TfTask(const std::string &frame_id, const std::string &child_frame_id) {
    ROS_DEBUG("[cartographer_ros] tf Thread started with rate %f: ", drift_estimation_rate_);
    tf_listener::TfClass obj_tf;
    ros::Rate loop_rate(drift_estimation_rate_);
    geometry_msgs::TransformStamped transform;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped local_accumul_odom_drift;

    ROS_INFO("[cartographer_ros]: tf task started for tf from %s to %s.",
             frame_id.c_str(), child_frame_id.c_str());

    // Variable initialization
    accumul_odom_drift_.header.frame_id = frame_id;
    accumul_odom_drift_.child_frame_id = child_frame_id;

    while (!terminate_thread_ && ros::ok()) {
        // Get the transforms
        if(obj_tf.GetTransform(child_frame_id, frame_id)) {
          transform = tf_to_tf2(obj_tf.transform_);

          // Publish accumulated drift
          local_accumul_odom_drift.transform = 
            ComposeTransforms(accumul_odom_drift_.transform, transform.transform);
          local_accumul_odom_drift.header.stamp = transform.header.stamp;
          local_accumul_odom_drift.header.frame_id = frame_id;
          local_accumul_odom_drift.child_frame_id = child_frame_id + "_drift";
          // transform.header.frame_id = frame_id;
          // transform.child_frame_id  = child_frame_id + "_drift";
          tf_broadcaster.sendTransform(local_accumul_odom_drift);

          // Convert into XY-only drift
          local_accumul_odom_drift.transform.translation.z = 0.0;
          local_accumul_odom_drift.transform.rotation = ZeroQuaternion();
          local_accumul_odom_drift.header.frame_id = child_frame_id + "_corrected";
          local_accumul_odom_drift.child_frame_id = child_frame_id;
          accumul_odom_drift_publisher_.publish(local_accumul_odom_drift);

          // Publish accumulated drift into cartographer's tf tree
          // obj_tf.PrintTransform();
        }

        loop_rate.sleep();
    }
    // Update the accumulated drift pose
    accumul_odom_drift_.transform = 
            ComposeTransforms(accumul_odom_drift_.transform, transform.transform);
    terminate_thread_ = false;

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
        if (cur_odom_drift_.header.stamp.toSec() != 0) {
        }
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
    new_map_srv = node_handle.advertiseService(
        kStartMappingServiceName, &MapServer::StartNewMapService, this);
    terminate_map_srv = node_handle.advertiseService(
        kStopMappingServiceName,  &MapServer::StopMappingService, this);
    // odom_drift_publisher_ = node_handle.advertise<::geometry_msgs::TransformStamped>(
    //     kOdomDriftTopic, kLatestOnlyPublisherQueueSize);
    accumul_odom_drift_publisher_ = node_handle.advertise<::geometry_msgs::TransformStamped>(
        kAccumulOdomDriftTopic, kLatestOnlyPublisherQueueSize);
    std::string odom_frame = "vislam";
    std::string init_odom_frame = "first_" + odom_frame;

    ::ros::Rate rate(100);
    while(::ros::ok()) {
      if (start_map_) {
        // Start thread that estimates drift
        std::thread h_drift_estimator_task = 
          std::thread(&MapServer::TfTask, this, init_odom_frame, odom_frame);

        // The function below blocks until it finishes mapping
        this->StartNewMap();

        // Wait until thread returns
        h_drift_estimator_task.join();

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
