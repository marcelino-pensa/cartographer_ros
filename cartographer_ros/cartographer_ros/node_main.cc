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

class MapClass {
 public:
  bool start_map_;
  bool terminate_map_;
  ::ros::Publisher odom_drift_publisher_, accumul_odom_drift_publisher_;
  ::ros::ServiceServer new_map_srv, terminate_map_srv;
  ::geometry_msgs::PoseStamped accumul_odom_drift_;
  ::geometry_msgs::PoseStamped cur_odom_drift_;

  MapClass(const bool &start_map) {
    start_map_ = start_map;
    terminate_map_ = false;
    accumul_odom_drift_.pose = ZeroPose();
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
      
      cur_odom_drift_ = node.GetOdomDrift();

      // Check if mapping needs to stop
      if (terminate_map_) {
        terminate_map_ = false;

        // Update the accumulated odometry drift
        accumul_odom_drift_.header = cur_odom_drift_.header;
        accumul_odom_drift_.pose = 
          ComposePoses(accumul_odom_drift_.pose, cur_odom_drift_.pose);

        // Break loop
        break;
      } else {
        if (cur_odom_drift_.header.stamp.toSec() != 0) {
          // Current odom drift
          odom_drift_publisher_.publish(cur_odom_drift_);

          // Odom drift accumulated over multiple map passes
          ::geometry_msgs::PoseStamped local_accumul_odom_drift;
          local_accumul_odom_drift.header = cur_odom_drift_.header;
          local_accumul_odom_drift.pose = 
            ComposePoses(accumul_odom_drift_.pose, cur_odom_drift_.pose);
          accumul_odom_drift_publisher_.publish(local_accumul_odom_drift);
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
    ::ros::NodeHandle node_handle_;
    new_map_srv = node_handle_.advertiseService(
        kStartMappingServiceName, &MapClass::StartNewMapService, this);
    terminate_map_srv = node_handle_.advertiseService(
        kStopMappingServiceName,  &MapClass::StopMappingService, this);
    odom_drift_publisher_ = node_handle_.advertise<::geometry_msgs::PoseStamped>(
        kOdomDriftTopic, kLatestOnlyPublisherQueueSize);
    accumul_odom_drift_publisher_ = node_handle_.advertise<::geometry_msgs::PoseStamped>(
        kAccumulOdomDriftTopic, kLatestOnlyPublisherQueueSize);

    ::ros::Rate rate(100);
    while(::ros::ok()) {
      if (start_map_) {

        // The function below blocks until it finishes mapping
        this->StartNewMap();

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

  cartographer_ros::MapClass map_obj(true);
  map_obj.Run();

  // cartographer_ros::Run(true);

  ::ros::shutdown();
}
