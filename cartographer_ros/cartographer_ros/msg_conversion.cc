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

#include "cartographer_ros/msg_conversion.h"

#include <cmath>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "glog/logging.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"

namespace cartographer_ros {
namespace {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::cartographer::sensor::LandmarkData;
using ::cartographer::sensor::LandmarkObservation;
using ::cartographer::sensor::PointCloudWithIntensities;
using ::cartographer::transform::Rigid3d;
using ::cartographer_ros_msgs::LandmarkEntry;
using ::cartographer_ros_msgs::LandmarkList;

sensor_msgs::PointCloud2 PreparePointCloud2Message(const int64_t timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);
  return msg;
}

// For sensor_msgs::LaserScan.
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }

// For sensor_msgs::MultiEchoLaserScan.
bool HasEcho(const sensor_msgs::LaserEcho& echo) {
  return !echo.echoes.empty();
}

float GetFirstEcho(const sensor_msgs::LaserEcho& echo) {
  return echo.echoes[0];
}

// For sensor_msgs::LaserScan and sensor_msgs::MultiEchoLaserScan.
template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) {
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
      const float first_echo = GetFirstEcho(echoes);
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        Eigen::Vector4f point;
        point << rotation * (first_echo * Eigen::Vector3f::UnitX()),
            i * msg.time_increment;
        point_cloud.points.push_back(point);
        if (msg.intensities.size() > 0) {
          CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          const auto& echo_intensities = msg.intensities[i];
          CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
        } else {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += msg.angle_increment;
  }
  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back()[3];
    timestamp += cartographer::common::FromSeconds(duration);
    for (Eigen::Vector4f& point : point_cloud.points) {
      point[3] -= duration;
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

bool PointCloud2HasField(const sensor_msgs::PointCloud2& pc2,
                         const std::string& field_name) {
  for (const auto& field : pc2.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

}  // namespace

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud) {
  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (const Eigen::Vector4f& point : point_cloud) {
    stream.next(point.x());
    stream.next(point.y());
    stream.next(point.z());
    stream.next(kPointCloudComponentFourMagic);
  }
  return msg;
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::LaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::MultiEchoLaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::PointCloud2& message) {
  PointCloudWithIntensities point_cloud;
  // We check for intensity field here to avoid run-time warnings if we pass in
  // a PointCloud2 without intensity.
  if (PointCloud2HasField(message, "intensity")) {
    pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
    pcl::fromROSMsg(message, pcl_point_cloud);
    for (const auto& point : pcl_point_cloud) {
      point_cloud.points.emplace_back(point.x, point.y, point.z, 0.f);
      point_cloud.intensities.push_back(point.intensity);
    }
  } else {
    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
    pcl::fromROSMsg(message, pcl_point_cloud);

    // If we don't have an intensity field, just copy XYZ and fill in
    // 1.0.
    for (const auto& point : pcl_point_cloud) {
      point_cloud.points.emplace_back(point.x, point.y, point.z, 0.f);
      point_cloud.intensities.push_back(1.0);
    }
  }
  return std::make_tuple(point_cloud, FromRos(message.header.stamp));
}

LandmarkData ToLandmarkData(const LandmarkList& landmark_list) {
  LandmarkData landmark_data;
  landmark_data.time = FromRos(landmark_list.header.stamp);
  for (const LandmarkEntry& entry : landmark_list.landmark) {
    landmark_data.landmark_observations.push_back(
        {entry.id, ToRigid3d(entry.tracking_from_landmark_transform),
         entry.translation_weight, entry.rotation_weight});
  }
  return landmark_data;
}

Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}

Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

geometry_msgs::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) {
  geometry_msgs::Transform transform;
  transform.translation.x = rigid3d.translation().x();
  transform.translation.y = rigid3d.translation().y();
  transform.translation.z = rigid3d.translation().z();
  transform.rotation.w = rigid3d.rotation().w();
  transform.rotation.x = rigid3d.rotation().x();
  transform.rotation.y = rigid3d.rotation().y();
  transform.rotation.z = rigid3d.rotation().z();
  return transform;
}

geometry_msgs::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) {
  geometry_msgs::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

geometry_msgs::PoseStamped ToGeometryMsgPose(
    const geometry_msgs::TransformStamped& transform) {
  geometry_msgs::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) {
  geometry_msgs::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

geometry_msgs::Vector3 ToGeometryMsgVector3(const geometry_msgs::Point& pt) {
  geometry_msgs::Vector3 vec3;
  vec3.x = pt.x;
  vec3.y = pt.y;
  vec3.z = pt.z;
  return vec3;
}

Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(cartographer::common::DegToRad(latitude));
  const double cos_phi = std::cos(cartographer::common::DegToRad(latitude));
  const double sin_lambda = std::sin(cartographer::common::DegToRad(longitude));
  const double cos_lambda = std::cos(cartographer::common::DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude) {
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(cartographer::common::DegToRad(latitude - 90.),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(cartographer::common::DegToRad(-longitude),
                        Eigen::Vector3d::UnitZ());
  return cartographer::transform::Rigid3d(rotation * -translation, rotation);
}

std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const ros::Time& time) {
  auto occupancy_grid =
      ::cartographer::common::make_unique<nav_msgs::OccupancyGrid>();

  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height =
      cairo_image_surface_get_height(painted_slices.surface.get());
  // const ros::Time now = ros::Time::now();

  occupancy_grid->header.stamp = time;
  occupancy_grid->header.frame_id = frame_id;
  occupancy_grid->info.map_load_time = time;
  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = width;
  occupancy_grid->info.height = height;
  occupancy_grid->info.origin.position.x =
      -painted_slices.origin.x() * resolution;
  occupancy_grid->info.origin.position.y =
      (-height + painted_slices.origin.y()) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  occupancy_grid->info.origin.orientation.w = 1.;
  occupancy_grid->info.origin.orientation.x = 0.;
  occupancy_grid->info.origin.orientation.y = 0.;
  occupancy_grid->info.origin.orientation.z = 0.;

  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
  occupancy_grid->data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value =
          observed == 0
              ? -1
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid->data.push_back(value);
    }
  }

  return occupancy_grid;
}

geometry_msgs::Pose ComposePoses(
    const geometry_msgs::Pose& pose1,
    const geometry_msgs::Pose& pose2) {
  const tf2::Transform transf1 = PoseToTfTransform(pose1);
  const tf2::Transform transf2 = PoseToTfTransform(pose2);
  const tf2::Transform transf = transf1*transf2;
  const tf2::Quaternion q = transf.getRotation();
  const tf2::Vector3 t = transf.getOrigin();

  geometry_msgs::Pose pose;
  pose.position.x = t.x();
  pose.position.y = t.y();
  pose.position.z = t.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::Transform ComposeTransforms(
    const geometry_msgs::Transform& transform1,
    const geometry_msgs::Transform& transform2) {
  geometry_msgs::Transform transform;

  const tf2::Transform transf1 = TransformToTfTransform(transform1);
  const tf2::Transform transf2 = TransformToTfTransform(transform2);
  const tf2::Transform transf = transf1*transf2;
  const tf2::Quaternion q = transf.getRotation();
  const tf2::Vector3 t = transf.getOrigin();

  transform.translation.x = t.x();
  transform.translation.y = t.y();
  transform.translation.z = t.z();
  transform.rotation.x = q.x();
  transform.rotation.y = q.y();
  transform.rotation.z = q.z();
  transform.rotation.w = q.w();

  return transform;
}

geometry_msgs::Pose ComputeRelativePose(
    const geometry_msgs::Pose& pose1,
    const geometry_msgs::Pose& pose2) {
  const tf2::Transform transf1 = PoseToTfTransform(pose1);
  const tf2::Transform transf2 = PoseToTfTransform(pose2);
  const tf2::Transform transf_2_to_1 = transf1.inverse()*transf2;
  const tf2::Quaternion q = transf_2_to_1.getRotation();
  const tf2::Vector3 t = transf_2_to_1.getOrigin();

  geometry_msgs::Pose pose;
  pose.position.x = t.x();
  pose.position.y = t.y();
  pose.position.z = t.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::Point RotateVector(const geometry_msgs::Point &pt,
                                  const geometry_msgs::Quaternion &quat) {
  const tf2::Vector3 t(pt.x, pt.y, pt.z);
  const tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  const tf2::Matrix3x3 R(q);
  const tf2::Vector3 rot_t = R*t;
  geometry_msgs::Point rot_pt;
  rot_pt.x = rot_t.x();
  rot_pt.y = rot_t.y();
  rot_pt.z = rot_t.z();
  return rot_pt;
}

geometry_msgs::Quaternion ZeroQuaternion() {
  geometry_msgs::Quaternion quat;
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = 0.0;
  quat.w = 1.0;
  return quat;
}

geometry_msgs::Pose ZeroPose() {
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation = ZeroQuaternion();
  return pose;
}

geometry_msgs::Transform ZeroTransform() {
  geometry_msgs::Transform transform;
  transform.translation.x = 0.0;
  transform.translation.y = 0.0;
  transform.translation.z = 0.0;
  transform.rotation = ZeroQuaternion();
  return transform;
}

geometry_msgs::TransformStamped PoseToTransformStamped(
      const geometry_msgs::PoseStamped &pose,
      const std::string &child_frame_id) {
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header = pose.header;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation = ToGeometryMsgVector3(pose.pose.position);
  transform_stamped.transform.rotation = pose.pose.orientation;
  return transform_stamped;
}

tf2::Transform PoseToTfTransform(const geometry_msgs::Pose &pose) {
  const geometry_msgs::Point point = pose.position;
  const geometry_msgs::Quaternion quat = pose.orientation;

  const tf2::Vector3 t(point.x, point.y, point.z);
  const tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);

  const tf2::Transform transf(q, t);
  return transf;
}

tf2::Transform TransformToTfTransform(const geometry_msgs::Transform &transform) {
  const geometry_msgs::Vector3 point = transform.translation;
  const geometry_msgs::Quaternion quat = transform.rotation;

  const tf2::Vector3 t(point.x, point.y, point.z);
  const tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);

  const tf2::Transform transf(q, t);
  return transf;
}

geometry_msgs::TransformStamped tf_to_tf2(tf::StampedTransform transform) {
  geometry_msgs::TransformStamped transform2;
  tf::Quaternion q = transform.getRotation();
  tf::Vector3 v = transform.getOrigin();
  transform2.transform.translation.x = v.getX();
  transform2.transform.translation.y = v.getY();
  transform2.transform.translation.z = v.getZ();
  transform2.transform.rotation.x = q.getX();
  transform2.transform.rotation.y = q.getY();
  transform2.transform.rotation.z = q.getZ();
  transform2.transform.rotation.w = q.getW();
  transform2.header.frame_id = transform.frame_id_;
  transform2.header.stamp    = transform.stamp_;
  transform2.child_frame_id  = transform.child_frame_id_;
  return transform2;
}

double GetYaw(const geometry_msgs::Quaternion& quat) {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 R(q);
  double r, p, y;
  R.getRPY(r, p, y);
  return y;
}

geometry_msgs::Quaternion QuatFromYaw(const double& yaw) {
  geometry_msgs::Quaternion quat;
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = sin(yaw/2.0);
  quat.w = cos(yaw/2.0);
  return quat;
}

}  // namespace cartographer_ros
