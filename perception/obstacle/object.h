#ifndef MODULES_PERCEPTION_OBSTACLE_BASE_OBJECT_H_
#define MODULES_PERCEPTION_OBSTACLE_BASE_OBJECT_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "modules/common/proto/error_code.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/time/time_util.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/base/object_supplement.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"

namespace perception {

struct alignas(16) Object {
  Object();
  // deep copy
  void clone(const Object& rhs);
  std::string ToString() const;
  void AddFourCorners(PerceptionObstacle* pb_obj) const;
  void Serialize(PerceptionObstacle* pb_obj) const;
  void Deserialize(const PerceptionObstacle& pb_obs);

  // object id per frame
  int id = 0;
  // point cloud of the object
  pcl_util::PointCloudPtr cloud;
  // convex hull of the object
  PolygonDType polygon;

  // oriented boundingbox information
  // main direction
  Eigen::Vector3d direction = Eigen::Vector3d(1, 0, 0);
  // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
  double theta = 0.0;
  // ground center of the object (cx, cy, z_min)
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  // size of the oriented bbox, length is the size in the main direction
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  // shape feature used for tracking
  std::vector<float> shape_features;

  // foreground score/probability
  float score = 0.0;
  // foreground score/probability type
  ScoreType score_type = ScoreType::SCORE_CNN;

  // Object classification type.
  ObjectType type = ObjectType::UNKNOWN;
  // Probability of each type, used for track type.
  std::vector<float> type_probs;

  // fg/bg flag
  bool is_background = false;

  // tracking information
  int track_id = 0;
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  // age of the tracked object
  double tracking_time = 0.0;
  double latest_tracked_time = 0.0;
  double timestamp = 0.0;

  // stable anchor_point during time, e.g., barycenter
  Eigen::Vector3d anchor_point;

  // noise covariance matrix for uncertainty of position and velocity
  Eigen::Matrix3d position_uncertainty;
  Eigen::Matrix3d velocity_uncertainty;

  // modeling uncertainty from sensor level tracker
  Eigen::Matrix4d state_uncertainty = Eigen::Matrix4d::Identity();
  // Tailgating (trajectory of objects)
  std::vector<Eigen::Vector3d> drops;
  // CIPV
  bool b_cipv = false;
  // local lidar track id
  int local_lidar_track_id = -1;
  // local radar track id
  int local_radar_track_id = -1;
  // local camera track id
  int local_camera_track_id = -1;

  // local lidar track ts
  double local_lidar_track_ts = -1;
  // local radar track ts
  double local_radar_track_ts = -1;
  // local camera track ts
  double local_camera_track_ts = -1;

  // sensor particular suplplements, default nullptr
  RadarSupplementPtr radar_supplement = nullptr;
  CameraSupplementPtr camera_supplement = nullptr;
};

// Sensor single frame objects.
struct SensorObjects {
  SensorObjects() {
    sensor2world_pose = Eigen::Matrix4d::Zero();
    sensor2world_pose_static = Eigen::Matrix4d::Zero();
  }

  std::string ToString() const;

  // Transmit error_code to next subnode.
  common::ErrorCode error_code = common::ErrorCode::OK;

  SensorType sensor_type = SensorType::UNKNOWN_SENSOR_TYPE;
  std::string sensor_id;
  double timestamp = 0.0;
  SeqId seq_num = 0;
  std::vector<std::shared_ptr<Object>> objects;
  Eigen::Matrix4d sensor2world_pose;
  Eigen::Matrix4d sensor2world_pose_static;
  LaneObjectsPtr lane_objects;

  uint32_t cipv_index = -1;
  uint32_t cipv_track_id = -1;

  // sensor particular suplplements, default nullptr
  RadarFrameSupplementPtr radar_frame_supplement = nullptr;
  CameraFrameSupplementPtr camera_frame_supplement = nullptr;
};

}  // namespace perception

#endif  // MODULES_PERCEPTION_OBSTACLE_BASE_OBJECT_H_