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

#include "cartographer/sensor/range_data.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Applies 'transform' to each of 'origins'.
std::vector<Eigen::Vector3f> TransformOrigins(
    const std::vector<Eigen::Vector3f>& origins,
    const transform::Rigid3f& transform) {
  std::vector<Eigen::Vector3f> result;
  result.reserve(origins.size());
  for (const Eigen::Vector3f& origin : origins) {
    result.push_back(transform * origin);
  }
  return result;
}

// Keeps the entries of 'origins', which is empty or parallel to 'points', whose
// point CropPointCloud() keeps.
std::vector<Eigen::Vector3f> CropOrigins(
    const PointCloud& points, const std::vector<Eigen::Vector3f>& origins,
    const float min_z, const float max_z) {
  if (!origins.empty()) {
    CHECK_EQ(origins.size(), points.size());
  }
  std::vector<Eigen::Vector3f> result;
  result.reserve(origins.size());
  for (size_t i = 0; i < origins.size(); ++i) {
    const float z = points[i].position.z();
    if (min_z <= z && z <= max_z) {
      result.push_back(origins[i]);
    }
  }
  return result;
}

}  // namespace

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform) {
  return RangeData{
      transform * range_data.origin,
      TransformPointCloud(range_data.returns, transform),
      TransformPointCloud(range_data.misses, transform),
      TransformOrigins(range_data.return_origins, transform),
      TransformOrigins(range_data.miss_origins, transform),
  };
}

RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{
      range_data.origin, CropPointCloud(range_data.returns, min_z, max_z),
      CropPointCloud(range_data.misses, min_z, max_z),
      CropOrigins(range_data.returns, range_data.return_origins, min_z, max_z),
      CropOrigins(range_data.misses, range_data.miss_origins, min_z, max_z)};
}

proto::RangeData ToProto(const RangeData& range_data) {
  proto::RangeData proto;
  *proto.mutable_origin() = transform::ToProto(range_data.origin);
  proto.mutable_returns()->Reserve(range_data.returns.size());
  for (const RangefinderPoint& point : range_data.returns) {
    *proto.add_returns() = ToProto(point);
  }
  proto.mutable_misses()->Reserve(range_data.misses.size());
  for (const RangefinderPoint& point : range_data.misses) {
    *proto.add_misses() = ToProto(point);
  }
  return proto;
}

RangeData FromProto(const proto::RangeData& proto) {
  std::vector<RangefinderPoint> returns;
  if (proto.returns_size() > 0) {
    returns.reserve(proto.returns().size());
    for (const auto& point_proto : proto.returns()) {
      returns.push_back(FromProto(point_proto));
    }
  } else {
    returns.reserve(proto.returns_legacy().size());
    for (const auto& point_proto : proto.returns_legacy()) {
      returns.push_back({transform::ToEigen(point_proto)});
    }
  }
  std::vector<RangefinderPoint> misses;
  if (proto.misses_size() > 0) {
    misses.reserve(proto.misses().size());
    for (const auto& point_proto : proto.misses()) {
      misses.push_back(FromProto(point_proto));
    }
  } else {
    misses.reserve(proto.misses_legacy().size());
    for (const auto& point_proto : proto.misses_legacy()) {
      misses.push_back({transform::ToEigen(point_proto)});
    }
  }
  return RangeData{transform::ToEigen(proto.origin()), PointCloud(returns),
                   PointCloud(misses)};
}

}  // namespace sensor
}  // namespace cartographer
