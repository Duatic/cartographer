/*
 * Copyright 2026 Duatic AG
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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <memory>
#include <string>
#include <vector>

#include "absl/memory/memory.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_options_2d.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr char kSensorId[] = "points";
constexpr int kNumPoints = 20;

/**
 * @brief Options of Cartographer's 2D configuration without an IMU and with
 * the given intensity cutoff.
 * @param min_intensity Intensity below which points are dropped.
 * @return The options.
 */
proto::LocalTrajectoryBuilderOptions2D CreateOptions(
    const double min_intensity) {
  common::LuaParameterDictionary dictionary(
      R"text(
        include "trajectory_builder_2d.lua"
        TRAJECTORY_BUILDER_2D.use_imu_data = false
        TRAJECTORY_BUILDER_2D.min_intensity = )text" +
          std::to_string(min_intensity) + R"text(
        return TRAJECTORY_BUILDER_2D)text",
      absl::make_unique<common::ConfigurationFileResolver>(
          std::vector<std::string>{}));
  return CreateLocalTrajectoryBuilderOptions2D(&dictionary);
}

/**
 * @brief Number of returns local SLAM keeps of one cloud whose points
 * alternate between intensity 1 and 10.
 * @param min_intensity Intensity below which points are dropped.
 * @return Returns in the range data local SLAM inserted.
 */
size_t KeptReturns(const double min_intensity) {
  LocalTrajectoryBuilder2D builder(CreateOptions(min_intensity), {kSensorId});
  sensor::TimedPointCloudData cloud{
      common::FromUniversal(1000000000), Eigen::Vector3f::Zero(), {}, {}};
  for (int i = 0; i < kNumPoints; ++i) {
    // A wall 2 m ahead, its points far enough apart to survive the voxel
    // filter.
    cloud.ranges.push_back({Eigen::Vector3f(2.f, -1.f + 0.1f * i, 0.f), 0.f});
    cloud.intensities.push_back(i % 2 == 0 ? 1.f : 10.f);
  }
  const auto result = builder.AddRangeData(kSensorId, cloud);
  EXPECT_NE(result, nullptr);
  return result == nullptr ? 0 : result->range_data_in_local.returns.size();
}

TEST(LocalTrajectoryBuilder2DTest, DropsReturnsBelowMinIntensity) {
  EXPECT_EQ(KeptReturns(0.), kNumPoints);
  EXPECT_EQ(KeptReturns(4.), kNumPoints / 2);
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
