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

#include <tuple>
#include <vector>

#include "cartographer/sensor/internal/test_helpers.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;

class RangeDataTest : public ::testing::Test {
 protected:
  RangeDataTest() : origin_(Eigen::Vector3f(1, 1, 1)) {
    returns_.emplace_back(0, 1, 2);
    returns_.emplace_back(4, 5, 6);
    returns_.emplace_back(0, 1, 2);
    misses_.emplace_back(7, 8, 9);
  }
  Eigen::Vector3f origin_;
  std::vector<Eigen::Vector3f> returns_;
  std::vector<Eigen::Vector3f> misses_;
};

TEST(RangeDataRayOriginsTest, CropAndTransformKeepOriginsWithTheirPoints) {
  RangeData range_data{Eigen::Vector3f::Zero(), {}, {}};
  for (int i = 0; i < 10; ++i) {
    range_data.returns.push_back({{1.f, 0.f, static_cast<float>(i)}});
    range_data.return_origins.push_back({0.f, 0.f, static_cast<float>(i)});
    range_data.misses.push_back({{2.f, 0.f, static_cast<float>(i)}});
    range_data.miss_origins.push_back({0.f, 1.f, static_cast<float>(i)});
  }
  const RangeData cropped = CropRangeData(range_data, 2.5f, 5.5f);
  ASSERT_EQ(cropped.returns.size(), 3);
  ASSERT_EQ(cropped.return_origins.size(), 3);
  ASSERT_EQ(cropped.miss_origins.size(), 3);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(cropped.returns[i].position.z(), cropped.return_origins[i].z());
    EXPECT_EQ(cropped.misses[i].position.z(), cropped.miss_origins[i].z());
  }

  const transform::Rigid3f shift({0.f, 0.f, 10.f},
                                 Eigen::Quaternionf::Identity());
  const RangeData moved = TransformRangeData(cropped, shift);
  EXPECT_EQ(moved.return_origins[0], Eigen::Vector3f(0.f, 0.f, 13.f));
  EXPECT_EQ(moved.miss_origins[2], Eigen::Vector3f(0.f, 1.f, 15.f));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
