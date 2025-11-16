// Copyright 2018, 2019 Kevin Hallenbeck, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "velodyne_laserscan/velodyne_laserscan.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include <map>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace velodyne_laserscan
{

VelodyneLaserScan::VelodyneLaserScan(const rclcpp::NodeOptions & options)
: rclcpp::Node("velodyne_laserscan_node", options)
{
  rcl_interfaces::msg::ParameterDescriptor ring_desc;
  ring_desc.name = "ring";
  ring_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  ring_desc.description = "Ring";
  rcl_interfaces::msg::IntegerRange ring_range;
  ring_range.from_value = -1;
  ring_range.to_value = 31;
  ring_desc.integer_range.push_back(ring_range);
  ring_ = declare_parameter("ring", -1, ring_desc);

  // 2つ目のリングパラメータの追加
  rcl_interfaces::msg::ParameterDescriptor ring2_desc;
  ring2_desc.name = "ring2";
  ring2_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  ring2_desc.description = "Second Ring for 2.5D scan";
  rcl_interfaces::msg::IntegerRange ring2_range;
  ring2_range.from_value = -1;
  ring2_range.to_value = 31;
  ring2_desc.integer_range.push_back(ring2_range);
  ring_2_ = declare_parameter("ring2", -1, ring2_desc);

  rcl_interfaces::msg::ParameterDescriptor resolution_desc;
  resolution_desc.name = "resolution";
  resolution_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  resolution_desc.description = "Resolution";
  rcl_interfaces::msg::FloatingPointRange resolution_range;
  resolution_range.from_value = 0.001;
  resolution_range.to_value = 0.05;
  resolution_desc.floating_point_range.push_back(resolution_range);
  resolution_ = declare_parameter("resolution", 0.007, resolution_desc);

  // 複数リング用のパラメータの追加
  rcl_interfaces::msg::ParameterDescriptor use_multi_rings_desc;
  use_multi_rings_desc.name = "use_multi_rings";
  use_multi_rings_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  use_multi_rings_desc.description = "Use multiple rings for 2.5D scan3";
  use_multi_ring_ = declare_parameter("use_multi_rings", true, use_multi_rings_desc);

  // リングの選択パラメータ（使用するリング番号の配列）
  std::vector<int64_t> default_rings = {1, 2, 3, 4, 5, 6, 8};
  ring_ids_ = declare_parameter("ring_ids", default_rings);

  // 各リングの最大距離パラメータ（8番目のリングは100mまで見えるように設定）
  // 各リングの最大距離（地面に当たらない範囲）
  // 1: 1.25m
  // 2: 1.50m
  // 3: 1.75m
  // 4: 2.00m
  // 5: 2.50m
  // 6: 3.50m
  // 7: 5.25m
  std::vector<double> default_max_distances = {1.25, 1.50, 1.75, 2.00, 2.50, 3.50, 100.0};
  ring_max_distances_ = declare_parameter("ring_max_distances", default_max_distances);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points", rclcpp::QoS(10),
    std::bind(&VelodyneLaserScan::recvCallback, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  // pub_2d5_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan2", 10);  // 2.5D用のパブリッシャー初期化
  pub_multi_ring_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan3", 10);  // 複数リング2.5D用のパブリッシャー初期化
}

void VelodyneLaserScan::recvCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Latch ring count
  if (!ring_count_) {
    // Check for PointCloud2 field 'ring'
    bool found = false;
    for (size_t i = 0; i < msg->fields.size(); i++) {
      if (msg->fields[i].datatype == sensor_msgs::msg::PointField::UINT16) {
        if (msg->fields[i].name == "ring") {
          found = true;
          break;
        }
      }
    }

    if (!found) {
      RCLCPP_ERROR(this->get_logger(), "Field 'ring' of type 'UINT16' not present in PointCloud2");
      return;
    }

    for (sensor_msgs::PointCloud2ConstIterator<uint16_t> it(*msg, "ring"); it != it.end(); ++it) {
      const uint16_t ring = *it;

      if (ring + 1 > ring_count_) {
        ring_count_ = ring + 1;
      }
    }

    if (ring_count_) {
      RCLCPP_INFO(this->get_logger(), "Latched ring count of %u", ring_count_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Field 'ring' of type 'UINT16' not present in PointCloud2");
      return;
    }
  }

  // Select rings to use
  uint16_t ring;
  uint16_t ring2;

  if ((ring_ < 0) || (ring_ >= ring_count_)) {
    // Default to ring closest to being level for each known sensor
    if (ring_count_ > 32) {
      ring = 57;  // HDL-64E
    } else if (ring_count_ > 16) {
      ring = 23;  // HDL-32E
    } else {
      ring = 8;  // VLP-16
    }
  } else {
    ring = ring_;
  }

  if ((ring_2_ < 0) || (ring_2_ >= ring_count_)) {
    // Default to a different ring for 2.5D
    if (ring_count_ > 32) {
      ring2 = 45;  // HDL-64E
    } else if (ring_count_ > 16) {
      ring2 = 15;  // HDL-32E
    } else {
      ring2 = 5;  // VLP-16
    }
  } else {
    ring2 = ring_2_;
  }

  // Load structure of PointCloud2
  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;
  int offset_i = -1;
  int offset_r = -1;

  for (size_t i = 0; i < msg->fields.size(); i++) {
    if (msg->fields[i].datatype == sensor_msgs::msg::PointField::FLOAT32) {
      if (msg->fields[i].name == "x") {
        offset_x = msg->fields[i].offset;
      } else if (msg->fields[i].name == "y") {
        offset_y = msg->fields[i].offset;
      } else if (msg->fields[i].name == "z") {
        offset_z = msg->fields[i].offset;
      } else if (msg->fields[i].name == "intensity") {
        offset_i = msg->fields[i].offset;
      }
    } else if (msg->fields[i].datatype == sensor_msgs::msg::PointField::UINT16) {
      if (msg->fields[i].name == "ring") {
        offset_r = msg->fields[i].offset;
      }
    }
  }

  (void)offset_z;

  // 通常のLaserScanの処理
  if ((offset_x >= 0) && (offset_y >= 0) && (offset_r >= 0)) {
    const float kResolution = std::abs(resolution_);
    const size_t kSize = std::round(2.0 * M_PI / kResolution);
    auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan->header = msg->header;
    scan->angle_increment = kResolution;
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->range_min = 0.0;
    scan->range_max = 200.0;
    scan->time_increment = 0.0;
    scan->ranges.resize(kSize, INFINITY);

    if ((offset_x == 0) &&
      (offset_y == 4) &&
      (offset_i % 4 == 0) &&
      (offset_r % 4 == 0))
    {
      scan->intensities.resize(kSize);

      const size_t X = 0;
      const size_t Y = 1;
      const size_t I = offset_i / 4;
      const size_t R = offset_r / 4;
      for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        const uint16_t r = *(reinterpret_cast<const uint16_t *>(&it[R]));

        if (r == ring) {
          const float x = it[X];  // x
          const float y = it[Y];  // y
          const float i = it[I];  // intensity
          const int bin = (::atan2f(y, x) + static_cast<float>(M_PI)) / kResolution;

          if ((bin >= 0) && (bin < static_cast<int>(kSize))) {
            scan->ranges[bin] = ::sqrtf(x * x + y * y);
            scan->intensities[bin] = i;
          }
        }
      }
    } else {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "PointCloud2 fields in unexpected order. Using slower generic method.");

      if (offset_i >= 0) {
        scan->intensities.resize(kSize);
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");

        for (; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r, ++iter_i) {
          const uint16_t r = *iter_r;  // ring

          if (r == ring) {
            const float x = *iter_x;  // x
            const float y = *iter_y;  // y
            const float i = *iter_i;  // intensity
            const int bin = (::atan2f(y, x) + static_cast<float>(M_PI)) / kResolution;

            if ((bin >= 0) && (bin < static_cast<int>(kSize))) {
              scan->ranges[bin] = ::sqrtf(x * x + y * y);
              scan->intensities[bin] = i;
            }
          }
        }
      } else {
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

        for (; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r) {
          const uint16_t r = *iter_r;  // ring

          if (r == ring) {
            const float x = *iter_x;  // x
            const float y = *iter_y;  // y
            const int bin = (::atan2f(y, x) + static_cast<float>(M_PI)) / kResolution;

            if ((bin >= 0) && (bin < static_cast<int>(kSize))) {
              scan->ranges[bin] = ::sqrtf(x * x + y * y);
            }
          }
        }
      }
    }

    pub_->publish(std::move(scan));
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointCloud2 missing one or more required fields! (x,y,ring)");
    return;  // 必要なフィールドがない場合は早期リターン
  }

  /* 2.5D LaserScan (scan2) の処理
  if ((offset_x >= 0) && (offset_y >= 0) && (offset_r >= 0)) {
    const float kResolution = std::abs(resolution_);
    const size_t kSize = std::round(2.0 * M_PI / kResolution);
    auto scan2 = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan2->header = msg->header;
    scan2->angle_increment = kResolution;
    scan2->angle_min = -M_PI;
    scan2->angle_max = M_PI;
    scan2->range_min = 0.0;
    scan2->range_max = 200.0;
    scan2->time_increment = 0.0;
    scan2->ranges.resize(kSize, INFINITY);
    
    if (offset_i >= 0) {
      scan2->intensities.resize(kSize);
    }

    // リング1とリング2のデータを格納する配列
    std::vector<float> ring1_ranges(kSize, INFINITY);
    std::vector<float> ring2_ranges(kSize, INFINITY);
    std::vector<float> ring1_intensities;
    std::vector<float> ring2_intensities;
     
    if (offset_i >= 0) {
      ring1_intensities.resize(kSize, 0.0);
      ring2_intensities.resize(kSize, 0.0);
    }

    // リング1のデータを抽出
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");

    for (; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r, ++iter_i) {
      const uint16_t r = *iter_r;  // ring
      const float x = *iter_x;     // x
      const float y = *iter_y;     // y
      const int bin = (::atan2f(y, x) + static_cast<float>(M_PI)) / kResolution;

      if (bin >= 0 && bin < static_cast<int>(kSize)) {
        float distance = ::sqrtf(x * x + y * y);

        if (r == ring) {
          ring1_ranges[bin] = distance;
          if (offset_i >= 0) {
            ring1_intensities[bin] = *iter_i;
          }
        } else if (r == ring2) {
          ring2_ranges[bin] = distance;
          if (offset_i >= 0) {
            ring2_intensities[bin] = *iter_i;
          }
        }
      }
    }

    // 2つのリングのデータを合成（近い方の点を採用、ただしring2は4mまでの範囲に制限）
    for (size_t i = 0; i < kSize; ++i) {
      scan2->ranges[i] = ring1_ranges[i];
      
      // ring2の値が4m以内の場合のみ考慮する
      if (ring2_ranges[i] <= 4.0) {
        // ring1_ranges[i]がring2_ranges[i]より小さい場合は、ring1の値を使用
        if (ring1_ranges[i] <= ring2_ranges[i]) {
          if (offset_i >= 0) {
            scan2->intensities[i] = ring1_intensities[i];
          }
        } else {
          scan2->ranges[i] = ring2_ranges[i];
          if (offset_i >= 0) {
            scan2->intensities[i] = ring2_intensities[i];
          }
        }
      } else {
        // ring2が4mより遠い場合はring1のみを使用
        if (offset_i >= 0) {
          scan2->intensities[i] = ring1_intensities[i];
        }
      }
    }

    pub_2d5_->publish(std::move(scan2));
  }
  */

  // 複数リングを用いた拡張2.5DスキャンのためのLaserScan (scan3) の処理
  if (use_multi_ring_ && (offset_x >= 0) && (offset_y >= 0) && (offset_r >= 0)) {
    const float kResolution = std::abs(resolution_);
    const size_t kSize = std::round(2.0 * M_PI / kResolution);
    auto scan3 = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan3->header = msg->header;
    scan3->angle_increment = kResolution;
    scan3->angle_min = -M_PI;
    scan3->angle_max = M_PI;
    scan3->range_min = 0.0;
    scan3->range_max = 200.0;
    scan3->time_increment = 0.0;
    scan3->ranges.resize(kSize, INFINITY);
    
    if (offset_i >= 0) {
      scan3->intensities.resize(kSize, 0.0);
    }

    // リングIDと最大距離のマップを作成
    std::map<int, float> ring_distance_map;
    for (size_t i = 0; i < ring_ids_.size() && i < ring_max_distances_.size(); ++i) {
      ring_distance_map[ring_ids_[i]] = static_cast<float>(ring_max_distances_[i]);
    }

    // 各リングの距離データを保存する構造
    std::vector<std::vector<float>> ring_ranges_map;
    std::vector<std::vector<float>> ring_intensities_map;
    
    // リングIDごとにデータを蓄積するための準備
    ring_ranges_map.resize(16);  // VLP-16は16リングなので16に変更
    ring_intensities_map.resize(16);
    
    for (size_t i = 0; i < 16; ++i) {  // 16リングに変更
      ring_ranges_map[i].resize(kSize, INFINITY);
      if (offset_i >= 0) {
        ring_intensities_map[i].resize(kSize, 0.0);
      }
    }

    // 各リングのデータを抽出
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_r(*msg, "ring");
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");

    for (; iter_r != iter_r.end(); ++iter_x, ++iter_y, ++iter_r, ++iter_i) {
      const uint16_t r = *iter_r;  // ring
      if (r < 16) {  // 有効なリング番号のみ処理（16未満に変更）
        const float x = *iter_x;     // x
        const float y = *iter_y;     // y
        const int bin = (::atan2f(y, x) + static_cast<float>(M_PI)) / kResolution;

        if (bin >= 0 && bin < static_cast<int>(kSize)) {
          float distance = ::sqrtf(x * x + y * y);
          ring_ranges_map[r][bin] = distance;
          if (offset_i >= 0) {
            ring_intensities_map[r][bin] = *iter_i;
          }
        }
      }
    }

    // 選択したリングのデータを統合して2.5Dマップにする
    for (size_t i = 0; i < kSize; ++i) {
      float closest_range = INFINITY;
      float associated_intensity = 0.0;

      // 各リングから最も近い点を探す（ただし各リングの最大距離以内のみ）
      for (const auto & ring_pair : ring_distance_map) {
        int ring_id = ring_pair.first;
        float max_distance = ring_pair.second;
        
        if (ring_id >= 0 && ring_id < 16) {  // 16未満に変更
          // 指定した最大距離以内のデータのみを考慮
          if (ring_ranges_map[ring_id][i] <= max_distance && ring_ranges_map[ring_id][i] < closest_range) {
            closest_range = ring_ranges_map[ring_id][i];
            if (offset_i >= 0) {
              associated_intensity = ring_intensities_map[ring_id][i];
            }
          }
        }
      }

      scan3->ranges[i] = closest_range;
      if (offset_i >= 0) {
        scan3->intensities[i] = associated_intensity;
      }
    }

    pub_multi_ring_->publish(std::move(scan3));
  }
}

}  // namespace velodyne_laserscan

RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_laserscan::VelodyneLaserScan)
