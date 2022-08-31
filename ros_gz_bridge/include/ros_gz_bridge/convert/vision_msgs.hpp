// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS_GZ_BRIDGE__CONVERT__VISION_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__VISION_MSGS_HPP_

// Gazebo Msgs
#include <ignition/msgs/oriented_3d_box.pb.h>
#include <ignition/msgs/annotated_oriented_3d_box.pb.h>
#include <ignition/msgs/annotated_oriented_3d_box_v.pb.h>

// ROS 2 messages
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{
// vision_msgs
template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::BoundingBox3D & ros_msg,
  ignition::msgs::Oriented3DBox & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Oriented3DBox & gz_msg,
  vision_msgs::msg::BoundingBox3D & ros_msg);

template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection3D & ros_msg,
  ignition::msgs::AnnotatedOriented3DBox & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::AnnotatedOriented3DBox & gz_msg,
  vision_msgs::msg::Detection3D & ros_msg);

template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection3DArray & ros_msg,
  ignition::msgs::AnnotatedOriented3DBox_V & gz_msg);

template<>
void
convert_gz_to_ros(
  const ignition::msgs::AnnotatedOriented3DBox_V & gz_msg,
  vision_msgs::msg::Detection3DArray & ros_msg);


}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__VISION_MSGS_HPP_