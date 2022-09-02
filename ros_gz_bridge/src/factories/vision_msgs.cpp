// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include "factories/vision_msgs.hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/vision_msgs.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__vision_msgs(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
  // 3D Bounding Boxes
  if ((ros_type_name == "vision_msgs/msg/BoundingBox3D" || ros_type_name.empty()) &&
    (gz_type_name == "ignition.msgs.Oriented3DBox"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::BoundingBox3D,
        ignition::msgs::Oriented3DBox
      >
    >("vision_msgs/msg/BoundingBox3D", gz_type_name);
  }

  if ((ros_type_name == "vision_msgs/msg/Detection3D" || ros_type_name.empty()) &&
    (gz_type_name == "ignition.msgs.AnnotatedOriented3DBox"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::Detection3D,
        ignition::msgs::AnnotatedOriented3DBox
      >
    >("vision_msgs/msg/Detection3D", gz_type_name);
  }

  if ((ros_type_name == "vision_msgs/msg/Detection3DArray" || ros_type_name.empty()) &&
    (gz_type_name == "ignition.msgs.AnnotatedOriented3DBox_V"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::Detection3DArray,
        ignition::msgs::AnnotatedOriented3DBox_V
      >
    >("vision_msgs/msg/Detection3DArray", gz_type_name);
  }

  // 2D Bounding Boxes
  if ((ros_type_name == "vision_msgs/msg/BoundingBox2D" || ros_type_name.empty()) &&
    (gz_type_name == "ignition.msgs.AxisAligned2DBox"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::BoundingBox2D,
        ignition::msgs::AxisAligned2DBox
      >
    >("vision_msgs/msg/BoundingBox2D", gz_type_name);
  }

  if ((ros_type_name == "vision_msgs/msg/Detection2D" || ros_type_name.empty()) &&
    (gz_type_name == "ignition.msgs.AnnotatedAxisAligned2DBox"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::Detection2D,
        ignition::msgs::AnnotatedAxisAligned2DBox
      >
    >("vision_msgs/msg/Detection2D", gz_type_name);
  }

  if ((ros_type_name == "vision_msgs/msg/Detection2DArray" || ros_type_name.empty()) &&
    (gz_type_name == "ignition.msgs.AnnotatedAxisAligned2DBox_V"))
  {
    return std::make_shared<
      Factory<
        vision_msgs::msg::Detection2DArray,
        ignition::msgs::AnnotatedAxisAligned2DBox_V
      >
    >("vision_msgs/msg/Detection2DArray", gz_type_name);
  }

  return nullptr;
}

template<>
void
Factory<
  vision_msgs::msg::BoundingBox3D,
  ignition::msgs::Oriented3DBox
>::convert_ros_to_gz(
  const vision_msgs::msg::BoundingBox3D & ros_msg,
  ignition::msgs::Oriented3DBox & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::BoundingBox3D,
  ignition::msgs::Oriented3DBox
>::convert_gz_to_ros(
  const ignition::msgs::Oriented3DBox & gz_msg,
  vision_msgs::msg::BoundingBox3D & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection3D,
  ignition::msgs::AnnotatedOriented3DBox
>::convert_ros_to_gz(
  const vision_msgs::msg::Detection3D & ros_msg,
  ignition::msgs::AnnotatedOriented3DBox & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection3D,
  ignition::msgs::AnnotatedOriented3DBox
>::convert_gz_to_ros(
  const ignition::msgs::AnnotatedOriented3DBox & gz_msg,
  vision_msgs::msg::Detection3D & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection3DArray,
  ignition::msgs::AnnotatedOriented3DBox_V
>::convert_ros_to_gz(
  const vision_msgs::msg::Detection3DArray & ros_msg,
  ignition::msgs::AnnotatedOriented3DBox_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection3DArray,
  ignition::msgs::AnnotatedOriented3DBox_V
>::convert_gz_to_ros(
  const ignition::msgs::AnnotatedOriented3DBox_V & gz_msg,
  vision_msgs::msg::Detection3DArray & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

// Bounding Box 2D

template<>
void
Factory<
  vision_msgs::msg::BoundingBox2D,
  ignition::msgs::AxisAligned2DBox
>::convert_ros_to_gz(
  const vision_msgs::msg::BoundingBox2D & ros_msg,
  ignition::msgs::AxisAligned2DBox & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::BoundingBox2D,
  ignition::msgs::AxisAligned2DBox
>::convert_gz_to_ros(
  const ignition::msgs::AxisAligned2DBox & gz_msg,
  vision_msgs::msg::BoundingBox2D & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection2D,
  ignition::msgs::AnnotatedAxisAligned2DBox
>::convert_ros_to_gz(
  const vision_msgs::msg::Detection2D & ros_msg,
  ignition::msgs::AnnotatedAxisAligned2DBox & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection2D,
  ignition::msgs::AnnotatedAxisAligned2DBox
>::convert_gz_to_ros(
  const ignition::msgs::AnnotatedAxisAligned2DBox & gz_msg,
  vision_msgs::msg::Detection2D & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection2DArray,
  ignition::msgs::AnnotatedAxisAligned2DBox_V
>::convert_ros_to_gz(
  const vision_msgs::msg::Detection2DArray & ros_msg,
  ignition::msgs::AnnotatedAxisAligned2DBox_V & gz_msg)
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  vision_msgs::msg::Detection2DArray,
  ignition::msgs::AnnotatedAxisAligned2DBox_V
>::convert_gz_to_ros(
  const ignition::msgs::AnnotatedAxisAligned2DBox_V & gz_msg,
  vision_msgs::msg::Detection2DArray & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}

}  // namespace ros_gz_bridge
