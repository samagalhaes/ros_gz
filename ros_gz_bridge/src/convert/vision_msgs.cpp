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

#include "convert/utils.hpp"
#include "ros_gz_bridge/convert/vision_msgs.hpp"

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::BoundingBox3D & ros_msg,
  ignition::msgs::Oriented3DBox & gz_msg)
{
  convert_ros_to_gz(ros_msg.center.position, (*gz_msg.mutable_center()));
  convert_ros_to_gz(ros_msg.center.orientation, (*gz_msg.mutable_orientation()));
  convert_ros_to_gz(ros_msg.size, (*gz_msg.mutable_boxsize()));
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::Oriented3DBox & gz_msg,
  vision_msgs::msg::BoundingBox3D & ros_msg)
{
  convert_gz_to_ros(gz_msg.center(), ros_msg.center.position);
  convert_gz_to_ros(gz_msg.orientation(), ros_msg.center.orientation);
  convert_gz_to_ros(gz_msg.boxsize(), ros_msg.size);
}

template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection3D & ros_msg,
  ignition::msgs::AnnotatedOriented3DBox & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.bbox, (*gz_msg.mutable_box()));
  
  unsigned int ros_label = std::stoi(ros_msg.results[0].hypothesis.class_id);
  gz_msg.set_label(ros_label);
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::AnnotatedOriented3DBox & gz_msg,
  vision_msgs::msg::Detection3D & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg.box(), ros_msg.bbox);

  auto &object = ros_msg.results.emplace_back();
  object.hypothesis.class_id = std::to_string(gz_msg.label());
  object.hypothesis.score = 1;
  convert_gz_to_ros(gz_msg.box().center(), object.pose.pose.position);
  convert_gz_to_ros(gz_msg.box().orientation(), object.pose.pose.orientation);
  ros_msg.results.push_back(object);
}

template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection3DArray & ros_msg,
  ignition::msgs::AnnotatedOriented3DBox_V & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  for (auto& ros_detection : ros_msg.detections){
    ignition::msgs::AnnotatedOriented3DBox* gz_detection = gz_msg.add_annotated_box();
    convert_ros_to_gz(ros_detection, (*gz_detection));
  }
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::AnnotatedOriented3DBox_V & gz_msg,
  vision_msgs::msg::Detection3DArray & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  for(auto& gz_box : gz_msg.annotated_box()) {
    auto& ros_box = ros_msg.detections.emplace_back(); 
    convert_gz_to_ros(gz_box, ros_box);
  }
}

template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::BoundingBox2D & ros_msg,
  ignition::msgs::AxisAligned2DBox & gz_msg)
{
  auto ros_center = ros_msg.center.position;

  auto gz_min_corner = gz_msg.mutable_min_corner();
  auto gz_max_corner = gz_msg.mutable_max_corner();

  gz_min_corner->set_x(ros_center.x - ros_msg.size_x / 2.0);
  gz_min_corner->set_y(ros_center.x - ros_msg.size_x / 2.0);

  gz_max_corner->set_x(ros_center.x + ros_msg.size_x / 2.0);
  gz_max_corner->set_y(ros_center.x + ros_msg.size_x / 2.0);
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::AxisAligned2DBox & gz_msg,
  vision_msgs::msg::BoundingBox2D & ros_msg)
{
  auto min_corner = gz_msg.min_corner();
  auto max_corner = gz_msg.max_corner();
  ros_msg.center.position.x = min_corner.x() + (max_corner.x() - min_corner.x()) / 2.0;
  ros_msg.center.position.y = min_corner.y() + (max_corner.y() - min_corner.y()) / 2.0;
  ros_msg.center.theta = 0.0;

  ros_msg.size_x = max_corner.x() - min_corner.x();
  ros_msg.size_y = max_corner.y() - min_corner.y();
}

template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection2D & ros_msg,
  ignition::msgs::AnnotatedAxisAligned2DBox & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.bbox, (*gz_msg.mutable_box()));

  auto result = ros_msg.results.cbegin();
  if (result != ros_msg.results.end())
    gz_msg.set_label(std::stoi(result->hypothesis.class_id));
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::AnnotatedAxisAligned2DBox & gz_msg,
  vision_msgs::msg::Detection2D & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg.box(), ros_msg.bbox);

  auto& result = ros_msg.results.emplace_back();
  result.hypothesis.class_id = std::to_string(gz_msg.label());
  result.hypothesis.score = 1.0;
}

template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection2DArray & ros_msg,
  ignition::msgs::AnnotatedAxisAligned2DBox_V & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  for (auto& ros_detection : ros_msg.detections){
    ignition::msgs::AnnotatedAxisAligned2DBox* gz_detection = gz_msg.add_annotated_box();
    convert_ros_to_gz(ros_detection, (*gz_detection));
  }
}

template<>
void
convert_gz_to_ros(
  const ignition::msgs::AnnotatedAxisAligned2DBox_V & gz_msg,
  vision_msgs::msg::Detection2DArray & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  for (auto& gz_detection : gz_msg.annotated_box()){
    auto& ros_detection = ros_msg.detections.emplace_back();
    convert_gz_to_ros(gz_detection, ros_detection);
  }
}

}  // namespace ros_gz_bridge
