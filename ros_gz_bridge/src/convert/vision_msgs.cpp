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

// TODO
template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::BoundingBox3D & ros_msg,
  ignition::msgs::Oriented3DBox & gz_msg)
{
//   convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
//   convert_ros_to_gz(ros_msg.pose.pose, (*gz_msg.mutable_pose()));
//   convert_ros_to_gz(ros_msg.twist.twist, (*gz_msg.mutable_twist()));

//   auto childFrame = gz_msg.mutable_header()->add_data();
//   childFrame->set_key("child_frame_id");
//   childFrame->add_value(ros_msg.child_frame_id);
}

// TODO
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

// TODO
template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection3D & ros_msg,
  ignition::msgs::AnnotatedOriented3DBox & gz_msg)
{
//   convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
//   convert_ros_to_gz(ros_msg.pose.pose, (*gz_msg.mutable_pose()));
//   convert_ros_to_gz(ros_msg.twist.twist, (*gz_msg.mutable_twist()));

//   auto childFrame = gz_msg.mutable_header()->add_data();
//   childFrame->set_key("child_frame_id");
//   childFrame->add_value(ros_msg.child_frame_id);
}

// TODO
template<>
void
convert_gz_to_ros(
  const ignition::msgs::AnnotatedOriented3DBox & gz_msg,
  vision_msgs::msg::Detection3D & ros_msg)
{
//   convert_gz_to_ros(gz_msg.header(), ros_msg.header);
//   convert_gz_to_ros(gz_msg.pose(), ros_msg.pose.pose);
//   convert_gz_to_ros(gz_msg.twist(), ros_msg.twist.twist);

//   for (auto i = 0; i < gz_msg.header().data_size(); ++i) {
//     auto a_pair = gz_msg.header().data(i);
//     if (a_pair.key() == "child_frame_id" && a_pair.value_size() > 0) {
//       ros_msg.child_frame_id = frame_id_gz_to_ros(a_pair.value(0));
//       break;
//     }
//   }
}

// TODO: Change this
template<>
void
convert_ros_to_gz(
  const vision_msgs::msg::Detection3DArray & ros_msg,
  ignition::msgs::AnnotatedOriented3DBox_V & gz_msg)
{
//   convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
//   convert_ros_to_gz(ros_msg.pose.pose, (*gz_msg.mutable_pose()));
//   convert_ros_to_gz(ros_msg.twist.twist, (*gz_msg.mutable_twist()));

//   auto childFrame = gz_msg.mutable_header()->add_data();
//   childFrame->set_key("child_frame_id");
//   childFrame->add_value(ros_msg.child_frame_id);
}

// TODO: change this
template<>
void
convert_gz_to_ros(
  const ignition::msgs::AnnotatedOriented3DBox_V & gz_msg,
  vision_msgs::msg::Detection3DArray & ros_msg)
{
//   convert_gz_to_ros(gz_msg.header(), ros_msg.header);
//   convert_gz_to_ros(gz_msg.pose(), ros_msg.pose.pose);
//   convert_gz_to_ros(gz_msg.twist(), ros_msg.twist.twist);

//   for (auto i = 0; i < gz_msg.header().data_size(); ++i) {
//     auto a_pair = gz_msg.header().data(i);
//     if (a_pair.key() == "child_frame_id" && a_pair.value_size() > 0) {
//       ros_msg.child_frame_id = frame_id_gz_to_ros(a_pair.value(0));
//       break;
//     }
//   }
}

}  // namespace ros_gz_bridge
