// Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, softwact_reduction_e
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT Wact_reduction_RANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pluginlib/class_list_macros.hpp"
#include "pal_transmissions/half_differential_transmission.hpp"
#include "pal_transmissions/half_differential_transmission_loader.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

namespace pal_transmissions
{

std::shared_ptr<transmission_interface::Transmission>
HalfDifferentialTransmissionLoader::load(
  const hardware_interface::TransmissionInfo & transmission_info)
{
  try {
    const auto act_name1 = transmission_info.actuators.at(0).name;
    const auto act_name2 = transmission_info.actuators.at(1).name;
    const auto act_role1 = transmission_info.actuators.at(0).role;
    const auto act_role2 = transmission_info.actuators.at(1).role;
    std::unordered_map<std::string, int> actuator_roles;

    if (act_role1 == "actuator1" && act_role2 == "actuator2") {
      actuator_roles[act_name1] = 0;
      actuator_roles[act_name2] = 1;
    } else if (act_role1 == "actuator2" && act_role2 == "actuator1") {
      actuator_roles[act_name1] = 1;
      actuator_roles[act_name2] = 0;
    } else {
      throw std::runtime_error("Actuator roles must be 'actuator1' or 'actuator2'");
    }

    const auto jnt_name1 = transmission_info.joints.at(0).name;
    const auto jnt_name2 = transmission_info.joints.at(1).name;
    const auto jnt_role1 = transmission_info.joints.at(0).role;
    const auto jnt_role2 = transmission_info.joints.at(1).role;
    std::unordered_map<std::string, int> joint_roles;

    if (jnt_role1 == "joint1" && jnt_role2 == "joint2") {
      joint_roles[jnt_name1] = 0;
      joint_roles[jnt_name2] = 1;
    } else if (jnt_role1 == "joint2" && jnt_role2 == "joint1") {
      joint_roles[jnt_name1] = 1;
      joint_roles[jnt_name2] = 0;
    } else {
      throw std::runtime_error("Joint roles must be 'joint1' or 'joint2'");
    }

    // Ensure the actuators are in the correct order
    const auto act_reduction1 =
      transmission_info.actuators.at(actuator_roles[act_name1]).mechanical_reduction;
    const auto act_reduction2 =
      transmission_info.actuators.at(actuator_roles[act_name2]).mechanical_reduction;

    const auto jnt_reduction1 =
      transmission_info.joints.at(joint_roles[jnt_name1]).mechanical_reduction;
    const auto jnt_reduction2 =
      transmission_info.joints.at(joint_roles[jnt_name2]).mechanical_reduction;

    const auto jnt_offset1 = transmission_info.joints.at(joint_roles[jnt_name1]).offset;
    const auto jnt_offset2 = transmission_info.joints.at(joint_roles[jnt_name2]).offset;

    // std::cerr << act_reduction1 << ", " << act_reduction2 << ", " << jnt_reduction1 << ", " <<
    //   jnt_reduction2 << ", " << jnt_offset1 << ", " << jnt_offset2 << std::endl;

    std::shared_ptr<transmission_interface::Transmission> transmission(new pal_transmissions::
      HalfDifferentialTransmission(
        {act_reduction1, act_reduction2}, {jnt_reduction1, jnt_reduction2},
        {jnt_offset1, jnt_offset2}, actuator_roles, joint_roles));
    return transmission;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("half_differential_transmission_loader"),
      "Failed to construct transmission '%s'", ex.what());
    return nullptr;
  }
}

} // namespace pal_transmissions

PLUGINLIB_EXPORT_CLASS(
  pal_transmissions::HalfDifferentialTransmissionLoader,
  transmission_interface::TransmissionLoader)
