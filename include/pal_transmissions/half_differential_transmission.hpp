// Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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

#ifndef HALF_DIFFERENTIAL_TRANSMISSION_H
#define HALF_DIFFERENTIAL_TRANSMISSION_H

#include <unordered_map>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/accessor.hpp"
#include "transmission_interface/exception.hpp"
#include "pal_logger/pal_logger.hpp"

using namespace std::literals::chrono_literals;


namespace pal_transmissions
{
class HalfDifferentialTransmission : public transmission_interface::Transmission
{
public:
  HalfDifferentialTransmission(
    const std::vector<double> & actuator_reduction,
    const std::vector<double> & joint_reduction,
    const std::vector<double> & joint_offset,
    const std::unordered_map<std::string, int> & actuator_roles,
    const std::unordered_map<std::string, int> & joint_roles);

  virtual ~HalfDifferentialTransmission() = default;

  void configure(
    const std::vector<transmission_interface::JointHandle> & joint_handles,
    const std::vector<transmission_interface::ActuatorHandle> & actuator_handles) override;

  std::size_t num_actuators() const override {return 2;}
  std::size_t num_joints() const override {return 2;}

  void actuator_to_joint() override;

  void joint_to_actuator() override;

  const std::vector<double> & get_actuator_reduction() const {return act_reduction_;}
  const std::vector<double> & get_joint_reduction() const {return jnt_reduction_;}
  const std::vector<double> & get_joint_offset() const {return jnt_offset_;}
  const std::unordered_map<std::string, int> & get_actuator_roles_map() {return actuator_roles_;}
  const std::unordered_map<std::string, int> & get_joint_roles_map() {return joint_roles_;}

protected:
  std::string get_handles_info() const;

  void actuatorToJointPosition();

  void actuatorToJointVelocity();

  void actuatorToJointEffort();

  void actuatorToJointAbsolutePosition();

  void actuatorToJointTorqueSensor();

  void jointToActuatorPosition();

  void jointToActuatorVelocity();

  void jointToActuatorEffort();

  std::vector<double> act_reduction_;
  std::vector<double> jnt_reduction_;
  std::vector<double> jnt_offset_;

  std::unordered_map<std::string, int> actuator_roles_;
  std::unordered_map<std::string, int> joint_roles_;

  bool needsZeroCalibration_;

  std::vector<transmission_interface::JointHandle> joint_position_;
  std::vector<transmission_interface::JointHandle> joint_velocity_;
  std::vector<transmission_interface::JointHandle> joint_effort_;
  std::vector<transmission_interface::JointHandle> joint_abs_position_;
  std::vector<transmission_interface::JointHandle> joint_torque_sensor_;

  std::vector<transmission_interface::ActuatorHandle> actuator_position_;
  std::vector<transmission_interface::ActuatorHandle> actuator_velocity_;
  std::vector<transmission_interface::ActuatorHandle> actuator_effort_;
  std::vector<transmission_interface::ActuatorHandle> actuator_abs_position_;
  std::vector<transmission_interface::ActuatorHandle> actuator_torque_sensor_;
};

HalfDifferentialTransmission::HalfDifferentialTransmission(
  const std::vector<double> & actuator_reduction,
  const std::vector<double> & joint_reduction,
  const std::vector<double> & joint_offset,
  const std::unordered_map<std::string, int> & actuator_roles,
  const std::unordered_map<std::string, int> & joint_roles)
: act_reduction_(actuator_reduction),
  jnt_reduction_(joint_reduction),
  jnt_offset_(joint_offset),
  actuator_roles_(actuator_roles),
  joint_roles_(joint_roles),
  needsZeroCalibration_(true)
{
  if (num_actuators() != act_reduction_.size() || num_joints() != jnt_reduction_.size() ||
    num_joints() != jnt_offset_.size() || num_actuators() != actuator_roles_.size() ||
    num_joints() != joint_roles_.size())
  {
    throw transmission_interface::Exception("Reduction and offset vectors must have size 2.");
  }

  if (std::abs(act_reduction_[0]) < 1.e-5 ||
    std::abs(act_reduction_[1]) < 1.e-5 ||
    std::abs(jnt_reduction_[0]) < 1.e-5 ||
    std::abs(jnt_reduction_[1]) < 1.e-5)
  {
    throw transmission_interface::Exception("Transmission reduction ratios cannot be zero.");
  }
}

std::string HalfDifferentialTransmission::get_handles_info() const
{
  return std::string("Got the following handles:\n") +
         "Joint position: " +
         transmission_interface::to_string(transmission_interface::get_names(joint_position_)) +
         ", Actuator position: " + transmission_interface::to_string(
    transmission_interface::get_names(
      actuator_position_)) + "\n" +
         "Joint velocity: " +
         transmission_interface::to_string(transmission_interface::get_names(joint_velocity_)) +
         ", Actuator velocity: " + transmission_interface::to_string(
    transmission_interface::get_names(
      actuator_velocity_)) + "\n" +
         "Joint effort: " +
         transmission_interface::to_string(transmission_interface::get_names(joint_effort_)) +
         ", Actuator effort: " + transmission_interface::to_string(
    transmission_interface::get_names(
      actuator_effort_));
}

void HalfDifferentialTransmission::configure(
  const std::vector<transmission_interface::JointHandle> & joint_handles,
  const std::vector<transmission_interface::ActuatorHandle> & actuator_handles)
{
  needsZeroCalibration_ = true;

  if (joint_handles.empty()) {
    throw transmission_interface::Exception("No joint handles were passed in");
  }

  if (actuator_handles.empty()) {
    throw transmission_interface::Exception("No actuator handles were passed in");
  }

  const auto joint_names = transmission_interface::get_names(joint_handles);
  if (joint_names.size() != 2) {
    throw transmission_interface::Exception(
            "There should be exactly two unique joint names but was given " + transmission_interface::to_string(
              joint_names));
  }
  const auto actuator_names = transmission_interface::get_names(actuator_handles);
  if (actuator_names.size() != 2) {
    throw transmission_interface::Exception(
            "There should be exactly two unique actuator names but was given " +
            transmission_interface::to_string(actuator_names));
  }

  std::vector<std::string> ordered_joint_names(joint_names.size());
  for (auto name : joint_names) {
    ordered_joint_names[joint_roles_.at(name)] = name;
  }

  joint_position_ =
    get_ordered_handles(joint_handles, ordered_joint_names, hardware_interface::HW_IF_POSITION);
  joint_velocity_ =
    get_ordered_handles(joint_handles, ordered_joint_names, hardware_interface::HW_IF_VELOCITY);
  joint_effort_ = get_ordered_handles(
    joint_handles, ordered_joint_names,
    hardware_interface::HW_IF_EFFORT);
  joint_abs_position_ = get_ordered_handles(
    joint_handles, ordered_joint_names,
    hardware_interface::HW_IF_ACCELERATION);
  joint_torque_sensor_ = get_ordered_handles(
    joint_handles, ordered_joint_names,
    hardware_interface::HW_IF_FORCE);

  if (joint_position_.size() != 2 && joint_velocity_.size() != 2 && joint_effort_.size() != 2 &&
    joint_abs_position_.size() != 2 &&
    joint_torque_sensor_.size() != 2)
  {
    throw transmission_interface::Exception(
            "Not enough valid or required joint handles were presented. \n" + get_handles_info());
  }

  std::vector<std::string> ordered_actuator_names(actuator_names.size());
  for (auto name : actuator_names) {
    ordered_actuator_names[actuator_roles_.at(name)] = name;
  }

  actuator_position_ =
    get_ordered_handles(
    actuator_handles, ordered_actuator_names,
    hardware_interface::HW_IF_POSITION);
  actuator_velocity_ =
    get_ordered_handles(
    actuator_handles, ordered_actuator_names,
    hardware_interface::HW_IF_VELOCITY);
  actuator_effort_ =
    get_ordered_handles(actuator_handles, ordered_actuator_names, hardware_interface::HW_IF_EFFORT);
  actuator_abs_position_ =
    get_ordered_handles(
    actuator_handles, ordered_actuator_names,
    hardware_interface::HW_IF_ACCELERATION);
  actuator_torque_sensor_ =
    get_ordered_handles(actuator_handles, ordered_actuator_names, hardware_interface::HW_IF_FORCE);

  if (actuator_position_.size() != 2 && actuator_velocity_.size() != 2 &&
    actuator_effort_.size() != 2 && actuator_abs_position_.size() != 2 &&
    actuator_torque_sensor_.size() != 2)
  {
    throw transmission_interface::Exception(
            "Not enough valid or required actuator handles were presented. \n" +
            get_handles_info());
  }

  if (joint_position_.size() != actuator_position_.size() &&
    joint_velocity_.size() != actuator_velocity_.size() &&
    joint_effort_.size() != actuator_effort_.size() &&
    joint_abs_position_.size() != actuator_abs_position_.size() &&
    joint_torque_sensor_.size() != actuator_torque_sensor_.size())
  {
    throw transmission_interface::Exception(
            "Pair-wise mismatch on interfaces. \n" + get_handles_info());
  }
}

void HalfDifferentialTransmission::actuatorToJointEffort()
{
  (void)joint_effort_[0].set_value(
    (actuator_effort_[0].get_optional().value() * act_reduction_[0]) * jnt_reduction_[0]);
  (void)joint_effort_[1].set_value(
      ((actuator_effort_[1].get_optional().value() * act_reduction_[1]) +
    (joint_effort_[0].get_optional().value() / jnt_reduction_[0])) * jnt_reduction_[1]);
}

void HalfDifferentialTransmission::actuatorToJointVelocity()
{
  (void)joint_velocity_[0].set_value(
    (actuator_velocity_[0].get_optional().value() / act_reduction_[0]) / jnt_reduction_[0]);
  (void)joint_velocity_[1].set_value(
    (actuator_velocity_[1].get_optional().value() / act_reduction_[1] -
    actuator_velocity_[0].get_optional().value() / act_reduction_[0]) / jnt_reduction_[1]);
}

void HalfDifferentialTransmission::actuatorToJointPosition()
{
  (void)joint_position_[0].set_value(
    (actuator_position_[0].get_optional().value() / act_reduction_[0]) / jnt_reduction_[0] +
    jnt_offset_[0]);
  (void)joint_position_[1].set_value(
    (actuator_position_[1].get_optional().value() / act_reduction_[1] -
    actuator_position_[0].get_optional().value() / act_reduction_[0]) /
    jnt_reduction_[1] + jnt_offset_[1]);

  if (needsZeroCalibration_ && actuator_position_[0].get_optional().has_value() &&
    actuator_position_[1].get_optional().has_value() &&
    actuator_abs_position_[0].get_optional().has_value() &&
    actuator_abs_position_[1].get_optional().has_value() &&
    std::isfinite(actuator_position_[0].get_optional().value()) &&
    std::isfinite(actuator_position_[1].get_optional().value()) &&
    std::isfinite(actuator_abs_position_[0].get_optional().value()) &&
    std::isfinite(actuator_abs_position_[1].get_optional().value()))
  {

    // @Note: This is always zero because absolute is equal to joint and actuator starts with zero!!
    jnt_offset_[0] = actuator_abs_position_[0].get_optional().value() -
      joint_position_[0].get_optional().value();
    jnt_offset_[1] = actuator_abs_position_[1].get_optional().value() -
      joint_position_[1].get_optional().value();

    (void)joint_position_[0].set_value(
      (actuator_position_[0].get_optional().value() / act_reduction_[0]) / ( jnt_reduction_[0]) +
      jnt_offset_[0]);
    (void)joint_position_[1].set_value(
      (actuator_position_[1].get_optional().value() / act_reduction_[1] -
      actuator_position_[0].get_optional().value() / act_reduction_[0]) /
      (jnt_reduction_[1]) + jnt_offset_[1]);

    needsZeroCalibration_ = false;
  } else if (needsZeroCalibration_) {
    PL_INFO_THROTTLE(pal_log::PalLog::get_logger(), 1s, "Waiting for head initialization");
  }
}

void HalfDifferentialTransmission::actuatorToJointAbsolutePosition()
{
  (void)joint_abs_position_[0].set_value(actuator_abs_position_[0].get_optional().value());
  (void)joint_abs_position_[1].set_value(actuator_abs_position_[1].get_optional().value());
  (void)joint_position_[0].set_value(actuator_abs_position_[0].get_optional().value());
  (void)joint_position_[1].set_value(actuator_abs_position_[1].get_optional().value());
}

void HalfDifferentialTransmission::actuatorToJointTorqueSensor()
{
  (void)joint_torque_sensor_[0].set_value(
    (actuator_torque_sensor_[0].get_optional().value() *
    act_reduction_[0]));
  (void)joint_torque_sensor_[1].set_value(
    jnt_reduction_[1] *
    (actuator_torque_sensor_[0].get_optional().value() * act_reduction_[0] +
    actuator_torque_sensor_[1].get_optional().value() * act_reduction_[1]));
}

void HalfDifferentialTransmission::jointToActuatorEffort()
{
  (void)actuator_effort_[0].set_value(
    (joint_effort_[0].get_optional().value() / jnt_reduction_[0]) / act_reduction_[0]);
  // @Note This equation is not correct. This one is not bijective.
  (void)actuator_effort_[1].set_value(
    (joint_effort_[1].get_optional().value() / act_reduction_[1] +
    joint_effort_[0].get_optional().value() / jnt_reduction_[0]) /
    jnt_reduction_[1]);
  // @Note This one is bijective.
  // (void)actuator_effort_[1].set_value(
  //     (joint_effort_[1].get_optional().value() / jnt_reduction_[1] -
  //      joint_effort_[0].get_optional().value() / jnt_reduction_[0]) /
  //     act_reduction_[1]);
}

void HalfDifferentialTransmission::jointToActuatorVelocity()
{
  (void)actuator_velocity_[0].set_value(
    (joint_velocity_[0].get_optional().value() * jnt_reduction_[0]) * act_reduction_[0]);
  (void)actuator_velocity_[1].set_value(
    (( joint_velocity_[1].get_optional().value()) * jnt_reduction_[1] +
    (actuator_velocity_[0].get_optional().value() / act_reduction_[0])) *
    act_reduction_[1]);
}

void HalfDifferentialTransmission::jointToActuatorPosition()
{
  // @Note This equation is not correct. This one is not bijective.
  (void)actuator_position_[0].set_value(
    (joint_position_[0].get_optional().value() * jnt_reduction_[0]) * act_reduction_[0] -
    jnt_offset_[0]);
  (void)actuator_position_[1].set_value(
    (( joint_position_[1].get_optional().value() - jnt_offset_[1]) * jnt_reduction_[1] +
    (actuator_position_[0].get_optional().value() / act_reduction_[0])) * act_reduction_[1]);
}

void HalfDifferentialTransmission::actuator_to_joint()
{
  actuatorToJointAbsolutePosition();
  actuatorToJointPosition();
  actuatorToJointVelocity();
  actuatorToJointEffort();
  actuatorToJointTorqueSensor();
}

void HalfDifferentialTransmission::joint_to_actuator()
{
  jointToActuatorPosition();
  jointToActuatorVelocity();
  jointToActuatorEffort();
}

} // pal_transmissions

#endif // HALF_DIFFERENTIAL_TRANSMISSION_H
