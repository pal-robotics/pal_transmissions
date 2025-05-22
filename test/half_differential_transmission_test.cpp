// Copyright 2025 PAL Robotics S.L.
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

#include <limits>
#include <gtest/gtest.h>
#include <random>
#include <transmission_interface/transmission_interface_exception.hpp>
#include "pal_transmissions/half_differential_transmission.hpp"
#include "hardware_interface/handle.hpp"

using namespace transmission_interface;
using namespace hardware_interface;
using namespace pal_transmissions;

const double TOLERANCE = 1.e-5;

class HalfDifferentialTransmissionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    act_reduction_ = {2.0, 3.0};   // {1.0, 1.0};
    jnt_reduction_ = {4.0, 8.0}; // {1.0, 2.0};
    jnt_offset_ = {0.0, 0.0}; //{0.1, -0.2};

    actuator_roles_ = {{"motor1", 0}, {"motor2", 1}};
    joint_roles_ = {{"joint1", 0}, {"joint2", 1}};

    transmission = std::make_unique<HalfDifferentialTransmission>(
      act_reduction_, jnt_reduction_, jnt_offset_, actuator_roles_, joint_roles_);

    std::vector<ActuatorHandle> actuator_handles;
    actuator_handles.emplace_back("motor1", HW_IF_POSITION, &actuator_position_handles_[0]);
    actuator_handles.emplace_back("motor2", HW_IF_POSITION, &actuator_position_handles_[1]);
    actuator_handles.emplace_back("motor1", HW_IF_VELOCITY, &actuator_velocity_handles_[0]);
    actuator_handles.emplace_back("motor2", HW_IF_VELOCITY, &actuator_velocity_handles_[1]);
    actuator_handles.emplace_back("motor1", HW_IF_EFFORT, &actuator_effort_handles_[0]);
    actuator_handles.emplace_back("motor2", HW_IF_EFFORT, &actuator_effort_handles_[1]);
    actuator_handles.emplace_back(
      "motor1", "absolute_position",
      &actuator_abs_position_handles_[0]);
    actuator_handles.emplace_back(
      "motor2", "absolute_position",
      &actuator_abs_position_handles_[1]);

    std::vector<JointHandle> joint_handles;
    joint_handles.emplace_back("joint1", HW_IF_POSITION, &joint_position_handles_[0]);
    joint_handles.emplace_back("joint2", HW_IF_POSITION, &joint_position_handles_[1]);
    joint_handles.emplace_back("joint1", HW_IF_VELOCITY, &joint_velocity_handles_[0]);
    joint_handles.emplace_back("joint2", HW_IF_VELOCITY, &joint_velocity_handles_[1]);
    joint_handles.emplace_back("joint1", HW_IF_EFFORT, &joint_effort_handles_[0]);
    joint_handles.emplace_back("joint2", HW_IF_EFFORT, &joint_effort_handles_[1]);

    transmission->configure(joint_handles, actuator_handles);
  }

public:
  std::vector<double> actJointPosVelEquations(const std::vector<double> act)
  {
    std::vector<double> jnt(2);
    jnt[0] = act[0] / act_reduction_[0] / jnt_reduction_[0];
    jnt[1] =
      (act[1] / act_reduction_[1] - act[0] /
      act_reduction_[0]) / jnt_reduction_[1];
    return jnt;
  }

  std::vector<double> act_reduction_;
  std::vector<double> jnt_reduction_;
  std::vector<double> jnt_offset_;

  std::unordered_map<std::string, int> actuator_roles_;
  std::unordered_map<std::string, int> joint_roles_;

  std::unique_ptr<HalfDifferentialTransmission> transmission;

  std::vector<double> actuator_position_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> actuator_velocity_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> actuator_effort_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> actuator_abs_position_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  std::vector<double> joint_position_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> joint_velocity_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> joint_effort_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
};

TEST_F(HalfDifferentialTransmissionTest, NotInitialized)
{
  ASSERT_NO_THROW(transmission->actuator_to_joint());
  ASSERT_NO_THROW(transmission->joint_to_actuator());
}

TEST_F(HalfDifferentialTransmissionTest, Bijectivity)
{
  const int num_tests = 100;

  std::mt19937 rng(42);    // Fixed seed for reproducibility
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

  double a0_abs = dist(rng);
  double a1_abs = dist(rng);
  actuator_abs_position_handles_[0] = a0_abs;
  actuator_abs_position_handles_[1] = a1_abs;

  for (int i = 0; i < num_tests; ++i) {

    double a0_pos = dist(rng);
    double a1_pos = dist(rng);
    double a0_vel = dist(rng);
    double a1_vel = dist(rng);
    double a0_eff = dist(rng);
    double a1_eff = dist(rng);

    // Random actuator values
    actuator_position_handles_[0] = a0_pos;
    actuator_position_handles_[1] = a1_pos;
    actuator_velocity_handles_[0] = a0_vel;
    actuator_velocity_handles_[1] = a1_vel;
    actuator_effort_handles_[0] = a0_eff;
    actuator_effort_handles_[1] = a1_eff;

    // Forward
    transmission->actuator_to_joint();

    // Backward
    transmission->joint_to_actuator();

    // Compare original actuator values with results
    EXPECT_NEAR(actuator_position_handles_[0], a0_pos, TOLERANCE);
    EXPECT_NEAR(actuator_position_handles_[1], a1_pos, TOLERANCE);
    EXPECT_NEAR(actuator_velocity_handles_[0], a0_vel, TOLERANCE);
    EXPECT_NEAR(actuator_velocity_handles_[1], a1_vel, TOLERANCE);
    EXPECT_NEAR(actuator_effort_handles_[0], a0_eff, TOLERANCE);
    EXPECT_NEAR(actuator_effort_handles_[1], a1_eff, TOLERANCE);
  }
}

TEST_F(HalfDifferentialTransmissionTest, ActuatorToJointValidation)
{
  const int num_tests = 100;

  std::mt19937 rng(12);      // Fixed seed for reproducibility
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

  actuator_abs_position_handles_[0] = 5.0;
  actuator_abs_position_handles_[1] = 8.0;
  const double jnt_offset0 = 5.13445;
  const double jnt_offset1 = 7.62162;

  for (int i = 0; i < num_tests; ++i) {
    actuator_position_handles_[0] = dist(rng);
    actuator_position_handles_[1] = dist(rng);
    actuator_velocity_handles_[0] = dist(rng);
    actuator_velocity_handles_[1] = dist(rng);
    actuator_effort_handles_[0] = dist(rng);
    actuator_effort_handles_[1] = dist(rng);

    transmission->actuator_to_joint();

    if (i == 0) {
      EXPECT_NEAR(joint_position_handles_[0], actuator_abs_position_handles_[0], TOLERANCE);
      EXPECT_NEAR(joint_position_handles_[1], actuator_abs_position_handles_[1], TOLERANCE);
    } else {
      auto pos_jnt = actJointPosVelEquations(actuator_position_handles_);
      EXPECT_NEAR(joint_position_handles_[0], pos_jnt[0] + jnt_offset0, TOLERANCE);
      EXPECT_NEAR(joint_position_handles_[1], pos_jnt[1] + jnt_offset1, TOLERANCE);
    }

    auto vel_jnt = actJointPosVelEquations(actuator_velocity_handles_);
    EXPECT_NEAR(joint_velocity_handles_[0], vel_jnt[0], TOLERANCE);
    EXPECT_NEAR(joint_velocity_handles_[1], vel_jnt[1], TOLERANCE);

    double expected_j0_eff = actuator_effort_handles_[0] * act_reduction_[0] * jnt_reduction_[0];
    double expected_j1_eff =
      (actuator_effort_handles_[1] * act_reduction_[1] + actuator_effort_handles_[0] *
      act_reduction_[0]) * jnt_reduction_[1];

    EXPECT_NEAR(joint_effort_handles_[0], expected_j0_eff, TOLERANCE);
    EXPECT_NEAR(joint_effort_handles_[1], expected_j1_eff, TOLERANCE);
  }
}
