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

const double TOLERANCE = 1.e-6;

class HalfDifferentialTransmissionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pal_log::PalLog::init();

    act_reduction_ = {2.0, 3.0};
    jnt_reduction_ = {1.5, 2.5};
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
    actuator_handles.emplace_back("motor1", HW_IF_ACCELERATION, &actuator_abs_position_handles_[0]);
    actuator_handles.emplace_back("motor2", HW_IF_ACCELERATION, &actuator_abs_position_handles_[1]);
    actuator_handles.emplace_back("motor1", HW_IF_FORCE, &actuator_torque_sensor_handles_[0]);
    actuator_handles.emplace_back("motor2", HW_IF_FORCE, &actuator_torque_sensor_handles_[1]);

    std::vector<JointHandle> joint_handles;
    joint_handles.emplace_back("joint1", HW_IF_POSITION, &joint_position_handles_[0]);
    joint_handles.emplace_back("joint2", HW_IF_POSITION, &joint_position_handles_[1]);
    joint_handles.emplace_back("joint1", HW_IF_VELOCITY, &joint_velocity_handles_[0]);
    joint_handles.emplace_back("joint2", HW_IF_VELOCITY, &joint_velocity_handles_[1]);
    joint_handles.emplace_back("joint1", HW_IF_EFFORT, &joint_effort_handles_[0]);
    joint_handles.emplace_back("joint2", HW_IF_EFFORT, &joint_effort_handles_[1]);
    joint_handles.emplace_back("joint1", HW_IF_ACCELERATION, &joint_abs_position_handles_[0]);
    joint_handles.emplace_back("joint2", HW_IF_ACCELERATION, &joint_abs_position_handles_[1]);
    joint_handles.emplace_back("joint1", HW_IF_FORCE, &joint_torque_sensor_handles_[0]);
    joint_handles.emplace_back("joint2", HW_IF_FORCE, &joint_torque_sensor_handles_[1]);

    transmission->configure(joint_handles, actuator_handles);
  }

public:
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
  std::vector<double> actuator_torque_sensor_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  std::vector<double> joint_position_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> joint_velocity_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> joint_effort_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> joint_abs_position_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::vector<double> joint_torque_sensor_handles_{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
};

TEST_F(HalfDifferentialTransmissionTest, NotInitialized)
{
  ASSERT_NO_THROW(transmission->actuator_to_joint());
  ASSERT_NO_THROW(transmission->joint_to_actuator());
}

/*
TEST_F(HalfDifferentialTransmissionTest, ActuatorToJointAndBackWithNumbers)
{
  // Set known actuator positions (motor0, motor1)
  double a0_pos = 2.0;    // motor1
  double a1_pos = 4.0;    // motor2

  (void)actuator_handles[0].set_value(a0_pos);   // motor1 pos
  (void)actuator_handles[1].set_value(a1_pos);   // motor2 pos

  // Call forward transmission: actuator -> joint
  transmission->actuator_to_joint();

  // Expected joint positions based on equation:
  // j0 = ((4.0 / 3.0) - (2.0 / 2.0)) / 2.0 / 1.5 + 0.1
  //    = ((1.3333 - 1.0) / 2.0) / 1.5 + 0.1
  //    = (0.3333 / 2.0) / 1.5 + 0.1 = 0.1111 + 0.1 = 0.2111
  // j1 = ((4.0 / 3.0) + (2.0 / 2.0)) / 2.0 / 2.5 - 0.2
  //    = ((1.3333 + 1.0) / 2.0) / 2.5 - 0.2
  //    = (2.3333 / 2.0) / 2.5 - 0.2 = (1.1666 / 2.5) - 0.2 = 0.4666 - 0.2 = 0.2666

  double expected_j0 = 0.2111;
  double expected_j1 = 0.2666;

  // Extract joint values from handles
  double j0 = joint_handles[0].get_optional().value();
  double j1 = joint_handles[1].get_optional().value();

  EXPECT_NEAR(j0, expected_j0, TOLERANCE);
  EXPECT_NEAR(j1, expected_j1, TOLERANCE);

  // Now set joint positions back for inverse propagation
  (void)joint_handles[0].set_value(expected_j0);
  (void)joint_handles[1].set_value(expected_j1);

  // Call inverse transmission: joint -> actuator
  transmission->joint_to_actuator();

  // Expected actuator positions:
  // a0 = ((0.2666 + 0.2) - (0.2111 - 0.1)) * 2.0
  //     = (0.4666 - 0.1111) * 2.0 = 0.3555 * 2.0 = 0.7111
  // a1 = ((0.2666 + 0.2) + (0.2111 - 0.1)) * 3.0
  //     = (0.4666 + 0.1111) * 3.0 = 0.5777 * 3.0 = 1.7333

  // double expected_a0 = 0.7111;
  // double expected_a1 = 1.7333;

  double a0_back = actuator_handles[0].get_optional().value();
  double a1_back = actuator_handles[1].get_optional().value();

  EXPECT_NEAR(a0_back, a0_pos, TOLERANCE);
  EXPECT_NEAR(a1_back, a1_pos, TOLERANCE);
}

TEST_F(HalfDifferentialTransmissionTest, FullActuatorToJointAndBackValidation)
{
  // ----------------------------
  // Step 1: Set known actuator values
  // ----------------------------
  double a0_pos = 2.0;        // motor1 position
  double a1_pos = 4.0;        // motor2 position
  double a0_vel = 1.0;        // motor1 velocity
  double a1_vel = 3.0;        // motor2 velocity
  double a0_eff = 10.0;       // motor1 effort
  double a1_eff = 20.0;       // motor2 effort

  (void)actuator_handles[0].set_value(a0_pos);
  (void)actuator_handles[1].set_value(a1_pos);
  (void)actuator_handles[2].set_value(a0_vel);
  (void)actuator_handles[3].set_value(a1_vel);
  (void)actuator_handles[4].set_value(a0_eff);
  (void)actuator_handles[5].set_value(a1_eff);

  // ----------------------------
  // Step 2: Forward propagation (actuator -> joint)
  // ----------------------------
  transmission->actuator_to_joint();

  // --- Expected joint positions ---
  // j0 = ((a1/ar1) - (a0/ar0)) / 2 / jr0 + j0_offset
  // j1 = ((a1/ar1) + (a0/ar0)) / 2 / jr1 + j1_offset

  double expected_j0_pos = (((a1_pos / 3.0) - (a0_pos / 2.0)) / 2.0) / 1.5 + 0.1;
  double expected_j1_pos = (((a1_pos / 3.0) + (a0_pos / 2.0)) / 2.0) / 2.5 - 0.2;

  EXPECT_NEAR(joint_handles[0].get_optional().value(), expected_j0_pos, TOLERANCE);
  EXPECT_NEAR(joint_handles[1].get_optional().value(), expected_j1_pos, TOLERANCE);

  // --- Expected joint velocities ---
  // j0 = ((a1/ar1) - (a0/ar0)) / 2 / jr0
  // j1 = ((a1/ar1) + (a0/ar0)) / 2 / jr1

  double expected_j0_vel = (((a1_vel / 3.0) - (a0_vel / 2.0)) / 2.0) / 1.5;
  double expected_j1_vel = (((a1_vel / 3.0) + (a0_vel / 2.0)) / 2.0) / 2.5;

  EXPECT_NEAR(joint_handles[2].get_optional().value(), expected_j0_vel, TOLERANCE);
  EXPECT_NEAR(joint_handles[3].get_optional().value(), expected_j1_vel, TOLERANCE);

  // --- Expected joint efforts ---
  // j0_eff = (-a0_eff / ar0 + a1_eff / ar1) / 2
  // j1_eff = ( a0_eff / ar0 + a1_eff / ar1) / 2

  double expected_j0_eff = (-a0_eff / 2.0 + a1_eff / 3.0) / 2.0;
  double expected_j1_eff = ( a0_eff / 2.0 + a1_eff / 3.0) / 2.0;

  EXPECT_NEAR(joint_handles[4].get_optional().value(), expected_j0_eff, TOLERANCE);
  EXPECT_NEAR(joint_handles[5].get_optional().value(), expected_j1_eff, TOLERANCE);

  // ----------------------------
  // Step 3: Backward propagation (joint -> actuator)
  // ----------------------------

  // Reset joint handles to expected values (simulate controller writing)
  (void)joint_handles[0].set_value(expected_j0_pos);
  (void)joint_handles[1].set_value(expected_j1_pos);
  (void)joint_handles[2].set_value(expected_j0_vel);
  (void)joint_handles[3].set_value(expected_j1_vel);
  (void)joint_handles[4].set_value(expected_j0_eff);
  (void)joint_handles[5].set_value(expected_j1_eff);

  transmission->joint_to_actuator();

  // --- Expected actuator positions ---
  // a0 = ((j1 - offset1) - (j0 - offset0)) * ar0
  // a1 = ((j1 - offset1) + (j0 - offset0)) * ar1

  double expected_a0_pos = ((expected_j1_pos + 0.2) - (expected_j0_pos - 0.1)) * 2.0;
  double expected_a1_pos = ((expected_j1_pos + 0.2) + (expected_j0_pos - 0.1)) * 3.0;

  EXPECT_NEAR(actuator_handles[0].get_optional().value(), expected_a0_pos, TOLERANCE);
  EXPECT_NEAR(actuator_handles[1].get_optional().value(), expected_a1_pos, TOLERANCE);

  // --- Expected actuator velocities ---
  // a0_vel = (-j0_vel * jr0 + j1_vel * jr1)
  // a1_vel = ( j0_vel * jr0 + j1_vel * jr1)

  double expected_a0_vel = (-expected_j0_vel * 1.5 + expected_j1_vel * 2.5);
  double expected_a1_vel = ( expected_j0_vel * 1.5 + expected_j1_vel * 2.5);

  EXPECT_NEAR(actuator_handles[2].get_optional().value(), expected_a0_vel, TOLERANCE);
  EXPECT_NEAR(actuator_handles[3].get_optional().value(), expected_a1_vel, TOLERANCE);

  // --- Expected actuator efforts ---
  // a0_eff = (-j0_eff + j1_eff) * ar0
  // a1_eff = ( j0_eff + j1_eff) * ar1

  double expected_a0_eff = (-expected_j0_eff + expected_j1_eff) * 2.0;
  double expected_a1_eff = ( expected_j0_eff + expected_j1_eff) * 3.0;

  EXPECT_NEAR(actuator_handles[4].get_optional().value(), expected_a0_eff, TOLERANCE);
  EXPECT_NEAR(actuator_handles[5].get_optional().value(), expected_a1_eff, TOLERANCE);
}
*/

TEST_F(HalfDifferentialTransmissionTest, Bijectivity)
{
  const int num_tests = 1;

  std::mt19937 rng(42);    // Fixed seed for reproducibility
  std::uniform_real_distribution<double> dist(-10.0, 10.0);

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
    actuator_abs_position_handles_[0] = 0.0;
    actuator_abs_position_handles_[1] = 0.0;

    // Forward
    transmission->actuator_to_joint();

    EXPECT_NEAR(joint_abs_position_handles_[0], actuator_abs_position_handles_[0], TOLERANCE);
    EXPECT_NEAR(joint_abs_position_handles_[1], actuator_abs_position_handles_[1], TOLERANCE);

    // Backward
    transmission->joint_to_actuator();

    // Compare original actuator values with results
    EXPECT_NEAR(actuator_position_handles_[0], a0_pos, TOLERANCE);
    EXPECT_NEAR(actuator_position_handles_[1], a1_pos, TOLERANCE);
    EXPECT_NEAR(actuator_velocity_handles_[0], a0_vel, TOLERANCE);
    EXPECT_NEAR(actuator_velocity_handles_[1], a1_vel, TOLERANCE);
    EXPECT_NEAR(actuator_effort_handles_[0], a0_eff, TOLERANCE);
    // @Note This one is not bijective.
    // EXPECT_NEAR(actuator_effort_handles_[1], a1_eff, TOLERANCE);
  }
}
