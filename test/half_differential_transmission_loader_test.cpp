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

#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_loader.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "pal_transmissions/half_differential_transmission.hpp"
#include "pal_transmissions/half_differential_transmission_loader.hpp"

using testing::DoubleNear;
using testing::SizeIs;

// Floating-point value comparison threshold
const double EPS = 1e-5;

class TransmissionPluginLoader
{
public:
  std::shared_ptr<transmission_interface::TransmissionLoader> create(const std::string & type)
  {
    try {
      return class_loader_.createUniqueInstance(type);
    } catch (std::exception & ex) {
      std::cerr << ex.what() << std::endl;
      return std::shared_ptr<transmission_interface::TransmissionLoader>();
    }
  }

private:
  // must keep it alive because instance destroyers need it
  pluginlib::ClassLoader<transmission_interface::TransmissionLoader> class_loader_ = {
    "transmission_interface", "transmission_interface::TransmissionLoader"};
};

TEST(HalfDifferentialTransmissionLoaderTest, FullSpec)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
    R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>pal_transmissions/HalfDifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>50</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>-50</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <offset>0.5</offset>
            <mechanical_reduction>2.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <offset>-0.5</offset>
            <mechanical_reduction>-2.0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";

  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  pal_transmissions::HalfDifferentialTransmission * half_differential_transmission =
    dynamic_cast<pal_transmissions::HalfDifferentialTransmission *>(transmission.get());
  ASSERT_TRUE(nullptr != half_differential_transmission);

  const std::vector<double> & actuator_reduction =
    half_differential_transmission->get_actuator_reduction();
  EXPECT_THAT(50.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(-50.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction =
    half_differential_transmission->get_joint_reduction();
  EXPECT_THAT(2.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(-2.0, DoubleNear(joint_reduction[1], EPS));

  const std::vector<double> & joint_offset = half_differential_transmission->get_joint_offset();
  EXPECT_THAT(0.5, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(-0.5, DoubleNear(joint_offset[1], EPS));

  const std::unordered_map<std::string,
    int> & actuator_role_map = half_differential_transmission->get_actuator_roles_map();
  EXPECT_EQ(actuator_role_map.at("joint1_motor"), 0);
  EXPECT_EQ(actuator_role_map.at("joint2_motor"), 1);

  const std::unordered_map<std::string,
    int> & joint_role_map = half_differential_transmission->get_joint_roles_map();
  EXPECT_EQ(joint_role_map.at("joint1"), 0);
  EXPECT_EQ(joint_role_map.at("joint2"), 1);
}

TEST(HalfDifferentialTransmissionLoaderTest, Permutation)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
    R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>pal_transmissions/HalfDifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator2">
            <mechanical_reduction>50</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator1">
            <mechanical_reduction>-50</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint2">
            <offset>0.5</offset>
            <mechanical_reduction>2.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint1">
            <offset>-0.5</offset>
            <mechanical_reduction>-2.0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";

  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  pal_transmissions::HalfDifferentialTransmission * half_differential_transmission =
    dynamic_cast<pal_transmissions::HalfDifferentialTransmission *>(transmission.get());
  ASSERT_TRUE(nullptr != half_differential_transmission);

  const std::vector<double> & actuator_reduction =
    half_differential_transmission->get_actuator_reduction();
  EXPECT_THAT(-50.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(50.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction =
    half_differential_transmission->get_joint_reduction();
  EXPECT_THAT(-2.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(2.0, DoubleNear(joint_reduction[1], EPS));

  const std::vector<double> & joint_offset = half_differential_transmission->get_joint_offset();
  EXPECT_THAT(-0.5, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(0.5, DoubleNear(joint_offset[1], EPS));

  const std::unordered_map<std::string,
    int> & actuator_role_map = half_differential_transmission->get_actuator_roles_map();
  EXPECT_EQ(actuator_role_map.at("joint1_motor"), 1);
  EXPECT_EQ(actuator_role_map.at("joint2_motor"), 0);

  const std::unordered_map<std::string,
    int> & joint_role_map = half_differential_transmission->get_joint_roles_map();
  EXPECT_EQ(joint_role_map.at("joint1"), 1);
  EXPECT_EQ(joint_role_map.at("joint2"), 0);
}

TEST(HalfDifferentialTransmissionLoaderTest, InvalidValue1)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
    R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>pal_transmissions/HalfDifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>0</mechanical_reduction>
            <offset>0.5</offset>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>0</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <offset>2</offset>
            <mechanical_reduction>0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <offset>3</offset>
            <mechanical_reduction>0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);
  ASSERT_TRUE(nullptr == transmission);
}

TEST(HalfDifferentialTransmissionLoaderTest, InvalidValue2)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
    R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>pal_transmissions/HalfDifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>two</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>10.0</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <offset>2</offset>
            <mechanical_reduction>20.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <offset>3</offset>
            <mechanical_reduction>five</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  pal_transmissions::HalfDifferentialTransmission * half_differential_transmission =
    dynamic_cast<pal_transmissions::HalfDifferentialTransmission *>(transmission.get());
  ASSERT_TRUE(nullptr != half_differential_transmission);

  const std::vector<double> & actuator_reduction =
    half_differential_transmission->get_actuator_reduction();
  EXPECT_THAT(1.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(10.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction =
    half_differential_transmission->get_joint_reduction();
  EXPECT_THAT(20.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(1.0, DoubleNear(joint_reduction[1], EPS));

  const std::vector<double> & joint_offset = half_differential_transmission->get_joint_offset();
  EXPECT_THAT(2.0, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(3.0, DoubleNear(joint_offset[1], EPS));

  const std::unordered_map<std::string,
    int> & actuator_role_map = half_differential_transmission->get_actuator_roles_map();
  EXPECT_EQ(actuator_role_map.at("joint1_motor"), 0);
  EXPECT_EQ(actuator_role_map.at("joint2_motor"), 1);

  const std::unordered_map<std::string,
    int> & joint_role_map = half_differential_transmission->get_joint_roles_map();
  EXPECT_EQ(joint_role_map.at("joint1"), 0);
  EXPECT_EQ(joint_role_map.at("joint2"), 1);
}

TEST(HalfDifferentialTransmissionLoaderTest, InvalidValue3)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
    R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>pal_transmissions/HalfDifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>5.0</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>5.0</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <mechanical_reduction>5.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <mechanical_reduction>5.0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  pal_transmissions::HalfDifferentialTransmission * half_differential_transmission =
    dynamic_cast<pal_transmissions::HalfDifferentialTransmission *>(transmission.get());
  ASSERT_TRUE(nullptr != half_differential_transmission);

  const std::vector<double> & actuator_reduction =
    half_differential_transmission->get_actuator_reduction();
  EXPECT_THAT(5.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(5.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction =
    half_differential_transmission->get_joint_reduction();
  EXPECT_THAT(5.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(5.0, DoubleNear(joint_reduction[1], EPS));

  const std::vector<double> & joint_offset = half_differential_transmission->get_joint_offset();
  EXPECT_THAT(0.0, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(0.0, DoubleNear(joint_offset[1], EPS));

  const std::unordered_map<std::string,
    int> & actuator_role_map = half_differential_transmission->get_actuator_roles_map();
  EXPECT_EQ(actuator_role_map.at("joint1_motor"), 0);
  EXPECT_EQ(actuator_role_map.at("joint2_motor"), 1);

  const std::unordered_map<std::string,
    int> & joint_role_map = half_differential_transmission->get_joint_roles_map();
  EXPECT_EQ(joint_role_map.at("joint1"), 0);
  EXPECT_EQ(joint_role_map.at("joint2"), 1);
}

TEST(HalfDifferentialTransmissionLoaderTest, InvalidValue4)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
    R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>pal_transmissions/HalfDifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>1.0</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
          </actuator>
          <joint name="joint1" role="joint1">
          </joint>
          <joint name="joint2" role="joint2">
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);

  // Validate transmission
  pal_transmissions::HalfDifferentialTransmission * half_differential_transmission =
    dynamic_cast<pal_transmissions::HalfDifferentialTransmission *>(transmission.get());
  ASSERT_TRUE(nullptr != half_differential_transmission);

  const std::vector<double> & actuator_reduction =
    half_differential_transmission->get_actuator_reduction();
  EXPECT_THAT(1.0, DoubleNear(actuator_reduction[0], EPS));
  EXPECT_THAT(1.0, DoubleNear(actuator_reduction[1], EPS));

  const std::vector<double> & joint_reduction =
    half_differential_transmission->get_joint_reduction();
  EXPECT_THAT(1.0, DoubleNear(joint_reduction[0], EPS));
  EXPECT_THAT(1.0, DoubleNear(joint_reduction[1], EPS));

  const std::vector<double> & joint_offset = half_differential_transmission->get_joint_offset();
  EXPECT_THAT(0.0, DoubleNear(joint_offset[0], EPS));
  EXPECT_THAT(0.0, DoubleNear(joint_offset[1], EPS));

  const std::unordered_map<std::string,
    int> & actuator_role_map = half_differential_transmission->get_actuator_roles_map();
  EXPECT_EQ(actuator_role_map.at("joint1_motor"), 0);
  EXPECT_EQ(actuator_role_map.at("joint2_motor"), 1);

  const std::unordered_map<std::string,
    int> & joint_role_map = half_differential_transmission->get_joint_roles_map();
  EXPECT_EQ(joint_role_map.at("joint1"), 0);
  EXPECT_EQ(joint_role_map.at("joint2"), 1);
}

TEST(HalfDifferentialTransmissionLoaderTest, InvalidValue5)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
    R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>pal_transmissions/HalfDifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>1.0</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator1">
            <mechanical_reduction>1.0</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <offset>2</offset>
            <mechanical_reduction>1.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <offset>3</offset>
            <mechanical_reduction>1.0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);
  ASSERT_TRUE(nullptr == transmission);
}

TEST(HalfDifferentialTransmissionLoaderTest, InvalidValue6)
{
  // Parse transmission info
  std::string urdf_to_test = std::string(ros2_control_test_assets::urdf_head) +
    R"(
      <ros2_control name="FullSpec" type="system">
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
        <transmission name="transmission1">
          <plugin>pal_transmissions/HalfDifferentialTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>1.0</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>1.0</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint3">
            <offset>2</offset>
            <mechanical_reduction>1.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint4">
            <offset>3</offset>
            <mechanical_reduction>1.0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";
  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);
  ASSERT_THAT(infos[0].transmissions, SizeIs(1));

  // Transmission loader
  TransmissionPluginLoader loader;
  std::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader =
    loader.create(infos[0].transmissions[0].type);
  ASSERT_TRUE(nullptr != transmission_loader);

  std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
  const hardware_interface::TransmissionInfo & info = infos[0].transmissions[0];
  transmission = transmission_loader->load(info);
  ASSERT_TRUE(nullptr == transmission);
}
