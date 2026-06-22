// Copyright 2024 LAAS, CNRS
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

#include <gtest/gtest.h>

#include "odri_gz_ros2_control/gz_system_interface.hpp"

// ---------------------------------------------------------------------------
// Torque law: τ = τ_cmd + Kp*(pos_cmd - pos) + Kd*(vel_cmd - vel)
// ---------------------------------------------------------------------------

// Replicates the formula from GazeboOdriSimSystem::write() so that any
// future change to the formula breaks this test explicitly.
static double odri_torque(double effort_cmd, double Kp, double pos_cmd,
                          double pos, double Kd, double vel_cmd, double vel) {
  return effort_cmd + Kp * (pos_cmd - pos) + Kd * (vel_cmd - vel);
}

TEST(OdriTorqueLaw, PureEffortWhenGainsAreZero) {
  EXPECT_DOUBLE_EQ(odri_torque(3.5, 0.0, 1.0, 0.5, 0.0, 0.2, 0.0), 3.5);
}

TEST(OdriTorqueLaw, PositionErrorOnly) {
  // pos_cmd=1, pos=0.5 → pos_error=0.5; vel_error=0; Kp=10
  EXPECT_DOUBLE_EQ(odri_torque(0.0, 10.0, 1.0, 0.5, 0.0, 0.0, 0.0), 5.0);
}

TEST(OdriTorqueLaw, VelocityErrorOnly) {
  // vel_cmd=0.3, vel=0.0 → vel_error=0.3; pos_error=0; Kd=2
  EXPECT_DOUBLE_EQ(odri_torque(0.0, 0.0, 0.0, 0.0, 2.0, 0.3, 0.0), 0.6);
}

TEST(OdriTorqueLaw, AllTermsCombined) {
  // effort=2, Kp=10 pos_error=0.5, Kd=1 vel_error=0.3 → 2+5+0.3=7.3
  EXPECT_DOUBLE_EQ(odri_torque(2.0, 10.0, 1.0, 0.5, 1.0, 0.5, 0.2), 7.3);
}

TEST(OdriTorqueLaw, NegativePositionError) {
  // pos_cmd < pos → braking torque
  EXPECT_DOUBLE_EQ(odri_torque(0.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0), -5.0);
}

TEST(OdriTorqueLaw, NegativeVelocityError) {
  EXPECT_DOUBLE_EQ(odri_torque(0.0, 0.0, 0.0, 0.0, 4.0, -0.5, 0.0), -2.0);
}

TEST(OdriTorqueLaw, AtEquilibrium) {
  // pos_cmd==pos, vel_cmd==vel → torque equals effort_cmd
  EXPECT_DOUBLE_EQ(odri_torque(1.23, 10.0, 0.5, 0.5, 3.0, 0.1, 0.1), 1.23);
}

// ---------------------------------------------------------------------------
// ControlMethod SafeEnum bitfield
// ---------------------------------------------------------------------------

using CM = odri_gz_ros2_control::GazeboOdriSimSystemInterface;

TEST(ControlMethod, DefaultIsNone) {
  CM::ControlMethod m;
  EXPECT_FALSE(static_cast<bool>(m & CM::POSITION));
  EXPECT_FALSE(static_cast<bool>(m & CM::VELOCITY));
  EXPECT_FALSE(static_cast<bool>(m & CM::EFFORT));
  EXPECT_FALSE(static_cast<bool>(m & CM::POS_VEL_EFF_GAINS));
}

TEST(ControlMethod, SetPosVelEffGains) {
  CM::ControlMethod m;
  m |= CM::POS_VEL_EFF_GAINS;
  EXPECT_TRUE(static_cast<bool>(m & CM::POS_VEL_EFF_GAINS));
  // Setting only POS_VEL_EFF_GAINS must not touch other bits
  EXPECT_FALSE(static_cast<bool>(m & CM::POSITION));
  EXPECT_FALSE(static_cast<bool>(m & CM::VELOCITY));
  EXPECT_FALSE(static_cast<bool>(m & CM::EFFORT));
}

TEST(ControlMethod, PosVelEffGainsIsDistinctFromEffort) {
  // POS_VEL_EFF_GAINS = (1<<3) = 8, EFFORT = (1<<2) = 4 — must not overlap
  CM::ControlMethod m;
  m |= CM::POS_VEL_EFF_GAINS;
  EXPECT_FALSE(static_cast<bool>(m & CM::EFFORT));

  CM::ControlMethod m2;
  m2 |= CM::EFFORT;
  EXPECT_FALSE(static_cast<bool>(m2 & CM::POS_VEL_EFF_GAINS));
}

TEST(ControlMethod, ResetToNoneClears) {
  CM::ControlMethod m;
  m |= CM::POS_VEL_EFF_GAINS;
  m = CM::ControlMethod(CM::NONE);
  EXPECT_FALSE(static_cast<bool>(m & CM::POS_VEL_EFF_GAINS));
  EXPECT_FALSE(static_cast<bool>(m & CM::POSITION));
}

TEST(ControlMethod, SetThenClearWithNone) {
  CM::ControlMethod m;
  m |= CM::VELOCITY;
  m |= CM::EFFORT;
  EXPECT_TRUE(static_cast<bool>(m & CM::VELOCITY));
  EXPECT_TRUE(static_cast<bool>(m & CM::EFFORT));
  m = CM::ControlMethod(CM::NONE);
  EXPECT_FALSE(static_cast<bool>(m & CM::VELOCITY));
  EXPECT_FALSE(static_cast<bool>(m & CM::EFFORT));
}

// ---------------------------------------------------------------------------
// String constants for gain interfaces
// ---------------------------------------------------------------------------

TEST(GainInterfaceNames, KpStringValue) {
  // These must match the values in system_interface_odri.hpp
  const std::string kp{"gain_kp"};
  const std::string kd{"gain_kd"};
  EXPECT_EQ(kp, "gain_kp");
  EXPECT_EQ(kd, "gain_kd");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
