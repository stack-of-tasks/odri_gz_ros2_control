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

#include "odri_gz_ros2_control/gz_system.hpp"

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/physics/Geometry.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>


// String constants matching
// ros2_hardware_interface_odri/system_interface_odri.hpp
namespace ros2_control_odri {
constexpr const char *HW_IF_GAIN_KP = "gain_kp";
constexpr const char *HW_IF_GAIN_KD = "gain_kd";
}  // namespace ros2_control_odri

struct jointData {
  std::string name;
  sdf::JointType joint_type;
  sdf::JointAxis joint_axis;
  double joint_position;
  double joint_velocity;
  double joint_effort;
  double joint_Kp;
  double joint_Kd;
  double joint_position_cmd;
  double joint_velocity_cmd;
  double joint_effort_cmd;
  double joint_Kp_cmd;
  double joint_Kd_cmd;
  bool is_actuated;
  sim::Entity sim_joint;
  odri_gz_ros2_control::GazeboOdriSimSystemInterface::ControlMethod
      joint_control_method;
  std::string if_name_position;
  std::string if_name_velocity;
  std::string if_name_effort;
  std::string if_name_gain_kp;
  std::string if_name_gain_kd;
};

class ForceTorqueData {
 public:
  std::string name{};
  std::string topicName{};
  sim::Entity sim_ft_sensors_ = sim::kNullEntity;
  std::array<double, 6> ft_sensor_data_;
  void OnForceTorque(const gz::msgs::Wrench &_msg);
};

void ForceTorqueData::OnForceTorque(const gz::msgs::Wrench &_msg) {
  this->ft_sensor_data_[0] = _msg.force().x();
  this->ft_sensor_data_[1] = _msg.force().y();
  this->ft_sensor_data_[2] = _msg.force().z();
  this->ft_sensor_data_[3] = _msg.torque().x();
  this->ft_sensor_data_[4] = _msg.torque().y();
  this->ft_sensor_data_[5] = _msg.torque().z();
}

class ImuData {
 public:
  std::string name{};
  std::string topicName{};
  sim::Entity sim_imu_sensors_ = sim::kNullEntity;
  std::array<double, 10> imu_sensor_data_;
  void OnIMU(const gz::msgs::IMU &_msg);
};

void ImuData::OnIMU(const gz::msgs::IMU &_msg) {
  this->imu_sensor_data_[0] = _msg.orientation().x();
  this->imu_sensor_data_[1] = _msg.orientation().y();
  this->imu_sensor_data_[2] = _msg.orientation().z();
  this->imu_sensor_data_[3] = _msg.orientation().w();
  this->imu_sensor_data_[4] = _msg.angular_velocity().x();
  this->imu_sensor_data_[5] = _msg.angular_velocity().y();
  this->imu_sensor_data_[6] = _msg.angular_velocity().z();
  this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
  this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
  this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

class odri_gz_ros2_control::GazeboOdriSimSystemPrivate {
 public:
  GazeboOdriSimSystemPrivate() = default;
  ~GazeboOdriSimSystemPrivate() = default;

  rclcpp::Time last_update_sim_time_ros_;
  std::vector<struct jointData> joints_;
  std::vector<std::shared_ptr<ImuData>> imus_;
  std::vector<std::shared_ptr<ForceTorqueData>> ft_sensors_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  sim::EntityComponentManager *ecm;
  unsigned int update_rate;
  gz::transport::Node node;
  bool hold_joints_ = true;
  double position_proportional_gain_ = 0.1;
};

namespace odri_gz_ros2_control {

bool GazeboOdriSimSystem::initSim(
    rclcpp::Node::SharedPtr &model_nh,
    std::map<std::string, sim::Entity> &enableJoints,
    const hardware_interface::HardwareInfo &hardware_info,
    sim::EntityComponentManager &_ecm, unsigned int update_rate) {
  this->dataPtr = std::make_unique<GazeboOdriSimSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->update_rate = update_rate;

  try {
    this->dataPtr->hold_joints_ =
        this->nh_->get_parameter("hold_joints").as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException &ex) {
    RCLCPP_ERROR(this->nh_->get_logger(),
                 "Parameter 'hold_joints' not initialized, with error %s",
                 ex.what());
    RCLCPP_WARN_STREAM(this->nh_->get_logger(),
                       "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::exceptions::ParameterNotDeclaredException &ex) {
    RCLCPP_ERROR(this->nh_->get_logger(),
                 "Parameter 'hold_joints' not declared, with error %s",
                 ex.what());
    RCLCPP_WARN_STREAM(this->nh_->get_logger(),
                       "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::ParameterTypeException &ex) {
    RCLCPP_ERROR(this->nh_->get_logger(),
                 "Parameter 'hold_joints' has wrong type: %s", ex.what());
    RCLCPP_WARN_STREAM(this->nh_->get_logger(),
                       "Using default value: " << this->dataPtr->hold_joints_);
  }
  RCLCPP_DEBUG_STREAM(
      this->nh_->get_logger(),
      "hold_joints (system): " << this->dataPtr->hold_joints_ << std::endl);

  this->dataPtr->joints_.resize(hardware_info.joints.size());

  if (this->dataPtr->joints_.empty()) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    auto &joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    auto it_joint = enableJoints.find(joint_name);
    if (it_joint == enableJoints.end()) {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(),
                         "Skipping joint in the URDF named '"
                             << joint_name
                             << "' which is not in the gazebo model.");
      continue;
    }

    sim::Entity simjoint = it_joint->second;
    this->dataPtr->joints_[j].sim_joint = simjoint;
    this->dataPtr->joints_[j].joint_type =
        _ecm.Component<sim::components::JointType>(simjoint)->Data();
    this->dataPtr->joints_[j].joint_axis =
        _ecm.Component<sim::components::JointAxis>(simjoint)->Data();

    // Create joint position component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
            simjoint, sim::components::JointPosition().TypeId())) {
      _ecm.CreateComponent(simjoint, sim::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
            simjoint, sim::components::JointVelocity().TypeId())) {
      _ecm.CreateComponent(simjoint, sim::components::JointVelocity());
    }

    // Create joint transmitted wrench component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
            simjoint, sim::components::JointTransmittedWrench().TypeId())) {
      _ecm.CreateComponent(simjoint, sim::components::JointTransmittedWrench());
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                       "Loading joint: " << joint_name);

    // Precompute interface name strings to avoid allocations in
    // perform_command_mode_switch
    this->dataPtr->joints_[j].if_name_position =
        joint_name + "/" + hardware_interface::HW_IF_POSITION;
    this->dataPtr->joints_[j].if_name_velocity =
        joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    this->dataPtr->joints_[j].if_name_effort =
        joint_name + "/" + hardware_interface::HW_IF_EFFORT;
    this->dataPtr->joints_[j].if_name_gain_kp =
        joint_name + "/" + ros2_control_odri::HW_IF_GAIN_KP;
    this->dataPtr->joints_[j].if_name_gain_kd =
        joint_name + "/" + ros2_control_odri::HW_IF_GAIN_KD;

    // Log if joint is a mimic joint
    auto it_mimic = std::find_if(hardware_info.mimic_joints.begin(),
                                 hardware_info.mimic_joints.end(),
                                 [j](const hardware_interface::MimicJoint &mj) {
                                   return mj.joint_index == j;
                                 });
    if (it_mimic != hardware_info.mimic_joints.end()) {
      RCLCPP_INFO_STREAM(
          this->nh_->get_logger(),
          "Joint '" << joint_name << "' is mimicking joint '"
                    << hardware_info.joints[it_mimic->mimicked_joint_index].name
                    << "' with multiplier: " << it_mimic->multiplier
                    << " and offset: " << it_mimic->offset);
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value =
        [this,
         joint_name](const hardware_interface::InterfaceInfo &interface_info) {
          double initial_value{0.0};
          if (!interface_info.initial_value.empty()) {
            try {
              initial_value =
                  hardware_interface::stod(interface_info.initial_value);
              RCLCPP_INFO(this->nh_->get_logger(),
                          "\t\t\t found initial value: %f", initial_value);
            } catch (std::invalid_argument &) {
              RCLCPP_ERROR_STREAM(this->nh_->get_logger(),
                                  "Failed converting initial_value string to "
                                  "real number for the joint "
                                      << joint_name << " and state interface "
                                      << interface_info.name
                                      << ". Actual value of parameter: "
                                      << interface_info.initial_value
                                      << ". Initial value will be set to 0.0");
              throw std::invalid_argument(
                  "Failed converting initial_value string");
            }
          }
          return initial_value;
        };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();
    double initial_Kp = std::numeric_limits<double>::quiet_NaN();
    double initial_Kd = std::numeric_limits<double>::quiet_NaN();

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(
            joint_name, hardware_interface::HW_IF_POSITION,
            &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
            joint_name, hardware_interface::HW_IF_VELOCITY,
            &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(
            joint_name, hardware_interface::HW_IF_EFFORT,
            &this->dataPtr->joints_[j].joint_effort);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_effort = initial_effort;
      }
      if (joint_info.state_interfaces[i].name == "gain_kp") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t gain_kp");
        this->dataPtr->state_interfaces_.emplace_back(
            joint_name, ros2_control_odri::HW_IF_GAIN_KP,
            &this->dataPtr->joints_[j].joint_Kp);
        initial_Kp = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_Kp = initial_Kp;
      }
      if (joint_info.state_interfaces[i].name == "gain_kd") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t gain_kd");
        this->dataPtr->state_interfaces_.emplace_back(
            joint_name, ros2_control_odri::HW_IF_GAIN_KD,
            &this->dataPtr->joints_[j].joint_Kd);
        initial_Kd = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_Kd = initial_Kd;
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->command_interfaces_.emplace_back(
            joint_name, hardware_interface::HW_IF_POSITION,
            &this->dataPtr->joints_[j].joint_position_cmd);
        if (!std::isnan(initial_position)) {
          this->dataPtr->joints_[j].joint_position_cmd = initial_position;
        }
      } else if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(
            joint_name, hardware_interface::HW_IF_VELOCITY,
            &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      } else if (joint_info.command_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(
            joint_name, hardware_interface::HW_IF_EFFORT,
            &this->dataPtr->joints_[j].joint_effort_cmd);
        if (!std::isnan(initial_effort)) {
          this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
        }
      } else if (joint_info.command_interfaces[i].name == "gain_kp") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t Gain Kp");
        this->dataPtr->command_interfaces_.emplace_back(
            joint_name, ros2_control_odri::HW_IF_GAIN_KP,
            &this->dataPtr->joints_[j].joint_Kp_cmd);
        if (!std::isnan(initial_Kp)) {
          this->dataPtr->joints_[j].joint_Kp_cmd = initial_Kp;
        }
      } else if (joint_info.command_interfaces[i].name == "gain_kd") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t Gain Kd");
        this->dataPtr->command_interfaces_.emplace_back(
            joint_name, ros2_control_odri::HW_IF_GAIN_KD,
            &this->dataPtr->joints_[j].joint_Kd_cmd);
        if (!std::isnan(initial_Kd)) {
          this->dataPtr->joints_[j].joint_Kd_cmd = initial_Kd;
        }
      }
      // independently of existence of command interface set initial value if
      // defined
      if (!std::isnan(initial_position)) {
        this->dataPtr->joints_[j].joint_position = initial_position;
        this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[j].sim_joint,
            sim::components::JointPositionReset({initial_position}));
      }
      if (!std::isnan(initial_velocity)) {
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
        this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[j].sim_joint,
            sim::components::JointVelocityReset({initial_velocity}));
      }
    }

    this->dataPtr->joints_[j].is_actuated =
        (joint_info.command_interfaces.size() > 0);
  }

  registerSensors(hardware_info);
  return true;
}

void GazeboOdriSimSystem::registerSensors(
    const hardware_interface::HardwareInfo &hardware_info) {
  // Build a name→ComponentInfo lookup map for O(1) per-sensor access
  std::unordered_map<std::string, const hardware_interface::ComponentInfo *>
      sensor_map;
  sensor_map.reserve(hardware_info.sensors.size());
  for (const auto &comp : hardware_info.sensors) {
    sensor_map.emplace(comp.name, &comp);
  }

  RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                     "Number of sensors: " << hardware_info.sensors.size());

  this->dataPtr->ecm->Each<sim::components::Imu, sim::components::Name>(
      [&](const sim::Entity &_entity, const sim::components::Imu *,
          const sim::components::Name *_name) -> bool {
        auto imuData = std::make_shared<ImuData>();
        RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                           "Loading sensor: " << _name->Data());

        auto sensorTopicComp =
            this->dataPtr->ecm->Component<sim::components::SensorTopic>(
                _entity);
        if (sensorTopicComp) {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                             "Topic name: " << sensorTopicComp->Data());
        }

        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");
        imuData->name = _name->Data();
        imuData->sim_imu_sensors_ = _entity;

        auto sensor_it = sensor_map.find(_name->Data());
        if (sensor_it == sensor_map.end()) {
          RCLCPP_WARN_STREAM(this->nh_->get_logger(),
                             "IMU sensor '"
                                 << _name->Data()
                                 << "' not found in hardware_info, skipping.");
          return true;
        }
        const hardware_interface::ComponentInfo &component = *sensor_it->second;

        static const std::map<std::string, size_t> interface_name_map = {
            {"orientation.x", 0},         {"orientation.y", 1},
            {"orientation.z", 2},         {"orientation.w", 3},
            {"angular_velocity.x", 4},    {"angular_velocity.y", 5},
            {"angular_velocity.z", 6},    {"linear_acceleration.x", 7},
            {"linear_acceleration.y", 8}, {"linear_acceleration.z", 9},
        };

        for (const auto &state_interface : component.state_interfaces) {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                             "\t\t " << state_interface.name);
          size_t data_index = interface_name_map.at(state_interface.name);
          this->dataPtr->state_interfaces_.emplace_back(
              imuData->name, state_interface.name,
              &imuData->imu_sensor_data_[data_index]);
        }
        this->dataPtr->imus_.push_back(imuData);
        return true;
      });

  this->dataPtr->ecm->Each<sim::components::ForceTorque, sim::components::Name>(
      [&](const sim::Entity &_entity, const sim::components::ForceTorque *,
          const sim::components::Name *_name) -> bool {
        auto ftData = std::make_shared<ForceTorqueData>();
        RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                           "Loading sensor: " << _name->Data());

        auto sensorTopicComp =
            this->dataPtr->ecm->Component<sim::components::SensorTopic>(
                _entity);
        if (sensorTopicComp) {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                             "Topic name: " << sensorTopicComp->Data());
        }

        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");
        ftData->name = _name->Data();
        ftData->sim_ft_sensors_ = _entity;

        auto sensor_it = sensor_map.find(_name->Data());
        if (sensor_it == sensor_map.end()) {
          RCLCPP_WARN_STREAM(this->nh_->get_logger(),
                             "ForceTorque sensor '"
                                 << _name->Data()
                                 << "' not found in hardware_info, skipping.");
          return true;
        }
        const hardware_interface::ComponentInfo &component = *sensor_it->second;

        static const std::map<std::string, size_t> interface_name_map = {
            {"force.x", 0},  {"force.y", 1},  {"force.z", 2},
            {"torque.x", 3}, {"torque.y", 4}, {"torque.z", 5},
        };

        for (const auto &state_interface : component.state_interfaces) {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(),
                             "\t\t " << state_interface.name);
          size_t data_index = interface_name_map.at(state_interface.name);
          this->dataPtr->state_interfaces_.emplace_back(
              ftData->name, state_interface.name,
              &ftData->ft_sensor_data_[data_index]);
        }
        this->dataPtr->ft_sensors_.push_back(ftData);
        return true;
      });
}

CallbackReturn GazeboOdriSimSystem::on_init(
    const hardware_interface::HardwareInfo &actuator_info) {
  if (hardware_interface::SystemInterface::on_init(actuator_info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (actuator_info.hardware_plugin_name !=
      "odri_gz_ros2_control/GazeboOdriSimSystem") {
    RCLCPP_WARN(this->nh_->get_logger(),
                "The plugin name in <hardware><plugin> should be "
                "'odri_gz_ros2_control/GazeboOdriSimSystem'.");
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboOdriSimSystem::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(this->nh_->get_logger(), "System Successfully configured!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GazeboOdriSimSystem::export_state_interfaces() {
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
GazeboOdriSimSystem::export_command_interfaces() {
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn GazeboOdriSimSystem::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn GazeboOdriSimSystem::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type GazeboOdriSimSystem::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    const auto *jointVelocity =
        this->dataPtr->ecm->Component<sim::components::JointVelocity>(
            this->dataPtr->joints_[i].sim_joint);

    const auto *jointWrench =
        this->dataPtr->ecm->Component<sim::components::JointTransmittedWrench>(
            this->dataPtr->joints_[i].sim_joint);

    const auto *jointPositions =
        this->dataPtr->ecm->Component<sim::components::JointPosition>(
            this->dataPtr->joints_[i].sim_joint);

    if (!jointPositions || !jointVelocity || !jointWrench) {
      continue;
    }

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];

    gz::physics::Vector3d force_or_torque;
    if (this->dataPtr->joints_[i].joint_type == sdf::JointType::PRISMATIC) {
      force_or_torque = {jointWrench->Data().force().x(),
                         jointWrench->Data().force().y(),
                         jointWrench->Data().force().z()};
    } else {
      force_or_torque = {jointWrench->Data().torque().x(),
                         jointWrench->Data().torque().y(),
                         jointWrench->Data().torque().z()};
    }
    this->dataPtr->joints_[i].joint_effort = force_or_torque.dot(
        gz::physics::Vector3d{this->dataPtr->joints_[i].joint_axis.Xyz()[0],
                              this->dataPtr->joints_[i].joint_axis.Xyz()[1],
                              this->dataPtr->joints_[i].joint_axis.Xyz()[2]});
  }

  for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i) {
    if (this->dataPtr->imus_[i]->topicName.empty()) {
      auto sensorTopicComp =
          this->dataPtr->ecm->Component<sim::components::SensorTopic>(
              this->dataPtr->imus_[i]->sim_imu_sensors_);
      if (sensorTopicComp) {
        this->dataPtr->imus_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
            this->nh_->get_logger(),
            "IMU " << this->dataPtr->imus_[i]->name
                   << " has a topic name: " << sensorTopicComp->Data());
        this->dataPtr->node.Subscribe(this->dataPtr->imus_[i]->topicName,
                                      &ImuData::OnIMU,
                                      this->dataPtr->imus_[i].get());
      }
    }
  }

  for (unsigned int i = 0; i < this->dataPtr->ft_sensors_.size(); ++i) {
    if (this->dataPtr->ft_sensors_[i]->topicName.empty()) {
      auto sensorTopicComp =
          this->dataPtr->ecm->Component<sim::components::SensorTopic>(
              this->dataPtr->ft_sensors_[i]->sim_ft_sensors_);
      if (sensorTopicComp) {
        this->dataPtr->ft_sensors_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
            this->nh_->get_logger(),
            "ForceTorque " << this->dataPtr->ft_sensors_[i]->name
                           << " has a topic name: " << sensorTopicComp->Data());
        this->dataPtr->node.Subscribe(this->dataPtr->ft_sensors_[i]->topicName,
                                      &ForceTorqueData::OnForceTorque,
                                      this->dataPtr->ft_sensors_[i].get());
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
GazeboOdriSimSystem::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    for (const std::string &interface_name : stop_interfaces) {
      if (interface_name == this->dataPtr->joints_[j].if_name_position ||
          interface_name == this->dataPtr->joints_[j].if_name_velocity ||
          interface_name == this->dataPtr->joints_[j].if_name_effort ||
          interface_name == this->dataPtr->joints_[j].if_name_gain_kp ||
          interface_name == this->dataPtr->joints_[j].if_name_gain_kd) {
        this->dataPtr->joints_[j].joint_control_method = ControlMethod(NONE);
      }
    }

    for (const std::string &interface_name : start_interfaces) {
      if (interface_name == this->dataPtr->joints_[j].if_name_position) {
        this->dataPtr->joints_[j].joint_control_method |= POSITION;
      } else if (interface_name == this->dataPtr->joints_[j].if_name_velocity) {
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
      } else if (interface_name == this->dataPtr->joints_[j].if_name_effort) {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
      } else if (interface_name == this->dataPtr->joints_[j].if_name_gain_kp ||
                 interface_name == this->dataPtr->joints_[j].if_name_gain_kd) {
        this->dataPtr->joints_[j].joint_control_method =
            ControlMethod(POS_VEL_EFF_GAINS);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboOdriSimSystem::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    if (this->dataPtr->joints_[i].joint_control_method & POS_VEL_EFF_GAINS) {
      // ODRI master board torque law:
      // τ = τ_cmd + Kp * (pos_cmd - pos) + Kd * (vel_cmd - vel)
      double pos_error = this->dataPtr->joints_[i].joint_position_cmd -
                         this->dataPtr->joints_[i].joint_position;
      double vel_error = this->dataPtr->joints_[i].joint_velocity_cmd -
                         this->dataPtr->joints_[i].joint_velocity;
      double torque = this->dataPtr->joints_[i].joint_effort_cmd +
                      this->dataPtr->joints_[i].joint_Kp_cmd * pos_error +
                      this->dataPtr->joints_[i].joint_Kd_cmd * vel_error;
      auto forceCmd =
          this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
              this->dataPtr->joints_[i].sim_joint);
      if (forceCmd == nullptr) {
        this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[i].sim_joint,
            sim::components::JointForceCmd({torque}));
      } else {
        *forceCmd = sim::components::JointForceCmd({torque});
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & VELOCITY) {
      if (!this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint)) {
        this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[i].sim_joint,
            sim::components::JointVelocityCmd({0}));
      } else {
        const auto jointVelCmd =
            this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
                this->dataPtr->joints_[i].sim_joint);
        *jointVelCmd = sim::components::JointVelocityCmd(
            {this->dataPtr->joints_[i].joint_velocity_cmd});
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & POSITION) {
      double error = (this->dataPtr->joints_[i].joint_position -
                      this->dataPtr->joints_[i].joint_position_cmd) *
                     this->dataPtr->update_rate;

      double target_vel = -this->dataPtr->position_proportional_gain_ * error;

      auto vel =
          this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr) {
        this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[i].sim_joint,
            sim::components::JointVelocityCmd({target_vel}));
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = target_vel;
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) {
      if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
              this->dataPtr->joints_[i].sim_joint)) {
        this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[i].sim_joint,
            sim::components::JointForceCmd({0}));
      } else {
        const auto jointEffortCmd =
            this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
                this->dataPtr->joints_[i].sim_joint);
        *jointEffortCmd = sim::components::JointForceCmd(
            {this->dataPtr->joints_[i].joint_effort_cmd});
      }
    } else if (this->dataPtr->joints_[i].is_actuated &&
               this->dataPtr->hold_joints_) {
      double target_vel = 0.0;
      auto vel =
          this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr) {
        this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[i].sim_joint,
            sim::components::JointVelocityCmd({target_vel}));
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = target_vel;
      }
    }
  }

  // set values of all mimic joints with respect to mimicked joint
  for (const auto &mimic_joint : this->info_.mimic_joints) {
    double position_mimicked_joint =
        this->dataPtr->ecm
            ->Component<sim::components::JointPosition>(
                this->dataPtr->joints_[mimic_joint.mimicked_joint_index]
                    .sim_joint)
            ->Data()[0];

    double position_mimic_joint =
        this->dataPtr->ecm
            ->Component<sim::components::JointPosition>(
                this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)
            ->Data()[0];

    double position_error =
        position_mimic_joint - position_mimicked_joint * mimic_joint.multiplier;

    double velocity_sp = (-1.0) * position_error * this->dataPtr->update_rate;

    auto vel = this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
        this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);

    if (vel == nullptr) {
      this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
          sim::components::JointVelocityCmd({velocity_sp}));
    } else if (!vel->Data().empty()) {
      vel->Data()[0] = velocity_sp;
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace odri_gz_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(odri_gz_ros2_control::GazeboOdriSimSystem,
                       odri_gz_ros2_control::GazeboOdriSimSystemInterface)
