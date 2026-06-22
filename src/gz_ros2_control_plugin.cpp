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

#include "odri_gz_ros2_control/gz_ros2_control_plugin.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <controller_manager/controller_manager.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "odri_gz_ros2_control/gz_system.hpp"

namespace odri_gz_ros2_control {
class GZResourceManager : public hardware_interface::ResourceManager {
 public:
  GZResourceManager(rclcpp::Node::SharedPtr &node,
                    sim::EntityComponentManager &ecm,
                    std::map<std::string, sim::Entity> enabledJoints)
      : hardware_interface::ResourceManager(node->get_node_clock_interface(),
                                            node->get_node_logging_interface()),
        gz_system_loader_("odri_gz_ros2_control",
                          "odri_gz_ros2_control::GazeboOdriSimSystemInterface"),
        logger_(node->get_logger().get_child("GZResourceManager")) {
    node_ = node;
    ecm_ = &ecm;
    enabledJoints_ = enabledJoints;
  }

  GZResourceManager(const GZResourceManager &) = delete;

  // Called from Controller Manager when robot description is initialized from
  // callback
  bool load_and_initialize_components(const std::string &urdf,
                                      unsigned int update_rate) override {
    components_are_loaded_and_initialized_ = true;

    const auto hardware_info =
        hardware_interface::parse_control_resources_from_urdf(urdf);

    for (const auto &individual_hardware_info : hardware_info) {
      std::string robot_hw_sim_type_str_ =
          individual_hardware_info.hardware_plugin_name;
      RCLCPP_DEBUG(logger_, "Load hardware interface %s ...",
                   robot_hw_sim_type_str_.c_str());

      std::unique_ptr<odri_gz_ros2_control::GazeboOdriSimSystemInterface>
          gzSimSystem;
      std::scoped_lock guard(resource_interfaces_lock_,
                             claimed_command_interfaces_lock_);
      try {
        gzSimSystem =
            std::unique_ptr<odri_gz_ros2_control::GazeboOdriSimSystemInterface>(
                gz_system_loader_.createUnmanagedInstance(
                    robot_hw_sim_type_str_));
      } catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(logger_,
                     "The plugin failed to load for some reason. Error: %s\n",
                     ex.what());
        continue;
      }

      if (!gzSimSystem->initSim(node_, enabledJoints_, individual_hardware_info,
                                *ecm_, update_rate)) {
        RCLCPP_FATAL(logger_,
                     "Could not initialize robot simulation interface");
        components_are_loaded_and_initialized_ = false;
        break;
      }
      RCLCPP_DEBUG(logger_, "Initialized robot simulation interface %s!",
                   robot_hw_sim_type_str_.c_str());

      import_component(std::move(gzSimSystem), individual_hardware_info);
    }

    return components_are_loaded_and_initialized_;
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  sim::EntityComponentManager *ecm_;
  std::map<std::string, sim::Entity> enabledJoints_;

  pluginlib::ClassLoader<odri_gz_ros2_control::GazeboOdriSimSystemInterface>
      gz_system_loader_;

  rclcpp::Logger logger_;
};

//////////////////////////////////////////////////
class GazeboOdriSimROS2ControlPluginPrivate {
 public:
  std::map<std::string, sim::Entity> GetEnabledJoints(
      const sim::Entity &_entity, sim::EntityComponentManager &_ecm) const;

  sim::Entity entity_;

  std::shared_ptr<rclcpp::Node> node_{nullptr};

  std::thread thread_executor_spin_;

  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_{
      nullptr};

  rclcpp::Time last_update_sim_time_ros_ =
      rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME);

  sim::EntityComponentManager *ecm{nullptr};

  int update_rate;

  bool period_mismatch_warned_{false};
};

//////////////////////////////////////////////////
std::map<std::string, sim::Entity>
GazeboOdriSimROS2ControlPluginPrivate::GetEnabledJoints(
    const sim::Entity &_entity, sim::EntityComponentManager &_ecm) const {
  std::map<std::string, sim::Entity> output;

  auto jointEntities =
      _ecm.ChildrenByComponents(_entity, sim::components::Joint());

  for (const auto &jointEntity : jointEntities) {
    const auto jointName =
        _ecm.Component<sim::components::Name>(jointEntity)->Data();

    const auto *jointType =
        _ecm.Component<sim::components::JointType>(jointEntity);
    switch (jointType->Data()) {
      case sdf::JointType::PRISMATIC:
      case sdf::JointType::REVOLUTE:
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::GEARBOX: {
        break;
      }
      case sdf::JointType::FIXED: {
        RCLCPP_INFO(node_->get_logger(),
                    "[odri_gz_ros2_control] Fixed joint ['%s'] (Entity='%lu') "
                    "is skipped.",
                    jointName.c_str(), jointEntity);
        continue;
      }
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::BALL:
      case sdf::JointType::UNIVERSAL: {
        RCLCPP_WARN(node_->get_logger(),
                    "[odri_gz_ros2_control] Joint ['%s'] (Entity='%lu') is of "
                    "unsupported type."
                    " Only joints with a single axis are supported.",
                    jointName.c_str(), jointEntity);
        continue;
      }
      default: {
        RCLCPP_WARN(node_->get_logger(),
                    "[odri_gz_ros2_control] Joint ['%s'] (Entity='%lu') is of "
                    "unknown type.",
                    jointName.c_str(), jointEntity);
        continue;
      }
    }
    output[jointName] = jointEntity;
  }

  return output;
}

//////////////////////////////////////////////////
GazeboOdriSimROS2ControlPlugin::GazeboOdriSimROS2ControlPlugin()
    : dataPtr(std::make_unique<GazeboOdriSimROS2ControlPluginPrivate>()) {}

//////////////////////////////////////////////////
GazeboOdriSimROS2ControlPlugin::~GazeboOdriSimROS2ControlPlugin() {
  this->dataPtr->executor_->cancel();
  this->dataPtr->thread_executor_spin_.join();
}

//////////////////////////////////////////////////
void GazeboOdriSimROS2ControlPlugin::Configure(
    const sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm, sim::EventManager &) {
  rclcpp::Logger logger = rclcpp::get_logger("GazeboOdriSimROS2ControlPlugin");

  const auto model = sim::Model(_entity);
  if (!model.Valid(_ecm)) {
    RCLCPP_ERROR(logger,
                 "[Gazebo ROS 2 Control] Failed to initialize because [%s] "
                 "(Entity=%lu)] is not a model."
                 "Please make sure that Gazebo ROS 2 Control is attached to a "
                 "valid model.",
                 model.Name(_ecm).c_str(), _entity);
    return;
  }

  std::string paramFileName = _sdf->Get<std::string>("parameters");
  if (paramFileName.empty()) {
    RCLCPP_ERROR(logger,
                 "Gazebo ros2 control found an empty parameters file. Failed "
                 "to initialize.");
    return;
  }

  std::vector<std::string> arguments = {"--ros-args"};

  auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

  sdf::ElementPtr argument_sdf_param = sdfPtr->GetElement("parameters");
  while (argument_sdf_param) {
    std::string argument = argument_sdf_param->Get<std::string>();
    arguments.push_back(RCL_PARAM_FILE_FLAG);
    arguments.push_back(argument);
    argument_sdf_param = argument_sdf_param->GetNextElement("parameters");
  }

  std::string controllerManagerNodeName{"controller_manager"};
  if (sdfPtr->HasElement("controller_manager_name")) {
    controllerManagerNodeName =
        sdfPtr->GetElement("controller_manager_name")->Get<std::string>();
  }

  std::string ns = "/";

  bool hold_joints = true;
  if (sdfPtr->HasElement("hold_joints")) {
    hold_joints = sdfPtr->GetElement("hold_joints")->Get<bool>();
  }

  double position_proportional_gain = 0.1;
  if (sdfPtr->HasElement("position_proportional_gain")) {
    position_proportional_gain =
        sdfPtr->GetElement("position_proportional_gain")->Get<double>();
  }

  if (sdfPtr->HasElement("ros")) {
    sdf::ElementPtr sdfRos = sdfPtr->GetElement("ros");

    if (sdfRos->HasElement("namespace")) {
      ns = sdfRos->GetElement("namespace")->Get<std::string>();
      if (ns.empty() || ns[0] != '/') {
        ns = '/' + ns;
      }
    }

    if (sdfRos->HasElement("remapping")) {
      sdf::ElementPtr argument_sdf_remapping = sdfRos->GetElement("remapping");
      arguments.push_back(RCL_ROS_ARGS_FLAG);
      while (argument_sdf_remapping) {
        std::string argument = argument_sdf_remapping->Get<std::string>();
        arguments.push_back(RCL_REMAP_FLAG);
        arguments.push_back(argument);
        argument_sdf_remapping =
            argument_sdf_remapping->GetNextElement("remapping");
      }
    }
  }

  std::vector<const char *> argv;
  for (const auto &arg : arguments) {
    argv.push_back(reinterpret_cast<const char *>(arg.data()));
  }

  if (!rclcpp::ok()) {
    rclcpp::init(static_cast<int>(argv.size()), argv.data(),
                 rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  }

  this->dataPtr->node_ = rclcpp::Node::make_shared("odri_gz_ros2_control", ns);
  this->dataPtr->executor_ =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->dataPtr->executor_->add_node(this->dataPtr->node_);
  auto spin = [this]() { this->dataPtr->executor_->spin(); };
  this->dataPtr->thread_executor_spin_ = std::thread(spin);

  RCLCPP_DEBUG_STREAM(this->dataPtr->node_->get_logger(),
                      "[Gazebo Sim ROS 2 Control] Setting up controller for ["
                          << model.Name(_ecm) << "] (Entity=" << _entity
                          << ")].");

  auto enabledJoints = this->dataPtr->GetEnabledJoints(_entity, _ecm);

  if (enabledJoints.empty()) {
    RCLCPP_DEBUG_STREAM(
        this->dataPtr->node_->get_logger(),
        "[Gazebo ROS 2 Control] There are no available Joints.");
    return;
  }

  try {
    this->dataPtr->node_->declare_parameter(
        "hold_joints", rclcpp::ParameterValue(hold_joints));
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e) {
    RCLCPP_ERROR(this->dataPtr->node_->get_logger(),
                 "Parameter 'hold_joints' has already been declared, %s",
                 e.what());
  } catch (const rclcpp::exceptions::InvalidParametersException &e) {
    RCLCPP_ERROR(this->dataPtr->node_->get_logger(),
                 "Parameter 'hold_joints' has invalid name, %s", e.what());
  } catch (const rclcpp::exceptions::InvalidParameterValueException &e) {
    RCLCPP_ERROR(this->dataPtr->node_->get_logger(),
                 "Parameter 'hold_joints' value is invalid, %s", e.what());
  } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
    RCLCPP_ERROR(this->dataPtr->node_->get_logger(),
                 "Parameter 'hold_joints' value has wrong type, %s", e.what());
  }

  try {
    this->dataPtr->node_->declare_parameter(
        "position_proportional_gain",
        rclcpp::ParameterValue(position_proportional_gain));
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &e) {
    RCLCPP_ERROR(
        this->dataPtr->node_->get_logger(),
        "Parameter 'position_proportional_gain' has already been declared, %s",
        e.what());
  } catch (const rclcpp::exceptions::InvalidParametersException &e) {
    RCLCPP_ERROR(this->dataPtr->node_->get_logger(),
                 "Parameter 'position_proportional_gain' has invalid name, %s",
                 e.what());
  } catch (const rclcpp::exceptions::InvalidParameterValueException &e) {
    RCLCPP_ERROR(this->dataPtr->node_->get_logger(),
                 "Parameter 'position_proportional_gain' value is invalid, %s",
                 e.what());
  } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
    RCLCPP_ERROR(
        this->dataPtr->node_->get_logger(),
        "Parameter 'position_proportional_gain' value has wrong type, %s",
        e.what());
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
      std::make_unique<odri_gz_ros2_control::GZResourceManager>(
          this->dataPtr->node_, _ecm, enabledJoints);

  RCLCPP_INFO(this->dataPtr->node_->get_logger(), "Loading controller_manager");
  rclcpp::NodeOptions options = controller_manager::get_cm_node_options();
  arguments.push_back("-r");
  arguments.push_back("__node:=" + controllerManagerNodeName);
  arguments.push_back("-r");
  arguments.push_back("__ns:=" + ns);
  arguments.push_back("-p");
  arguments.push_back("use_sim_time:=true");
  options.arguments(arguments);
  this->dataPtr->controller_manager_.reset(
      new controller_manager::ControllerManager(
          std::move(resource_manager_), this->dataPtr->executor_,
          controllerManagerNodeName, this->dataPtr->node_->get_namespace(),
          options));
  this->dataPtr->executor_->add_node(this->dataPtr->controller_manager_);

  this->dataPtr->update_rate =
      static_cast<int>(this->dataPtr->controller_manager_->get_update_rate());
  this->dataPtr->control_period_ =
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(
              1.0 / static_cast<double>(this->dataPtr->update_rate))));

  // Wait for CM to receive robot description and initialize Resource Manager
  while (
      !this->dataPtr->controller_manager_->is_resource_manager_initialized()) {
    RCLCPP_WARN(this->dataPtr->node_->get_logger(),
                "Waiting RM to load and initialize hardware...");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  this->dataPtr->entity_ = _entity;
}

//////////////////////////////////////////////////
void GazeboOdriSimROS2ControlPlugin::PreUpdate(
    const sim::UpdateInfo &_info, sim::EntityComponentManager & /*_ecm*/) {
  if (!this->dataPtr->controller_manager_) {
    return;
  }

  if (!this->dataPtr->period_mismatch_warned_) {
    rclcpp::Duration gazebo_period(_info.dt);

    if (this->dataPtr->control_period_ < gazebo_period) {
      RCLCPP_ERROR_STREAM(
          this->dataPtr->node_->get_logger(),
          "Desired controller update period ("
              << this->dataPtr->control_period_.seconds()
              << " s) is faster than the gazebo simulation period ("
              << gazebo_period.seconds() << " s).");
    } else if (this->dataPtr->control_period_ > gazebo_period) {
      RCLCPP_WARN_STREAM(
          this->dataPtr->node_->get_logger(),
          " Desired controller update period ("
              << this->dataPtr->control_period_.seconds()
              << " s) is slower than the gazebo simulation period ("
              << gazebo_period.seconds() << " s).");
    }
    this->dataPtr->period_mismatch_warned_ = true;
  }

  rclcpp::Time sim_time_ros(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime)
          .count(),
      RCL_ROS_TIME);
  rclcpp::Duration sim_period =
      sim_time_ros - this->dataPtr->last_update_sim_time_ros_;
  this->dataPtr->controller_manager_->write(sim_time_ros, sim_period);
}

//////////////////////////////////////////////////
void GazeboOdriSimROS2ControlPlugin::PostUpdate(
    const sim::UpdateInfo &_info,
    const sim::EntityComponentManager & /*_ecm*/) {
  if (!this->dataPtr->controller_manager_) {
    return;
  }

  rclcpp::Time sim_time_ros(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime)
          .count(),
      RCL_ROS_TIME);
  rclcpp::Duration sim_period =
      sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

  if (sim_period >= this->dataPtr->control_period_) {
    this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
    this->dataPtr->controller_manager_->read(sim_time_ros, sim_period);
    this->dataPtr->controller_manager_->update(sim_time_ros, sim_period);
  }
}
}  // namespace odri_gz_ros2_control

GZ_ADD_PLUGIN(
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin, gz::sim::System,
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin::ISystemConfigure,
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin::ISystemPreUpdate,
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin,
                    "ign_ros2_control::IgnitionROS2ControlPlugin")
