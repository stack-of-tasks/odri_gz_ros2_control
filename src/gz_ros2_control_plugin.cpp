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

#include <unistd.h>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef GZ_SIM_8
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#else
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/plugin/Register.hh>
#endif

#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gz_ros2_control/gz_ros2_control_plugin.hpp"
#include "gz_ros2_control/gz_system.hpp"

namespace odri_gz_ros2_control {
class GZResourceManager : public hardware_interface::ResourceManager {
 public:
  GZResourceManager(rclcpp::Node::SharedPtr &node,
                    sim::EntityComponentManager &ecm,
                    std::map<std::string, sim::Entity> enabledJoints)
      : hardware_interface::ResourceManager(node->get_node_clock_interface(),
                                            node->get_node_logging_interface()),
        gz_system_loader_("gz_ros2_control",
                          "gz_ros2_control::GazeboOdriSimSystemInterface"),
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

      // Load hardware
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

      // initialize simulation requirements
      if (!gzSimSystem->initSim(node_, enabledJoints_, individual_hardware_info,
                                *ecm_, update_rate)) {
        RCLCPP_FATAL(logger_,
                     "Could not initialize robot simulation interface");
        components_are_loaded_and_initialized_ = false;
        break;
      }
      RCLCPP_DEBUG(logger_, "Initialized robot simulation interface %s!",
                   robot_hw_sim_type_str_.c_str());

      // initialize hardware
      import_component(std::move(gzSimSystem), individual_hardware_info);
    }

    return components_are_loaded_and_initialized_;
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  sim::EntityComponentManager *ecm_;
  std::map<std::string, sim::Entity> enabledJoints_;

  /// \brief Interface loader
  pluginlib::ClassLoader<odri_gz_ros2_control::GazeboOdriSimSystemInterface>
      gz_system_loader_;

  rclcpp::Logger logger_;
};

//////////////////////////////////////////////////
class GazeboOdriSimROS2ControlPluginPrivate {
 public:
  /// \brief Get the URDF XML from the parameter server
  std::string getURDF() const;

  /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
  /// joint names are specified in the plugin configuration, all valid 1-axis
  /// joints are returned
  /// \param[in] _entity Entity of the model that the plugin is being
  /// configured for
  /// \param[in] _ecm Gazebo Entity Component Manager
  /// \return List of entities containing all enabled joints
  std::map<std::string, sim::Entity> GetEnabledJoints(
      const sim::Entity &_entity, sim::EntityComponentManager &_ecm) const;

  /// \brief Entity ID for sensor within Gazebo.
  sim::Entity entity_;

  /// \brief Node Handles
  std::shared_ptr<rclcpp::Node> node_{nullptr};

  /// \brief Thread where the executor will spin
  std::thread thread_executor_spin_;

  /// \brief Flag to stop the executor thread when this plugin is exiting
  bool stop_{false};

  /// \brief Executor to spin the controller
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  /// \brief Timing
  rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

  /// \brief Interface loader
  std::shared_ptr<pluginlib::ClassLoader<
      odri_gz_ros2_control::GazeboOdriSimSystemInterface>>
      robot_hw_sim_loader_{nullptr};

  /// \brief Controller manager
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_{
      nullptr};

  /// \brief String with the robot description param_name
  // TODO(ahcorde): Add param in plugin tag
  std::string robot_description_ = "robot_description";

  /// \brief String with the name of the node that contains the
  /// robot_description
  // TODO(ahcorde): Add param in plugin tag
  std::string robot_description_node_ = "robot_state_publisher";

  /// \brief Last time the update method was called
  rclcpp::Time last_update_sim_time_ros_ =
      rclcpp::Time((int64_t)0, RCL_ROS_TIME);

  /// \brief ECM pointer
  sim::EntityComponentManager *ecm{nullptr};

  /// \brief controller update rate
  int update_rate;
};

//////////////////////////////////////////////////
std::map<std::string, sim::Entity>
GazeboOdriSimROS2ControlPluginPrivate::GetEnabledJoints(
    const sim::Entity &_entity, sim::EntityComponentManager &_ecm) const {
  std::map<std::string, sim::Entity> output;

  std::vector<std::string> enabledJoints;

  // Get all available joints
  auto jointEntities =
      _ecm.ChildrenByComponents(_entity, sim::components::Joint());

  // Iterate over all joints and verify whether they can be enabled or not
  for (const auto &jointEntity : jointEntities) {
    const auto jointName =
        _ecm.Component<sim::components::Name>(jointEntity)->Data();

    // Make sure the joint type is supported, i.e. it has exactly one
    // actuated axis
    const auto *jointType =
        _ecm.Component<sim::components::JointType>(jointEntity);
    switch (jointType->Data()) {
      case sdf::JointType::PRISMATIC:
      case sdf::JointType::REVOLUTE:
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::GEARBOX: {
        // Supported joint type
        break;
      }
      case sdf::JointType::FIXED: {
        RCLCPP_INFO(
            node_->get_logger(),
            "[odri_gz_ros2_control] Fixed joint [%s] (Entity=%lu)] is skipped",
            jointName.c_str(), jointEntity);
        continue;
      }
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::BALL:
      case sdf::JointType::UNIVERSAL: {
        RCLCPP_WARN(node_->get_logger(),
                    "[odri_gz_ros2_control] Joint [%s] (Entity=%lu)] is of "
                    "unsupported type."
                    " Only joints with a single axis are supported.",
                    jointName.c_str(), jointEntity);
        continue;
      }
      default: {
        RCLCPP_WARN(node_->get_logger(),
                    "[odri_gz_ros2_control] Joint [%s] (Entity=%lu)] is of "
                    "unknown type",
                    jointName.c_str(), jointEntity);
        continue;
      }
    }
    output[jointName] = jointEntity;
  }

  return output;
}

//////////////////////////////////////////////////
std::string GazeboOdriSimROS2ControlPluginPrivate::getURDF() const {
  std::string urdf_string;

  using namespace std::chrono_literals;
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, robot_description_node_);
  while (!parameters_client->wait_for_service(0.5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Interrupted while waiting for %s service. Exiting.",
                   robot_description_node_.c_str());
      return 0;
    }
    RCLCPP_ERROR(node_->get_logger(),
                 "%s service not available, waiting again...",
                 robot_description_node_.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "connected to service!! %s asking for %s",
              robot_description_node_.c_str(),
              this->robot_description_.c_str());

  // search and wait for robot_description on param server
  while (urdf_string.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "param_name %s",
                 this->robot_description_.c_str());

    try {
      auto f = parameters_client->get_parameters({this->robot_description_});
      f.wait();
      std::vector<rclcpp::Parameter> values = f.get();
      urdf_string = values[0].as_string();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    }

    if (!urdf_string.empty()) {
      break;
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "odri_gz_ros2_control plugin is waiting for model"
                   " URDF in parameter [%s] on the ROS param server.",
                   this->robot_description_.c_str());
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  RCLCPP_INFO(node_->get_logger(), "Received URDF from param server");

  return urdf_string;
}

//////////////////////////////////////////////////
GazeboOdriSimROS2ControlPlugin::GazeboOdriSimROS2ControlPlugin()
    : dataPtr(std::make_unique<GazeboOdriSimROS2ControlPluginPrivate>()) {}

//////////////////////////////////////////////////
GazeboOdriSimROS2ControlPlugin::~GazeboOdriSimROS2ControlPlugin() {
  // Stop controller manager thread
  this->dataPtr->stop_ = true;
  this->dataPtr->executor_->remove_node(this->dataPtr->controller_manager_);
  this->dataPtr->executor_->cancel();
  this->dataPtr->thread_executor_spin_.join();
}

//////////////////////////////////////////////////
void GazeboOdriSimROS2ControlPlugin::Configure(
    const sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm, sim::EventManager &) {
  rclcpp::Logger logger = rclcpp::get_logger("GazeboOdriSimROS2ControlPlugin");
  // Make sure the controller is attached to a valid model
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

  // Get params from SDF
  std::string paramFileName = _sdf->Get<std::string>("parameters");

  if (paramFileName.empty()) {
    RCLCPP_ERROR(logger,
                 "Gazebo ros2 control found an empty parameters file. Failed "
                 "to initialize.");
    return;
  }

  std::vector<std::string> arguments = {"--ros-args"};

  auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

  sdf::ElementPtr argument_sdf = sdfPtr->GetElement("parameters");
  while (argument_sdf) {
    std::string argument = argument_sdf->Get<std::string>();
    arguments.push_back(RCL_PARAM_FILE_FLAG);
    arguments.push_back(argument);
    argument_sdf = argument_sdf->GetNextElement("parameters");
  }

  // Get controller manager node name
  std::string controllerManagerNodeName{"controller_manager"};

  if (sdfPtr->HasElement("controller_manager_name")) {
    controllerManagerNodeName =
        sdfPtr->GetElement("controller_manager_name")->Get<std::string>();
  }

  std::string ns = "/";

  // Hold joints if no control mode is active?
  bool hold_joints = true;  // default
  if (sdfPtr->HasElement("hold_joints")) {
    hold_joints = sdfPtr->GetElement("hold_joints")->Get<bool>();
  }

  if (sdfPtr->HasElement("ros")) {
    sdf::ElementPtr sdfRos = sdfPtr->GetElement("ros");

    // Set namespace if tag is present
    if (sdfRos->HasElement("namespace")) {
      ns = sdfRos->GetElement("namespace")->Get<std::string>();
      // prevent exception: namespace must be absolute, it must lead with a '/'
      if (ns.empty() || ns[0] != '/') {
        ns = '/' + ns;
      }

      if (ns.length() > 1) {
        this->dataPtr->robot_description_node_ =
            ns + "/" + this->dataPtr->robot_description_node_;
      }
    }

    // Get list of remapping rules from SDF
    if (sdfRos->HasElement("remapping")) {
      sdf::ElementPtr argument_sdf = sdfRos->GetElement("remapping");

      arguments.push_back(RCL_ROS_ARGS_FLAG);
      while (argument_sdf) {
        std::string argument = argument_sdf->Get<std::string>();
        arguments.push_back(RCL_REMAP_FLAG);
        arguments.push_back(argument);
        argument_sdf = argument_sdf->GetNextElement("remapping");
      }
    }
  }

  std::vector<const char *> argv;
  for (const auto &arg : arguments) {
    argv.push_back(reinterpret_cast<const char *>(arg.data()));
  }
  // Create a default context, if not already
  if (!rclcpp::ok()) {
    rclcpp::init(static_cast<int>(argv.size()), argv.data(),
                 rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  }

  std::string node_name = "odri_gz_ros2_control";

  this->dataPtr->node_ = rclcpp::Node::make_shared(node_name, ns);
  this->dataPtr->executor_ =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->dataPtr->executor_->add_node(this->dataPtr->node_);
  this->dataPtr->stop_ = false;
  auto spin = [this]() {
    while (rclcpp::ok() && !this->dataPtr->stop_) {
      this->dataPtr->executor_->spin_once();
    }
  };
  this->dataPtr->thread_executor_spin_ = std::thread(spin);

  RCLCPP_DEBUG_STREAM(this->dataPtr->node_->get_logger(),
                      "[Gazebo Sim ROS 2 Control] Setting up controller for ["
                          << model.Name(_ecm) << "] (Entity=" << _entity
                          << ")].");

  // Get list of enabled joints
  auto enabledJoints = this->dataPtr->GetEnabledJoints(_entity, _ecm);

  if (enabledJoints.size() == 0) {
    RCLCPP_DEBUG_STREAM(
        this->dataPtr->node_->get_logger(),
        "[Gazebo ROS 2 Control] There are no available Joints.");
    return;
  }

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  std::string urdf_string;
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try {
    urdf_string = this->dataPtr->getURDF();
    control_hardware_info =
        hardware_interface::parse_control_resources_from_urdf(urdf_string);
  } catch (const std::runtime_error &ex) {
    RCLCPP_ERROR_STREAM(
        this->dataPtr->node_->get_logger(),
        "Error parsing URDF in odri_gz_ros2_control plugin, plugin not active "
        ": " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
      std::make_unique<odri_gz_ros2_control::GZResourceManager>(
          this->dataPtr->node_, _ecm, enabledJoints);

  // Create the controller manager
  RCLCPP_INFO(this->dataPtr->node_->get_logger(), "Loading controller_manager");
  this->dataPtr->controller_manager_.reset(
      new controller_manager::ControllerManager(
          std::move(resource_manager_), this->dataPtr->executor_,
          controllerManagerNodeName, this->dataPtr->node_->get_namespace()));
  this->dataPtr->executor_->add_node(this->dataPtr->controller_manager_);

  this->dataPtr->update_rate =
      this->dataPtr->controller_manager_->get_update_rate();
  this->dataPtr->control_period_ =
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(
              1.0 / static_cast<double>(this->dataPtr->update_rate))));

  // Force setting of use_sim_time parameter
  this->dataPtr->controller_manager_->set_parameter(
      rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  // Wait for CM to receive robot description from the topic and then initialize
  // Resource Manager
  while (
      !this->dataPtr->controller_manager_->is_resource_manager_initialized()) {
    RCLCPP_WARN(this->dataPtr->node_->get_logger(),
                "Waiting RM to load and initialize hardware...");
    std::this_thread::sleep_for(std::chrono::microseconds(2000000));
  }

  this->dataPtr->entity_ = _entity;
}

//////////////////////////////////////////////////
void GazeboOdriSimROS2ControlPlugin::PreUpdate(
    const sim::UpdateInfo &_info, sim::EntityComponentManager & /*_ecm*/) {
  if (!this->dataPtr->controller_manager_) {
    return;
  }

  static bool warned{false};
  if (!warned) {
    rclcpp::Duration gazebo_period(_info.dt);

    // Check the period against the simulation period
    if (this->dataPtr->control_period_ < _info.dt) {
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
    warned = true;
  }

  rclcpp::Time sim_time_ros(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime)
          .count(),
      RCL_ROS_TIME);
  rclcpp::Duration sim_period =
      sim_time_ros - this->dataPtr->last_update_sim_time_ros_;
  // Always set commands on joints, otherwise at low control frequencies the
  // joints tremble as they are updated at a fraction of gazebo sim time
  this->dataPtr->controller_manager_->write(sim_time_ros, sim_period);
}

//////////////////////////////////////////////////
void GazeboOdriSimROS2ControlPlugin::PostUpdate(
    const sim::UpdateInfo &_info,
    const sim::EntityComponentManager & /*_ecm*/) {
  // Get the simulation time and period
  rclcpp::Time sim_time_ros(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime)
          .count(),
      RCL_ROS_TIME);
  rclcpp::Duration sim_period =
      sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

  if (sim_period >= this->dataPtr->control_period_) {
    this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
    auto gz_controller_manager = std::dynamic_pointer_cast<
        odri_gz_ros2_control::GazeboOdriSimSystemInterface>(
        this->dataPtr->controller_manager_);
    this->dataPtr->controller_manager_->read(sim_time_ros, sim_period);
    this->dataPtr->controller_manager_->update(sim_time_ros, sim_period);
  }
}
}  // namespace odri_gz_ros2_control

#ifdef GZ_SIM_8
GZ_ADD_PLUGIN(
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin, gz::sim::System,
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin::ISystemConfigure,
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin::ISystemPreUpdate,
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin,
                    "ign_ros2_control::IgnitionROS2ControlPlugin")
#else
IGNITION_ADD_PLUGIN(
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin,
    ignition::gazebo::System,
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin::ISystemConfigure,
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin::ISystemPreUpdate,
    odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(odri_gz_ros2_control::GazeboOdriSimROS2ControlPlugin,
                          "ign_ros2_control::IgnitionROS2ControlPlugin")
#endif
