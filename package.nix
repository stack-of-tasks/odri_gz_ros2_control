{
  lib,
  buildRosPackage,

  ament-cmake,

  ament-index-cpp,
  gz-sim,
  gz-plugin,
  pluginlib,
  rclcpp,
  yaml-cpp-vendor,
  rclcpp-lifecycle,
  hardware-interface,
  controller-manager,

  ament-cmake-cppcheck,
  ament-cmake-cpplint,
  ament-cmake-copyright,
  ament-cmake-lint-cmake,
  ament-cmake-xmllint,
  ament-cpplint,
  ament-cppcheck,
  ament-copyright,
  ament-lint-auto,
  ament-lint-cmake,
  ament-lint-common,
  ament-xmllint,
}:
buildRosPackage {
  pname = "odri-gz-ros2-control";
  version = "1.2.2";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./gz_hardware_odri_plugins.xml
      ./include
      ./package.xml
      ./src
    ];
  };

  buildType = "ament_cmake";
  __structuredAttrs = true;
  strictDeps = true;

  nativeBuildInputs = [
    ament-cmake
  ];
  buildInputs = [
    ament-cmake
  ];

  propagatedBuildInputs = [
    # keep-sorted start
    ament-index-cpp
    controller-manager
    gz-plugin
    gz-sim
    hardware-interface
    pluginlib
    rclcpp
    rclcpp-lifecycle
    yaml-cpp-vendor
    # keep-sorted end
  ];
  checkInputs = [
    # keep-sorted start
    ament-cmake-copyright
    ament-cmake-cppcheck
    ament-cmake-cpplint
    ament-cmake-lint-cmake
    ament-cmake-xmllint
    ament-lint-auto
    ament-lint-common
    # keep-sorted end
  ];
  nativeCheckInputs = [
    # keep-sorted start
    ament-copyright
    ament-cppcheck
    ament-cpplint
    ament-lint-cmake
    ament-xmllint
    # keep-sorted end
  ];

  doCheck = true;

  meta = {
    description = "{{ pkg.description }}";
    license = with lib.licenses; [ asl20 ];
    homepage = "https://github.com/stack-of-tasks/odri_gz_ros2_control";
    platforms = lib.platforms.linux;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
