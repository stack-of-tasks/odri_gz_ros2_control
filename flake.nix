{
  description = "Gazebo Plugin to simulate the ODRI interface (add the Kp Kd gains to the hardware interface";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosDistros = [ "jazzy" ];
        rosShellDistro = "jazzy";
        rosOverrideAttrs.odri-gz-ros2-control = {
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
        };
      }
    );
}
