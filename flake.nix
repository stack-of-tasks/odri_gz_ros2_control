{
  description = "Gazebo Plugin to simulate the ODRI interface (add the Kp Kd gains to the hardware interface";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { ... }:
      {
        rosDistros = [ "jazzy" ];
        rosShellDistro = "jazzy";
        rosPackages.odri-gz-ros2-control = ./package.nix;
      }
    );
}
