# Existing-machine setup. Targets only the already-installed Ubuntu-24.04 distro.
$ErrorActionPreference = 'Stop'
& wsl.exe -d Ubuntu-24.04 -- test -f /opt/ros/jazzy/setup.bash
if ($LASTEXITCODE -ne 0) { throw 'This setup requires the existing Ubuntu-24.04 / ROS 2 Jazzy installation.' }
& wsl.exe -d Ubuntu-24.04 -u root -- apt-get update
if ($LASTEXITCODE -ne 0) { throw 'apt update failed.' }
& wsl.exe -d Ubuntu-24.04 -u root -- apt-get install -y python3-colcon-common-extensions python3-opencv python3-scipy python3-pytest python3-yaml ros-jazzy-rviz2 ros-jazzy-robot-state-publisher ros-jazzy-interactive-markers ros-jazzy-tf2-ros-py ros-jazzy-cv-bridge ros-jazzy-rosidl-default-generators ros-jazzy-control-msgs
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
Write-Host 'Ready. Run .\start_sim.ps1 to build and open RViz.'
