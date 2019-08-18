# Installing

Setup workspace:

```
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build
mkdir catkin_ws
cd catkin_ws
wstool init src 
wstool merge -t src https://raw.githubusercontent.com/marcelino-pensa/cartographer_ros/pensa-branch/cartographer_ros.rosinstall
wstool update -t src
```

Install:

```
src/cartographer/scripts/install_proto3.sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make_isolated --install --use-ninja
```
