# Installing

```
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build
mkdir catkin_ws
cd catkin_ws
wstool init src
cd src

git clone https://github.com/googlecartographer/cartographer.git
git clone https://github.com/marcelino-pensa/cartographer_ros.git
git clone https://ceres-solver.googlesource.com/ceres-solver.git

src/cartographer/scripts/install_proto3.sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make_isolated --install --use-ninja
```
