# Installing

Setup workspace:

```
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build
mkdir catkin_ws
cd catkin_ws
wstool init src
```

Clone repositories:

```
git clone https://github.com/googlecartographer/cartographer.git
git clone https://github.com/marcelino-pensa/cartographer_ros.git
git clone https://ceres-solver.googlesource.com/ceres-solver.git
```

Checkout the correct version of the repositories:

```
cd cartographer
git checkout 1.0.0
cd ../ceres-solver
git checkout 1.13.0
cd ../cartographer_ros
git checkout pensa-branch
cd ../..
```

Install:

```
src/cartographer/scripts/install_proto3.sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make_isolated --install --use-ninja
```
