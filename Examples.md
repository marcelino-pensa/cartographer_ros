# Running an example from a bag

- Copy the set of example bags:

```
git clone https://github.com/marcelino-pensa/cartographer_bags
```

- In one terminal, source `cartographer` and launch it:

```
source ~/catkin_ws/devel_isolated/setup.bash
roslaunch cartographer_ros demo_pcs_nobag.launch
```

- In another terminal, run one of the bags with the `--clock` option enabled, as the example below:

```
rosbag play heb1_high.bag --clock
```

- Cartographer creates multiple topics and services, but the ones below are the ones that were created locally at pensa:

Topics:

```
/cartographer/odom_from_start
/cartographer/odom_xy_yaw_drift
/cartographer/accumul_odom_xy_yaw_drift
```

Services:

```
/cartographer/terminate_mapping
/cartographer/start_mapping
```