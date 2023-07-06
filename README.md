# threefinger_ctl
ros2 package for three finger tool v2

# Environment

deactivate your conda environment if you have any

```
conda deactivate
```

This package is built based on ROS2 [Foxy](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

Source the environment
```
source install/setup.bash
```

From the root of your workspace (ros2_ws), you can now build your packages using the command:
```
colcon build
```

To build only the my_package package next time, you can run:

```console
colcon build --packages-select my_package
```


then run the driver package to connect to EPOS driver

```
ros2 run threefinger_ctl epos_node
```

then run the publisher node:

```
ros2 run py_pubsub talker
```

To see the data being published on a topic, use:

```
ros2 topic echo <topic_name>
```