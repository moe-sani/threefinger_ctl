# Install & Setup guide

deactivate your conda environment if you have any

```
conda deactivate
```

This package is built based on ROS2 [Foxy](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

```
source /opt/ros/foxy/setup.bash
```

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the following repositories:

```
git clone https://github.com/moe-sani/threefinger_ctl.git
git clone https://github.com/moe-sani/threefinger_publisher.git
git clone https://github.com/ros2/example_interfaces.git

```

```
cd ..
```

You can now build your packages using the command:
From the root of your workspace (ros2_ws)
```
colcon build
```

To build only the my_package package next time, you can run:

```console
colcon build --packages-select my_package
```


```
source install/setup.bash
```

# Running Order

You should first run the driver package to connect to EPOS driver

```
ros2 run threefinger_ctl epos_node
```

then run the keyboard based publisher node:

```
ros2 run threefinger_publisher keyboard_publisher
```
and follow the instructions.
Its advised to first find the limits for each motor and update the following values in

keyboard_publisher.py

```python
CLOSE_POSITION = [ 50000, -100000, 1000]
OPEN_POSITION = [ -20000, 0, 0]
```


