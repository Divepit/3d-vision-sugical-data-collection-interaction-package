To run the services, follow the steps below:

1. Build your package using catkin_make or catkin build.
2. Source the ROS environment and your workspace:

```sh
source /opt/ros/<your_ros_version>/setup.bash
source ~/catkin_ws/devel/setup.bash  # Replace with the path to your workspace if different
```

1. Launch the required nodes and services:

Start the trajectory_execution_node in one terminal:

```sh
rosrun sdc_interaction trajectory_execution_node
```

This will launch the node, which provides the "execute_observing_path" services.

1. In a separate terminal, source the ROS environment and your workspace again (as in step 2).

2. Test the service:

To test the "acquire_observable_coordinate" service, use the following command:

```sh
rosservice call /execute_observing_path "input_point:
  x: 1.0
  y: 2.0
  z: 3.0"
```

This will call the service with an (x, y, z) coordinate of (1.0, 2.0, 3.0) as input.