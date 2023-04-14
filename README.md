# Setup and run of full stack

To run the services, follow the steps below:

1. Build your package using catkin_make or catkin build.
2. Source the ROS environment and your workspace:

```sh
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash  # Replace with the path to your workspace if different
```

To run all nodes and services with the kinova mrirac library combined, launch the launchfile:

```sh
roslaunch sdc_interaction interaction.launch 
```

# Running separate nodes

This package contains two nodes, one to execute a trajectory and bring the end effector to a desired location while looking at the target and a second node that lets us select a new coordinate for the target:

## Trajectory execution node

Start the trajectory_execution_node in one terminal:

```sh
rosrun sdc_interaction trajectory_execution_node
```

This will launch the node, which provides the "execute_observing_path" services.

In a separate terminal, source the ROS environment and your workspace again (as in step 2).

To test the "execute_observing_path" service, use the following command:

```sh
rosservice call /execute_observing_path "input_point:
  x: 0.3
  y: 0.1
  z: 0.5"
```

This will call the service with an (x, y, z) coordinate of (1.0, 2.0, 3.0) as input.

## Target publisher node


Run the `target_publisher_node` to start publishing the target coordinates and the `change_target_coordinate` service.
   
```sh
rosrun sdc_interaction target_publisher_node
```

Open another terminal and source the ROS environment and your package's setup.bash file.
   
```sh
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Replace `noetic` with your installed ROS version (e.g., `melodic`, `noetic`).

To call the `change_target_coordinate` service and update the target coordinate, use the following command:
   
```sh
rosservice call /change_target_coordinate "{new_coordinate: {x: 1.0, y: 2.0, z: 3.0}}"
```

Adjust the `x`, `y`, and `z` values as needed.

  The service should respond with a success message, indicating that the target coordinate has been updated.
