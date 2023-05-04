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

To easily call services, it is easiest if you use rqt. To install rqt use the following command:

```sh
sudo apt-get install ros-noetic-rqt-common-plugins
```

In order to easily call services, first start the whole pipeline and once the rosmaster is running, open rqt as follows:

```sh
rosrun rqt_service_caller rqt_service_caller
```


# Running separate nodes

This package contains two nodes, one to execute a trajectory and bring the end effector to a desired location while looking at the target and a second node that lets us select a new coordinate for the target.

If you already launched the included launchfile mentioned above, you do not need to launch these nodes again, they are already in the launch file. You can however, call their services as explained in the secions below.

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
   
```bash
rosservice call /change_target_coordinate "{new_coordinate: {x: 4.0, y: 0.0, z: 1.75}}"
```

Adjust the `x`, `y`, and `z` values as needed.

  The service should respond with a success message, indicating that the target coordinate has been updated.

# Voxel Dome Generator

The Voxel Dome Generator is a ROS node that generates a half-dome filled with voxels in RViz. The dome's position, radius, and voxel count can be configured using a ROS service.

## Update Voxel Dome Service

The node provides a service named /update_voxel_dome that allows you to update the parameters of the voxel dome. The service takes the following arguments:

* `radius` (float64): The radius of the half-dome.
* `voxel_count` (int32): The number of voxels filling the half-dome.
* `scaling_factor` (float64): A factor to scale the size of the individual voxels. A value less than `1.0` will create space between the voxels.

The service returns a boolean success indicating if the operation was successful and a message providing additional information about the result.

### Usage

After starting the Voxel Dome Generator node, you can call the /update_voxel_dome service using the rosservice command-line tool. Here's an example:

``` bash
rosservice call /update_voxel_dome "radius: 2.0
voxel_count: 200
scaling_factor: 0.9"
```

This command updates the voxel dome's radius to `2.0`, the number of voxels to `200`, and sets the scaling factor to `0.9`, which will result in smaller voxels with more space between them.

You can adjust these values as needed to generate a voxel dome with the desired size and density.

# Sphere Obstacle Publisher Node

## Changing obstacles
To change the obstacles, call the `change_obstacles` service with a list of `Sphere` messages. Here's an example of how to call the service using the `rosservice` command-line tool:

``` bash
rosservice call /change_obstacles "obstacles: {spheres: [{center: {x: 2.0, y: 0.0, z: 1.0}, radius: 0.125}, {center: {x: 2.0, y: 1.0, z: 1.0}, radius: 0.125}]}"
```

This command updates the list of obstacles with two spheres. The first sphere has its center at `(2.0, 0.0, 1.0)` and a radius of `0.125`. The second sphere has its center at `(4.0, 5.0, 6.0)` and a radius of `0.125`.

You can also change the obstacles programmatically in another ROS node by creating a client for the `change_obstacles` service and sending the new list of spheres.

