// This file implements a ROS node that spawns a moving obstacle in Gazebo and publishes its coordinates.
#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include "sdc_interaction/Sphere.h"
#include "sdc_interaction/SphereList.h"
#include "sdc_interaction/ChangeObstacles.h"

// Global variables for observable target coordinates
double observable_target_x, observable_target_y, observable_target_z;

// Callback function to update target coordinates
void target_coordinates_callback(const geometry_msgs::Point::ConstPtr &msg)
{
    observable_target_x = msg->x;
    observable_target_y = msg->y;
    observable_target_z = msg->z;
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "flying_cube_node");
    ros::NodeHandle nh;

    // Read parameter for using dynamic obstacle
    bool use_dynamic_obstacle;
    if (!nh.getParam("use_dynamic_obstacle", use_dynamic_obstacle))
    {
        ROS_ERROR("[flying_cube_node] Failed to get param 'use_dynamic_obstacle'");
    }

    // Initialize service clients and subscribers
    ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    ros::ServiceClient set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient change_obstacle_client = nh.serviceClient<sdc_interaction::ChangeObstacles>("change_obstacles");
    ros::Subscriber target_coordinates_sub = nh.subscribe("target_coordinates", 10, target_coordinates_callback);

    // Prepare spawn model service request
    gazebo_msgs::SpawnModel spawn_model_srv;
    spawn_model_srv.request.model_name = "obstacle_1";
    spawn_model_srv.request.reference_frame = "world";
    spawn_model_srv.request.initial_pose.position.x = 0;
    spawn_model_srv.request.initial_pose.position.y = 0;
    spawn_model_srv.request.initial_pose.position.z = 1;
    spawn_model_srv.request.initial_pose.orientation.x = 0.0;
    spawn_model_srv.request.initial_pose.orientation.y = 0.0;
    spawn_model_srv.request.initial_pose.orientation.z = 0.0;
    spawn_model_srv.request.initial_pose.orientation.w = 1.0;
    spawn_model_srv.request.model_xml =
        "<sdf version='1.6'>"
        "  <model name='obstacle_1'>"
        "    <link name='link'>"
        "      <gravity>false</gravity>"
        "      <visual name='visual'>"
        "        <geometry>"
        "          <sphere>"
        "            <radius>0.05</radius>"
        "          </sphere>"
        "        </geometry>"
        "        <material>"
        "          <script>"
        "            <uri>file://media/materials/scripts/gazebo.material</uri>"
        "            <name>Gazebo/Red</name>"
        "          </script>"
        "        </material>"
        "      </visual>"
        "    </link>"
        "  </model>"
        "</sdf>";

    if (use_dynamic_obstacle) {
        // Spawn the obstacle model in Gazebo
        if (!spawn_model_client.call(spawn_model_srv))
        {
            ROS_ERROR("Failed to spawn model 'obstacle_1' in Gazebo.");
            return 1;
        }
        // Prepare SetModelState message for updating obstacle position
        gazebo_msgs::SetModelState obstacle_1;
        obstacle_1.request.model_state.model_name = "obstacle_1";
        obstacle_1.request.model_state.reference_frame = "world";

        // Set parameters for obstacle motion
        double radius = 0.1;
        double angular_speed = 0.4; // rad/s
        double z_offset = 0.15;
        ros::Rate rate(10);

        // Move obstacle and publish its coordinates
        while (ros::ok()) {
            double current_time = ros::Time::now().toSec();
            double x = radius * cos(angular_speed * current_time);
            double y = radius * sin(angular_speed * current_time);

            // Update obstacle position
            obstacle_1.request.model_state.pose.position.x = observable_target_x + x;
            obstacle_1.request.model_state.pose.position.y = observable_target_y + y;
            obstacle_1.request.model_state.pose.position.z = observable_target_z + z_offset;

            // Send updated obstacle coordinates to change_obstacles service
            sdc_interaction::SphereList sphere_list;
            sdc_interaction::Sphere sphere;
            sphere.center.x = observable_target_x + x;
            sphere.center.y = observable_target_y + y;
            sphere.center.z = observable_target_z + z_offset;
            sphere.radius = 0.05;
            sphere_list.spheres.push_back(sphere);

            sdc_interaction::ChangeObstacles srv;
            srv.request.obstacles = sphere_list;

            if (!change_obstacle_client.call(srv)) {
                ROS_ERROR("[Flying Cube Node] Failed to call change_obstacles service.");
                return 1;
            }

            // Update obstacle position in Gazebo
            if (!set_model_state_client.call(obstacle_1)) {
                ROS_ERROR("[Flying Cube Node] Failed to call the set_model_state service.");
                return 1;
            }

            // Process callbacks and sleep
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}