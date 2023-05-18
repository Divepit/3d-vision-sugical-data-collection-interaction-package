#include <ros/ros.h>
#include <random>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "sdc_interaction/Sphere.h"
#include "sdc_interaction/SphereList.h"
#include "sdc_interaction/ChangeObstacles.h"

// Global variables
tf2_ros::Buffer tf_buffer;
ros::ServiceClient spawn_client;
ros::ServiceClient set_model_state_client;
ros::ServiceClient change_obstacle_client;
geometry_msgs::Point target_position;
bool replan_trigger_obstacle;
double replan_trigger_interval;
bool replan_trigger_square;
double replan_trigger_square_bound_x;
double replan_trigger_square_bound_y;
double replan_trigger_square_offset_z;

// Params
std::string camera_frame_name = "camera_coordinate_system";

geometry_msgs::Point getCameraPosition()
{
    geometry_msgs::Point camera_position;
    try
    {
        geometry_msgs::TransformStamped transform_camera_to_world = tf_buffer.lookupTransform("root", camera_frame_name, ros::Time(0), ros::Duration(0.1));

        camera_position.x = transform_camera_to_world.transform.translation.x;
        camera_position.y = transform_camera_to_world.transform.translation.y;
        camera_position.z = transform_camera_to_world.transform.translation.z;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return camera_position;
}

void targetPositionCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    // target_position = *msg;
    geometry_msgs::Point msg_pos = *msg;

    // Update target position
    target_position.x = msg_pos.x;
    target_position.y = msg_pos.y;
    target_position.z = msg_pos.z;
}

void positionTriggerObstacle(const ros::TimerEvent&) {
    if (replan_trigger_obstacle) {
        geometry_msgs::Point trigger_obstacle_position;

        if (replan_trigger_square){
            // Create a random number generator
            std::random_device rd;
            std::mt19937 gen(rd());
            
            // Define the range for x and y coordinates
            std::uniform_real_distribution<double> xDist(target_position.x - replan_trigger_square_bound_x, target_position.x + replan_trigger_square_bound_x);
            std::uniform_real_distribution<double> yDist(target_position.y - replan_trigger_square_bound_y, target_position.y + replan_trigger_square_bound_y);
            
            // Generate random x and y coordinates within the given intervals
            trigger_obstacle_position.x = xDist(gen);
            trigger_obstacle_position.y = yDist(gen);
            trigger_obstacle_position.z = target_position.z + replan_trigger_square_offset_z;

        }else{// Place obstacle between target and camera
            // Get camera position
            geometry_msgs::Point camera_position = getCameraPosition();
            trigger_obstacle_position.x = (camera_position.x + target_position.x) / 2.0;
            trigger_obstacle_position.y = (camera_position.y + target_position.y) / 2.0;
            trigger_obstacle_position.z = (camera_position.z + target_position.z) / 2.0;
        }
        // Set model state in gazebo
        gazebo_msgs::SetModelState set_model_state_srv;
        set_model_state_srv.request.model_state.model_name = "trigger_obstacle";
        set_model_state_srv.request.model_state.pose.position = trigger_obstacle_position;

        if (!set_model_state_client.call(set_model_state_srv)) {
            ROS_ERROR("Failed to call service set_model_state");
        }

        // Send updated obstacle coordinates to change_obstacles service
        sdc_interaction::SphereList sphere_list;
        sdc_interaction::Sphere sphere;
        sphere.center.x = trigger_obstacle_position.x;
        sphere.center.y = trigger_obstacle_position.y;
        sphere.center.z = trigger_obstacle_position.z;
        sphere.radius = 0.05;
        sphere_list.spheres.push_back(sphere);

        sdc_interaction::ChangeObstacles srv;
        srv.request.obstacles = sphere_list;

        if (!change_obstacle_client.call(srv)) {
            ROS_ERROR("[replan_trigger_obstacles_node] Failed to call change_obstacle service.");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "replan_trigger_obstacle_node");
    ros::NodeHandle nh;

    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    // Get required parameters
    if (!nh.getParam("replan_trigger_obstacle", replan_trigger_obstacle) || 
        !nh.getParam("replan_trigger_interval", replan_trigger_interval) || 
        !nh.getParam("replan_trigger_square", replan_trigger_square))
    {
        ROS_ERROR("[replan_trigger_obstacle_node] Failed to get config params.");
    }

    if (replan_trigger_square &&
        !nh.getParam("replan_trigger_square_bound_x", replan_trigger_square_bound_x) || 
        !nh.getParam("replan_trigger_square_bound_y", replan_trigger_square_bound_y) || 
        !nh.getParam("replan_trigger_square_offset_z", replan_trigger_square_offset_z))
    {
        ROS_ERROR("[replan_trigger_obstacle_node] Failed to get config params.");
    }

    // Subscribe to target_coordinates topic
    ros::Subscriber target_sub = nh.subscribe("target_coordinates", 0.5, targetPositionCallback);

    // service clients
    spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    change_obstacle_client = nh.serviceClient<sdc_interaction::ChangeObstacles>("change_obstacles");

    // Spawn model at origin
    gazebo_msgs::SpawnModel spawn_model_srv;
    spawn_model_srv.request.model_name = "trigger_obstacle";
    spawn_model_srv.request.reference_frame = "world";
    spawn_model_srv.request.initial_pose.position.x = 4.0;
    spawn_model_srv.request.initial_pose.position.y = 0.0;
    spawn_model_srv.request.initial_pose.position.z = 1.0;
    spawn_model_srv.request.initial_pose.orientation.x = 0.0;
    spawn_model_srv.request.initial_pose.orientation.y = 0.0;
    spawn_model_srv.request.initial_pose.orientation.z = 0.0;
    spawn_model_srv.request.initial_pose.orientation.w = 1.0;
    spawn_model_srv.request.model_xml =
        "<sdf version='1.6'>"
        "  <model name='trigger_obstacle'>"
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

    if (!spawn_client.call(spawn_model_srv)) {
        ROS_ERROR("[replan_trigger_obstacle_node] Failed to call service spawn_sdf_model");
    }

    // Create timer to place obstacle for triggering replanning
    ros::Timer timer = nh.createTimer(ros::Duration(replan_trigger_interval), positionTriggerObstacle);

    ros::spin();

    return 0;
}
