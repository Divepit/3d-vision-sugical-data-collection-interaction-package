#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sdc_interaction/ChangeTargetCoordinate.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
// #include <gazebo_msgs/SpawnModel.h>
// #include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>


geometry_msgs::Point target_coordinate;
visualization_msgs::Marker target_marker;
ros::Publisher target_marker_pub;
ros::Publisher target_coordinate_pub;
bool position_changed = true;
geometry_msgs::Point previous_coordinate;
std::string target_name;
ros::ServiceClient client_gazebo;
gazebo_msgs::SetModelState set_model_state_srv;



void visualizeGazebo(const geometry_msgs::Point &target_coordinate)
{
    // visualizeGazeboDelete(target_coordinate);
    // visualizeGazeboSpawn(target_coordinate);
    set_model_state_srv.request.model_state.pose.position.x = target_coordinate.x;
    set_model_state_srv.request.model_state.pose.position.y = target_coordinate.y;
    set_model_state_srv.request.model_state.pose.position.z = target_coordinate.z;

    if (client_gazebo.call(set_model_state_srv)) {
    } else {
        ROS_ERROR("[target_publisher_node] Failed to call the set_model_state service.");
    }
}

bool change_target_coordinate(sdc_interaction::ChangeTargetCoordinate::Request &req,
                              sdc_interaction::ChangeTargetCoordinate::Response &res)
{
    if (target_coordinate != req.new_coordinate)
    {
        target_coordinate = req.new_coordinate;
        ROS_INFO("New target coordinate: (%f, %f, %f)", target_coordinate.x, target_coordinate.y, target_coordinate.z);
        position_changed = true;
    }

    res.success = true;
    return true;
}


void timer_callback(const ros::TimerEvent&){
    // Publish target coordinates
    target_coordinate_pub.publish(target_coordinate);

    // Set target_marker properties
    target_marker.header.frame_id = "root";
    target_marker.id = 100;    
    target_marker.type = visualization_msgs::Marker::SPHERE;
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.scale.x = 0.05;
    target_marker.scale.y = 0.05;
    target_marker.scale.z = 0.05;
    target_marker.color.r = 0.0;
    target_marker.color.g = 0.0;
    target_marker.color.b = 1.0;
    target_marker.color.a = 1.0;
    target_marker.pose.position.x = target_coordinate.x;
    target_marker.pose.position.y = target_coordinate.y;
    target_marker.pose.position.z = target_coordinate.z;
    target_marker.pose.orientation.x = 0.0;
    target_marker.pose.orientation.y = 0.0;
    target_marker.pose.orientation.z = 0.0;
    target_marker.pose.orientation.w = 1.0;

    // Publish the target target_marker
    target_marker_pub.publish(target_marker);

    // Visualize the target in Gazebo only if the position has changed
    if (position_changed)
    {
        visualizeGazebo(target_coordinate);
        previous_coordinate = target_coordinate;
        position_changed = false;
    }
}

int main(int argc, char **argv)
{
    // Set target gazebo object name
    target_name = "target_gazebo";

    // Initialize the ROS node
    ros::init(argc, argv, "target_publisher_node");

    // Create a node handle
    ros::NodeHandle nh;

    // Create a publisher for the target_coordinates topic
    target_coordinate_pub = nh.advertise<geometry_msgs::Point>("target_coordinates", 10);

    // Create a service server for changing the target coordinate
    ros::ServiceServer service = nh.advertiseService("change_target_coordinate", change_target_coordinate);

    // Create a client for the clear_octomap service
    ros::ServiceClient clear_octomap_client = nh.serviceClient<std_srvs::Empty>("/clear_octomap");

    // Create a client to change the gazebo position of the target
    client_gazebo = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    set_model_state_srv.request.model_state.model_name = "target_sphere";
    set_model_state_srv.request.model_state.reference_frame = "world";

    // Create a publisher for the target target_marker
    target_marker_pub = nh.advertise<visualization_msgs::Marker>("target_marker", 1);

    // Set the initial target position
    target_coordinate.x = 0.9;
    target_coordinate.y = 0;
    target_coordinate.z = 0.1;

    // Init gazebo target
    // visualizeGazeboSpawn(target_coordinate);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);   

    // Handle callbacks
    ros::spin();

    ros::waitForShutdown();
    return 0;
}