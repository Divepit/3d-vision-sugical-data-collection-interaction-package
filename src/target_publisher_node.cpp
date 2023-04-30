#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sdc_interaction/ChangeTargetCoordinate.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

geometry_msgs::Point target_coordinate;
visualization_msgs::Marker target_marker;
ros::Publisher target_marker_pub;
ros::Publisher target_coordinate_pub;


bool change_target_coordinate(sdc_interaction::ChangeTargetCoordinate::Request &req,
                              sdc_interaction::ChangeTargetCoordinate::Response &res)
{
    target_coordinate = req.new_coordinate;
    ROS_INFO("New target coordinate: (%f, %f, %f)", target_coordinate.x, target_coordinate.y, target_coordinate.z);

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
    target_marker.scale.x = 0.125;
    target_marker.scale.y = 0.125;
    target_marker.scale.z = 0.125;
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
}

int main(int argc, char **argv)
{
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

    // Create a publisher for the target target_marker
    target_marker_pub = nh.advertise<visualization_msgs::Marker>("target_marker", 1);

    // Set the initial target position
    target_coordinate.x = 4;
    target_coordinate.y = 0;
    target_coordinate.z = 1;

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);   

    // Handle callbacks
    ros::spin();

    return 0;
}