#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sdc_interaction/ChangeTargetCoordinate.h>
#include <std_srvs/Empty.h> // Include the header file for the empty service

geometry_msgs::Point target_coordinate;

bool change_target_coordinate(sdc_interaction::ChangeTargetCoordinate::Request &req,
                              sdc_interaction::ChangeTargetCoordinate::Response &res)
{
    target_coordinate = req.new_coordinate;
    ROS_INFO("New target coordinate: (%f, %f, %f)", target_coordinate.x, target_coordinate.y, target_coordinate.z);

    res.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_publisher_node");
    ros::NodeHandle nh;

    // Create the publisher for the target coordinates topic
    ros::Publisher target_coordinate_pub = nh.advertise<geometry_msgs::Point>("target_coordinates", 10);

    // Create the service for changing the target coordinate
    ros::ServiceServer service = nh.advertiseService("change_target_coordinate", change_target_coordinate);

    // Create the client for the clear_octomap service
    ros::ServiceClient clear_octomap_client = nh.serviceClient<std_srvs::Empty>("/clear_octomap");

    // Set the initial target coordinate
    target_coordinate.x = 0.5;
    target_coordinate.y = 0.0;
    target_coordinate.z = 0.05;

    // Publish the target coordinate at 10 Hz
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        target_coordinate_pub.publish(target_coordinate);

        // // Call the clear_octomap service 5 times per second
        // for (int i = 0; i < 5; i++)
        // {
        //     std_srvs::Empty clear_octomap_srv;
        //     clear_octomap_client.call(clear_octomap_srv);
        //     ros::Duration(10).sleep(); // Sleep for 0.2 seconds (1/5 seconds)
        // }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
