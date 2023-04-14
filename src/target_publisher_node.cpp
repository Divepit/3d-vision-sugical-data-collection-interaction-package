#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sdc_interaction/ChangeTargetCoordinate.h>

geometry_msgs::Point target_coordinate;

bool change_target_coordinate(sdc_interaction::ChangeTargetCoordinate::Request &req,
                              sdc_interaction::ChangeTargetCoordinate::Response &res)
{
    target_coordinate = req.new_coordinate;
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

    // Set the initial target coordinate
    target_coordinate.x = 0.0;
    target_coordinate.y = 0.0;
    target_coordinate.z = 0.0;

    // Publish the target coordinate at 10 Hz
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        target_coordinate_pub.publish(target_coordinate);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
