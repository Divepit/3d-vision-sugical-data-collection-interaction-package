#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "sdc_interaction/ChangeTargetCoordinate.h"

geometry_msgs::PointStamped target_coordinate;

bool changeTargetCoordinate(
    sdc_interaction::ChangeTargetCoordinate::Request& req,
    sdc_interaction::ChangeTargetCoordinate::Response& res)
{
    target_coordinate.point = req.new_coordinate;
    res.success = true;
    res.message = "Target coordinate successfully updated";
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_publisher_node");
    ros::NodeHandle nh;

    ros::Publisher target_pub = nh.advertise<geometry_msgs::PointStamped>("target_coordinate", 10);
    ros::ServiceServer service = nh.advertiseService("change_target_coordinate", changeTargetCoordinate);

    target_coordinate.header.frame_id = "world";
    target_coordinate.point.x = 0.0;
    target_coordinate.point.y = 0.0;
    target_coordinate.point.z = 0.0;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        target_coordinate.header.stamp = ros::Time::now();
        target_pub.publish(target_coordinate);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
