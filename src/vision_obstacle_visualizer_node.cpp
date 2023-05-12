#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sdc_interaction/SphereList.h>

ros::Publisher marker_array_pub;

void obstacle_centers_callback(const sdc_interaction::SphereList::ConstPtr& msg)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& sphere : msg->spheres)
    {   
        visualization_msgs::Marker marker;
        marker.header.frame_id = "root";
        marker.header.stamp = ros::Time::now();
        marker.ns = "vision_obstacles";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position = sphere.center;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = sphere.radius * 2;
        marker.scale.y = sphere.radius * 2;
        marker.scale.z = sphere.radius * 2;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        marker_array.markers.push_back(marker);
    }

    marker_array_pub.publish(marker_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_obstacle_visualizer_node");
    ros::NodeHandle nh;

    ros::Subscriber obstacle_centers_sub = nh.subscribe("/obstacle_centers", 0.1, obstacle_centers_callback);
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("vision_obstacles_marker_array", 0.1);

    ros::spin();
    ros::waitForShutdown();
    return 0;
}
