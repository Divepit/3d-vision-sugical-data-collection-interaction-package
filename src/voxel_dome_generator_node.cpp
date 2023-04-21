#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sdc_interaction/UpdateVoxelDome.h> // Replace with your actual package name

// Global variables
double radius = 1.0;
int voxel_count = 100;
double scaling_factor = 0.8;

// Service callback
bool updateVoxelDome(sdc_interaction::UpdateVoxelDome::Request &req,
                     sdc_interaction::UpdateVoxelDome::Response &res) {
    radius = req.radius;
    voxel_count = req.voxel_count;
    scaling_factor = req.scaling_factor;

    res.success = true;
    res.message = "Voxel dome parameters updated.";

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "voxel_dome_generator_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("voxel_dome", 1);
    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("update_voxel_dome", updateVoxelDome);

    // Hardcoded parameters
    geometry_msgs::Point root_position;
    root_position.x = 0.0;
    root_position.y = 0.0;
    root_position.z = 0.0;

    double radius = 0.9;
    int voxel_count = 300;

    // Marker setup
    visualization_msgs::Marker marker;
    marker.header.frame_id = "root";
    marker.header.stamp = ros::Time();
    marker.ns = "voxel_dome";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    // Color setup
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    // Scale setup
    double scaling_factor = 0.8;
    double voxel_size = cbrt(pow(radius, 3) / voxel_count);
    marker.scale.x = voxel_size * scaling_factor;
    marker.scale.y = voxel_size * scaling_factor;
    marker.scale.z = voxel_size * scaling_factor;

    // Set the marker's pose orientation explicitly to identity quaternion
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Lock the marker to the frame
    marker.frame_locked = true;


    // Generate the voxel positions
    double iteration_step = voxel_size / scaling_factor;
    for (double x = -radius; x <= radius; x += iteration_step) {
        for (double y = -radius; y <= radius; y += iteration_step) {
            for (double z = 0; z <= radius; z += iteration_step) {
                double dist = sqrt(x * x + y * y + z * z);
                if (dist <= radius) {
                    geometry_msgs::Point p;
                    p.x = root_position.x + x;
                    p.y = root_position.y + y;
                    p.z = root_position.z + z;
                    marker.points.push_back(p);
                }
            }
        }
    }


    ros::Rate r(1);
    while (ros::ok()) {
        marker.header.stamp = ros::Time::now();
        marker_pub.publish(marker);
        r.sleep();
    }

    return 0;
}
