#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sdc_interaction/UpdateVoxelDome.h> // Replace with your actual package name

// Global variables
visualization_msgs::Marker marker;
ros::Publisher marker_pub;
double radius = 1.0;
int voxel_count = 100;
double scaling_factor = 0.8;

geometry_msgs::Point root_position;



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

void timerCallback(const ros::TimerEvent&) {
    // Generate the voxel positions
    marker.points.clear();
    double voxel_size = cbrt(pow(radius, 3) / voxel_count);
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

    // Update the marker's scale
    marker.scale.x = voxel_size * scaling_factor;
    marker.scale.y = voxel_size * scaling_factor;
    marker.scale.z = voxel_size * scaling_factor;

    // Publish the marker
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "voxel_dome_generator_node");
    ros::NodeHandle nh;

    // Root position (hardcoded)
    root_position.x = 0.0;
    root_position.y = 0.0;
    root_position.z = 0.0;

    // Initialize the marker
    marker.header.frame_id = "root";
    marker.ns = "voxel_dome";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();

    // Advertise the marker publisher
    marker_pub = nh.advertise<visualization_msgs::Marker>("voxel_dome_marker", 1);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("update_voxel_dome", updateVoxelDome);

    // Create a timer to update the marker
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

    // Handle callbacks using ros::spin()
    ros::spin();

    return 0;
}
