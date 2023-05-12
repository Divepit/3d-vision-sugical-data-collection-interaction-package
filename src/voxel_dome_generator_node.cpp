#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sdc_interaction/UpdateVoxelDome.h>
#include <sdc_interaction/UpdateObstacles.h>
#include <sdc_interaction/ExecuteOberservingPath.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <functional>
#include <sdc_interaction/SphereList.h>
#include <sdc_interaction/Sphere.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// Topic for obstacle locations - set to "/obstacle_locations" for ground truth
std::string obstacle_topic = "/obstacle_centers";
std::string camera_frame_name = "camera_coordinate_system";

// Global variables
visualization_msgs::Marker marker;
ros::Publisher marker_pub;
geometry_msgs::Point root_position;
geometry_msgs::Point target_position;
std::vector<sdc_interaction::Sphere> obstacles;
sdc_interaction::ExecuteOberservingPath srv;
tf2_ros::Buffer tf_buffer;

// Logging
ros::Publisher red_voxels_pub;
ros::Publisher occluded_pub;

bool is_target_occluded(geometry_msgs::Point origin, geometry_msgs::Point target, std::vector<sdc_interaction::Sphere> &obstacles);

double radius = 0.4;
int voxel_count = 2000;
double scaling_factor = 0.4;

ros::ServiceClient execute_observing_path_client;

bool change_flag = true;

geometry_msgs::Point getCameraPosition(tf2_ros::Buffer &tf_buffer)
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

// New target position callback
void targetPositionCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    geometry_msgs::Point msg_pos = *msg;

    // Check if the new target position is different from the current one
    if (msg_pos.x != target_position.x || msg_pos.y != target_position.y || msg_pos.z != target_position.z)
    {
        // Update the target position
        target_position.x = msg_pos.x;
        target_position.y = msg_pos.y;
        target_position.z = msg_pos.z;

        // Set change flag in order for dome to be updated
        change_flag = true;
    }
}

void obstacleLocationsCallback(const sdc_interaction::SphereList::ConstPtr &msg)
{
    // Check if the new obstacle list is different from the current one
    if (obstacles != msg->spheres)
    {
        // Update the obstacles list
        obstacles = msg->spheres;

        // Set the change_flag to true to indicate that the obstacles list has changed
        // Set change flag to false, since updated.
        geometry_msgs::Point camera_position = getCameraPosition(tf_buffer);
        if (is_target_occluded(camera_position , target_position, obstacles))
        {
            change_flag = true;
        }
    }
}

double euclideanDistance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
    double dx = point1.x - point2.x;
    double dy = point1.y - point2.y;
    double dz = point1.z - point2.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Service callback
bool updateVoxelDome(sdc_interaction::UpdateVoxelDome::Request &req,
                     sdc_interaction::UpdateVoxelDome::Response &res)
{
    radius = req.radius;
    voxel_count = req.voxel_count;
    scaling_factor = req.scaling_factor;

    res.success = true;
    res.message = "Voxel dome parameters updated.";

    // Set change flag to false, since updated.
        geometry_msgs::Point camera_position = getCameraPosition(tf_buffer);
        if (is_target_occluded(camera_position , target_position, obstacles))
        {
            change_flag = true;
        }
    return true;
}

bool is_target_occluded(geometry_msgs::Point origin, geometry_msgs::Point target, std::vector<sdc_interaction::Sphere> &obstacles)
{
    geometry_msgs::Point ray_direction;
    ray_direction.x = target.x - origin.x;
    ray_direction.y = target.y - origin.y;
    ray_direction.z = target.z - origin.z;

    double ray_length = std::sqrt(ray_direction.x * ray_direction.x +
                                  ray_direction.y * ray_direction.y +
                                  ray_direction.z * ray_direction.z);

    ray_direction.x /= ray_length;
    ray_direction.y /= ray_length;
    ray_direction.z /= ray_length;

    for (const auto &obstacle : obstacles)
    {
        geometry_msgs::Point origin_to_obstacle;
        origin_to_obstacle.x = obstacle.center.x - origin.x;
        origin_to_obstacle.y = obstacle.center.y - origin.y;
        origin_to_obstacle.z = obstacle.center.z - origin.z;

        double dot_product = origin_to_obstacle.x * ray_direction.x +
                             origin_to_obstacle.y * ray_direction.y +
                             origin_to_obstacle.z * ray_direction.z;

        geometry_msgs::Point closest_raypoint_to_obstacle;

        closest_raypoint_to_obstacle.x = dot_product * ray_direction.x;
        closest_raypoint_to_obstacle.y = dot_product * ray_direction.y;
        closest_raypoint_to_obstacle.z = dot_product * ray_direction.z;

        double shortest_distance_ray_obstacle = std::sqrt(
            pow(origin_to_obstacle.x - closest_raypoint_to_obstacle.x, 2) +
            pow(origin_to_obstacle.y - closest_raypoint_to_obstacle.y, 2) +
            pow(origin_to_obstacle.z - closest_raypoint_to_obstacle.z, 2));

        if (shortest_distance_ray_obstacle < obstacle.radius)
        {
            return true;
        }
    }

    return false;
}

void timerCallback(const ros::TimerEvent &, tf2_ros::Buffer &tf_buffer)
{
    ROS_INFO("[Interaction Debug]Timer callback triggered.");
    geometry_msgs::Point closest_point;
    double min_distance = std::numeric_limits<double>::max();
    double voxel_size = std::cbrt(std::pow(radius, 3) / voxel_count);
    double iteration_step = voxel_size / scaling_factor;
    geometry_msgs::Point camera_position = getCameraPosition(tf_buffer);

    // Generate the voxel positions
    marker.points.clear();
    marker.colors.clear();

    // Define colors
    std_msgs::ColorRGBA green;
    green.r = 0.0;
    green.g = 1.0;
    green.b = 0.0;
    green.a = 1.0;

    std_msgs::ColorRGBA red;
    red.r = 1.0;
    red.g = 0.0;
    red.b = 0.0;
    red.a = 1.0;

    std_msgs::ColorRGBA orange;
    orange.r = 1.0;
    orange.g = 0.5;
    orange.b = 0.0;
    orange.a = 1.0;

    int red_voxels = 0;
    int green_voxels = 0;
    double x_offset = 0.35;
    double z_offset = 0.4;

    ROS_INFO("[Interaction Debug]Generating voxels...");
    for (double x = -radius; x <= radius; x += iteration_step)
    {
        for (double y = -radius; y <= radius; y += iteration_step)
        {
            for (double z = 0; z <= radius; z += iteration_step)
            {
                double dist = std::sqrt(x * x + y * y + z * z);
                if (dist <= radius)
                {
                    geometry_msgs::Point p;

                    p.x = root_position.x + + x_offset + x;
                    p.y = root_position.y + y;
                    p.z = root_position.z + z_offset + z;

                    // Check if the voxel is occluded or not and set the color accordingly
                    bool occluded = is_target_occluded(p, target_position, obstacles);
                    if (occluded)
                    {
                        marker.colors.push_back(red);
                        red_voxels++;
                    }
                    else
                    {
                        marker.colors.push_back(green);
                        green_voxels++;

                        // Calculate Euclidean distance to the camera position
                        double distance_to_camera = euclideanDistance(camera_position, p);

                        // Update the minimum distance and the corresponding point if needed
                        if ((distance_to_camera < min_distance && distance_to_camera > 0.3) && change_flag)
                        {
                            min_distance = distance_to_camera;
                            closest_point = p;
                        }
                    }

                    marker.points.push_back(p);
                }
            }
        }
    }
    ROS_INFO("[Interaction Debug]Voxels generated.");
    // Publish the number of red and green voxels
    std_msgs::Int32 red_voxels_msg;
    red_voxels_msg.data = red_voxels;
    red_voxels_pub.publish(red_voxels_msg);

    if (change_flag)
    {
        srv.request.input_point.x = closest_point.x;
        srv.request.input_point.y = closest_point.y;
        srv.request.input_point.z = closest_point.z;

        // Call the service after the marker is published
        ROS_INFO("[Interaction Debug]Calling service /execute_observing_path...");
        if (execute_observing_path_client.call(srv))
        {
            ROS_INFO("[Interaction Debug]Service call succeeded.");
        }
        else
        {
            ROS_ERROR("Failed to call service /execute_observing_path");
        }
        change_flag = false;
    }
    
    // Publish boolean if the target is occluded or not
    std_msgs::Int32 occluded_msg;
    ROS_INFO("[Interaction Debug]Checking if the target is occluded...");
    if (is_target_occluded(target_position, root_position, obstacles))
    {
        occluded_msg.data = 1;
    }
    else
    {
        occluded_msg.data = 0;
    }
    occluded_pub.publish(occluded_msg);

    // Initialize the tf buffer and listener
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Update the marker's scale
    marker.scale.x = voxel_size * scaling_factor;
    marker.scale.y = voxel_size * scaling_factor;
    marker.scale.z = voxel_size * scaling_factor;

    // Publish the marker
    ROS_INFO("[Interaction Debug]Publishing the marker...");
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);

}

int main(int argc, char **argv)
{   
    ROS_INFO("[Interaction Debug]Starting voxel dome generator node...");
    ros::init(argc, argv, "voxel_dome_generator_node");
    ros::NodeHandle nh;

    // Initialize the tf buffer and listener
    tf2_ros::TransformListener tf_listener(tf_buffer);

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
    marker.lifetime = ros::Duration();

    // Advertise the marker publisher
    marker_pub = nh.advertise<visualization_msgs::Marker>("voxel_dome_marker", 0.5);

    ros::ServiceServer voxeldome_service = nh.advertiseService("update_voxel_dome", updateVoxelDome);
    ros::Subscriber obstacle_locations_sub = nh.subscribe(obstacle_topic, 0.5, obstacleLocationsCallback);
    ros::Subscriber target_coordinates_sub = nh.subscribe("target_coordinates", 0.5, targetPositionCallback);
    
    // Logging
    red_voxels_pub = nh.advertise<std_msgs::Int32>("log/red_voxels", 1);
    occluded_pub = nh.advertise<std_msgs::Int32>("log/occluded", 1);

    execute_observing_path_client = nh.serviceClient<sdc_interaction::ExecuteOberservingPath>("/execute_observing_path");

    // Create a timer to update the marker
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), std::bind(timerCallback, std::placeholders::_1, std::ref(tf_buffer)));

    // Handle callbacks using ros::spin()
    ros::spin();

    return 0;
}
