#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sdc_interaction/UpdateVoxelDome.h> 
#include <sdc_interaction/UpdateObstacles.h> 
#include <sdc_interaction/ExecuteObservingPath.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <functional>


// Global variables
visualization_msgs::Marker marker;
ros::Publisher marker_pub;
double radius = 0.7;
int voxel_count = 30000;
double scaling_factor = 0.4;

geometry_msgs::Point root_position;
std::vector<geometry_msgs::Point> obstacle_centers;
std::vector<double> obstacle_radii;
geometry_msgs::Point target_position;

std::string camera_frame_name = "camera_coordinate_system";


// Struct for defining obstacles
struct Obstacle {
    geometry_msgs::Point center;
    double radius;
};


geometry_msgs::Point getCameraPosition(tf2_ros::Buffer &tf_buffer) {
    geometry_msgs::Point camera_position;
    try {
        geometry_msgs::TransformStamped transform_camera_to_world = tf_buffer.lookupTransform("world", camera_frame_name, ros::Time(0), ros::Duration(0.1));

        camera_position.x = transform_camera_to_world.transform.translation.x;
        camera_position.y = transform_camera_to_world.transform.translation.y;
        camera_position.z = transform_camera_to_world.transform.translation.z;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }

    return camera_position;
}


// New target position callback
void targetPositionCallback(const geometry_msgs::Point::ConstPtr &msg) {
    target_position = *msg;
}

// New service callback for updating obstacles
bool updateObstacles(sdc_interaction::UpdateObstacles::Request &req,
                     sdc_interaction::UpdateObstacles::Response &res) {
    obstacle_centers = req.obstacle_centers;
    obstacle_radii = req.obstacle_radii;

    res.success = true;
    res.message = "Obstacle array updated.";

    return true;
}

double euclideanDistance(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2) {
    double dx = point1.x - point2.x;
    double dy = point1.y - point2.y;
    double dz = point1.z - point2.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

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

bool is_target_occluded(geometry_msgs::Point origin, geometry_msgs::Point target, std::vector<Obstacle> &obstacles) {
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

    for (const auto &obstacle : obstacles) {
        geometry_msgs::Point origin_to_obstacle;
        origin_to_obstacle.x = obstacle.center.x - origin.x;
        origin_to_obstacle.y = obstacle.center.y - origin.y;
        origin_to_obstacle.z = obstacle.center.z - origin.z;

        double dot_product = origin_to_obstacle.x * ray_direction.x +
                             origin_to_obstacle.y * ray_direction.y +
                             origin_to_obstacle.z * ray_direction.z;

        geometry_msgs::Point closest_raypoint_to_obstacle;
        // closest_raypoint_to_obstacle.x = origin.x + dot_product * ray_direction.x;
        // closest_raypoint_to_obstacle.y = origin.y + dot_product * ray_direction.y;
        // closest_raypoint_to_obstacle.z = origin.z + dot_product * ray_direction.z;

        closest_raypoint_to_obstacle.x = dot_product * ray_direction.x;
        closest_raypoint_to_obstacle.y = dot_product * ray_direction.y;
        closest_raypoint_to_obstacle.z = dot_product * ray_direction.z;

        double shortest_distance_ray_obstacle = std::sqrt(
            pow(origin_to_obstacle.x - closest_raypoint_to_obstacle.x, 2) +
            pow(origin_to_obstacle.y - closest_raypoint_to_obstacle.y, 2) +
            pow(origin_to_obstacle.z - closest_raypoint_to_obstacle.z, 2));

        if (shortest_distance_ray_obstacle < obstacle.radius) {
            return true;
        }
    }

    return false;
}

void timerCallback(const ros::TimerEvent&, tf2_ros::Buffer &tf_buffer) {
    // Generate the voxel positions
    marker.points.clear();
    marker.colors.clear();
    double voxel_size = std::cbrt(std::pow(radius, 3) / voxel_count);
    double iteration_step = voxel_size / scaling_factor;
    geometry_msgs::Point camera_position = getCameraPosition(tf_buffer);
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

    // Create a vector of obstacles from the global variables
    std::vector<Obstacle> obstacles;
    for (size_t i = 0; i < obstacle_centers.size(); ++i) {
        Obstacle obstacle;
        obstacle.center = obstacle_centers[i];
        obstacle.radius = obstacle_radii[i];
        obstacles.push_back(obstacle);
    }
    
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::Point closest_point;

    for (double x = -radius; x <= radius; x += iteration_step) {
        for (double y = -radius; y <= radius; y += iteration_step) {
            for (double z = 0; z <= radius; z += iteration_step) {
                double dist = std::sqrt(x * x + y * y + z * z);
                if (dist <= radius) {
                    geometry_msgs::Point p;

                    p.x = root_position.x + x;
                    p.y = root_position.y + y;
                    p.z = root_position.z + z;

                    // Check if the voxel is occluded or not and set the color accordingly
                    bool occluded = is_target_occluded(p, target_position, obstacles);
                    if (occluded) {
                        marker.colors.push_back(red);

                    } else {
                        marker.colors.push_back(green);

                        // Add the non-occluded point to the vector
                        non_occluded_points.push_back(p);

                        // Calculate Euclidean distance to the camera position
                        double distance_to_camera = euclideanDistance(camera_position, p);

                        // Update the minimum distance and the corresponding point if needed
                        if (distance_to_camera < min_distance) {
                            min_distance = distance_to_camera;
                            closest_point = p;
                        }
                    }

                    marker.points.push_back(p);
                }
            }
        }
    }

    sdc_interaction::ExecuteObservingPath srv;
    srv.request.input_point.x = closest_point.x;
    srv.request.input_point.y = closest_point.y;
    srv.request.input_point.z = closest_point.z;
    if (client.call(srv)) {
        // Handle the service response here, if needed
        ROS_INFO("Service call succeeded.");
    } else {
        ROS_ERROR("Failed to call service /execute_observing_path");
        return 1;
    }

    // Initialize the tf buffer and listener
    tf2_ros::TransformListener tf_listener(tf_buffer);
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

    // Initialize the tf buffer and listener
    tf2_ros::Buffer tf_buffer;
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
    marker_pub = nh.advertise<visualization_msgs::Marker>("voxel_dome_marker", 1);

    // Advertise the service
    ros::ServiceServer voxeldome_service = nh.advertiseService("update_voxel_dome", updateVoxelDome);
    ros::ServiceServer obstacle_service = nh.advertiseService("update_obstacles", updateObstacles);

    ros::Subscriber target_coordinates_sub = nh.subscribe("target_coordinates", 10, targetPositionCallback);

    os::ServiceClient client = nh.serviceClient<sdc_interaction::ExecuteObservingPath>("/execute_observing_path");

    // Create a timer to update the marker
    ros::Timer timer = nh.createTimer(ros::Duration(1), std::bind(timerCallback, std::placeholders::_1, std::ref(tf_buffer)));

    // Handle callbacks using ros::spin()
    ros::spin();

    return 0;
}
