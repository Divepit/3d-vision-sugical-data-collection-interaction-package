#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sdc_interaction/ChangeTargetCoordinate.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>


geometry_msgs::Point target_coordinate;
visualization_msgs::Marker target_marker;
ros::Publisher target_marker_pub;
ros::Publisher target_coordinate_pub;
bool position_changed = true;
geometry_msgs::Point previous_coordinate;
std::string target_name;


void visualizeGazeboDelete(const geometry_msgs::Point &target_coordinate)
{
    static ros::NodeHandle nh;
    static ros::ServiceClient delete_model_client = nh.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

    // Delete the previous target model in Gazebo
    gazebo_msgs::DeleteModel delete_model_srv;
    delete_model_srv.request.model_name = target_name;
    delete_model_client.call(delete_model_srv);
}

void visualizeGazeboSpawn(const geometry_msgs::Point &target_coordinate)
{
    static ros::NodeHandle nh;
    static ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");

    // Spawn the new target model in Gazebo
    gazebo_msgs::SpawnModel spawn_model_srv;
    spawn_model_srv.request.model_name = target_name;
    spawn_model_srv.request.model_xml =
        "<sdf version='1.6'>"
        "  <model name='" + target_name + "'>"
        "    <link name='link'>"
        "      <gravity>false</gravity>"
        "      <self_collide>false</self_collide>"
        "      <visual name='visual'>"
        "        <geometry>"
        "          <sphere>"
        "            <radius>0.05</radius>"
        "          </sphere>"
        "        </geometry>"
        "        <material>"
        "          <ambient>0.0 0.0 1.0 1.0</ambient>"
        "          <diffuse>0.0 0.0 1.0 1.0</diffuse>"
        "        </material>"
        "      </visual>"
        "    </link>"
        "  </model>"
        "</sdf>";
    spawn_model_srv.request.reference_frame = "root";

    geometry_msgs::Pose initial_pose;
    initial_pose.position = target_coordinate;
    initial_pose.orientation.x = 0.0;
    initial_pose.orientation.y = 0.0;
    initial_pose.orientation.z = 0.0;
    initial_pose.orientation.w = 1.0;

    spawn_model_srv.request.initial_pose = initial_pose;

    if (!spawn_model_client.call(spawn_model_srv))
    {
        ROS_ERROR_STREAM("Failed to spawn target model '" << target_name << "' in Gazebo.");
    }
}

void visualizeGazebo(const geometry_msgs::Point &target_coordinate)
{
    visualizeGazeboDelete(target_coordinate);
    visualizeGazeboSpawn(target_coordinate);
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

    // Create a publisher for the target target_marker
    target_marker_pub = nh.advertise<visualization_msgs::Marker>("target_marker", 1);

    // Set the initial target position
    target_coordinate.x = 0.6;
    target_coordinate.y = 0.0;
    target_coordinate.z = 0.0;

    // Init gazebo target
    visualizeGazeboSpawn(target_coordinate);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);   

    // Handle callbacks
    ros::spin();

    return 0;
}