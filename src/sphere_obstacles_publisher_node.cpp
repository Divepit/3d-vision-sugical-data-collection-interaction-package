#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "sdc_interaction/Sphere.h"
#include "sdc_interaction/SphereList.h"
#include "sdc_interaction/ChangeObstacles.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"



visualization_msgs::MarkerArray createObstacleMarkers(const sdc_interaction::SphereList& obstacles)
{
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;

  for (const auto& sphere : obstacles.spheres)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "root";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacles";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = sphere.center;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = sphere.radius * 2;
    marker.scale.y = sphere.radius * 2;
    marker.scale.z = sphere.radius * 2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}


class SphereObstaclesPublisher
{
public:
  SphereObstaclesPublisher()
  { 
    // Read in config params
    if (!nh_.getParam("use_dynamic_obstacle", use_dynamic_obstacle))
    {
        ROS_ERROR("[sphere_obstacles_publisher_node] Failed to get param 'use_dynamic_obstacle'");
    }

    publisher_ = nh_.advertise<sdc_interaction::SphereList>("obstacle_locations", 1);
    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 1);
    service_ = nh_.advertiseService("change_obstacles", &SphereObstaclesPublisher::changeObstacles, this);
  }

  void publishObstacles()
  {
    publisher_.publish(obstacles_);
    visualization_msgs::MarkerArray marker_array = createObstacleMarkers(obstacles_);
    marker_publisher_.publish(marker_array);
    // Only use this gazebo publisher in case we do not use the dynamic obstacle
    if(!use_dynamic_obstacle){
      publishGazebo();
    }
  }

  void publishGazebo()
  {

    if (!obstacles_updated_){
      return;
    }
      
    // Delete previous sphere obstacles in Gazebo
    ros::ServiceClient delete_model_client = nh_.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    int idx = 0;

    for (const auto& sphere : old_obstacles_.spheres)
    {
      std::string sphere_name = "obstacle_gaz_" + std::to_string(++idx);
      gazebo_msgs::DeleteModel delete_model_srv;
      delete_model_srv.request.model_name = sphere_name;
      delete_model_client.call(delete_model_srv); 
    }

    // Update obstacles and spawn new ones in Gazebo
    ros::ServiceClient spawn_model_client = nh_.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    idx = 0;
    for (const auto& sphere : obstacles_.spheres)
    {
      std::string sphere_name = "obstacle_gaz_" + std::to_string(++idx);
      gazebo_msgs::SpawnModel spawn_model_srv;
      spawn_model_srv.request.model_name = sphere_name;

      spawn_model_srv.request.model_xml =
        "<sdf version='1.6'>"
        "  <model name='" + sphere_name + "'>"
        "    <link name='link'>"
        "      <gravity>false</gravity>"
        "      <self_collide>false</self_collide>"
        "      <visual name='visual'>"
        "        <geometry>"
        "          <sphere>"
        "            <radius>" + std::to_string(sphere.radius) + "</radius>"
        "          </sphere>"
        "        </geometry>"
        "        <material>"
        "          <ambient>1.0 0.0 0.0 1.0</ambient>"
        "          <diffuse>1.0 0.0 0.0 1.0</diffuse>"
        "        </material>"
        "      </visual>"
        "    </link>"
        "  </model>"
        "</sdf>";

      spawn_model_srv.request.reference_frame = "root";

      // Set the initial_pose field
      geometry_msgs::Pose initial_pose;
      initial_pose.position = sphere.center;
      initial_pose.orientation.x = 0.0;
      initial_pose.orientation.y = 0.0;
      initial_pose.orientation.z = 0.0;
      initial_pose.orientation.w = 1.0;

      spawn_model_srv.request.initial_pose = initial_pose;

      if (!spawn_model_client.call(spawn_model_srv))
      {
        ROS_ERROR_STREAM("Failed to spawn model '" << sphere_name << "' in Gazebo.");
      }
    }

    // Reset the flag
    obstacles_updated_ = false;
  }

  bool changeObstacles(sdc_interaction::ChangeObstacles::Request& req, sdc_interaction::ChangeObstacles::Response& res)
  {
    old_obstacles_ = obstacles_;
    obstacles_ = req.obstacles;

    obstacles_updated_ = true;
  
    res.success = true;
    return true;
  }


private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  ros::Publisher marker_publisher_;
  ros::ServiceServer service_;
  sdc_interaction::SphereList obstacles_;
  sdc_interaction::SphereList old_obstacles_;
  bool obstacles_updated_ = false;
  bool use_dynamic_obstacle;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sphere_obstacles_publisher_node");
  SphereObstaclesPublisher sphere_obstacles_publisher;

  ros::Rate loop_rate(10); // 10 Hz
  while (ros::ok())
  {
    sphere_obstacles_publisher.publishObstacles();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
