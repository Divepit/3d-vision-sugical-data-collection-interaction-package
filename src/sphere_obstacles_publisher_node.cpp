#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "sdc_interaction/Sphere.h"
#include "sdc_interaction/SphereList.h"
#include "sdc_interaction/ChangeObstacles.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


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
    publisher_ = nh_.advertise<sdc_interaction::SphereList>("obstacle_locations", 1);
    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 1);
    service_ = nh_.advertiseService("change_obstacles", &SphereObstaclesPublisher::changeObstacles, this);
  }

  void publishObstacles()
  {
    publisher_.publish(obstacles_);
    visualization_msgs::MarkerArray marker_array = createObstacleMarkers(obstacles_);
    marker_publisher_.publish(marker_array);
  }

bool changeObstacles(sdc_interaction::ChangeObstacles::Request& req, sdc_interaction::ChangeObstacles::Response& res)
{
  obstacles_ = req.obstacles;
  res.success = true;
  return true;
}


private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  ros::Publisher marker_publisher_;
  ros::ServiceServer service_;
  sdc_interaction::SphereList obstacles_;
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
