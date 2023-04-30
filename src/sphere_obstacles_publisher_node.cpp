#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "sdc_interaction/Sphere.h"
#include "sdc_interaction/SphereList.h"
#include "sdc_interaction/ChangeObstacles.h"


class SphereObstaclesPublisher
{
public:
  SphereObstaclesPublisher()
  {
    publisher_ = nh_.advertise<sdc_interaction::SphereList>("obstacle_locations", 1);
    service_ = nh_.advertiseService("change_obstacles", &SphereObstaclesPublisher::changeObstacles, this);
  }

  void publishObstacles()
  {
    publisher_.publish(obstacles_);
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
