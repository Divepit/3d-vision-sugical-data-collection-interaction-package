// flying_cube_node.cpp
#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>



double observable_target_x, observable_target_y, observable_target_z;

void target_coordinates_callback(const geometry_msgs::Point::ConstPtr &msg)
{
    observable_target_x = msg->x;
    observable_target_y = msg->y;
    observable_target_z = msg->z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "flying_cube_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::Subscriber target_coordinates_sub = nh.subscribe("target_coordinates", 10, target_coordinates_callback);

    gazebo_msgs::SetModelState set_model_state_srv;
    set_model_state_srv.request.model_state.model_name = "target_sphere";
    set_model_state_srv.request.model_state.reference_frame = "world";

     gazebo_msgs::SetModelState obstacle_1;
    obstacle_1.request.model_state.model_name = "obstacle_1";
    obstacle_1.request.model_state.reference_frame = "world";


    double radius = 0.2;
    double angular_speed = 0.8; // rad/s
    double dt = 0.05; // 10 Hz update rate
    ros::Rate rate(1.0/dt);

    while (ros::ok()) {
        double current_time = ros::Time::now().toSec();
        double x = radius * cos(angular_speed * current_time);
        double y = radius * sin(angular_speed * current_time);
        //double z = 1.0; // Height above the ground

        set_model_state_srv.request.model_state.pose.position.x = observable_target_x;
        set_model_state_srv.request.model_state.pose.position.y = observable_target_y;
        set_model_state_srv.request.model_state.pose.position.z = observable_target_z;

        // tf2::Quaternion quaternion;
        // quaternion.setRPY(0, 0, angular_speed * current_time);
        // set_model_state_srv.request.model_state.pose.orientation = tf2::toMsg(quaternion);

        obstacle_1.request.model_state.pose.position.x = observable_target_x + x;
        obstacle_1.request.model_state.pose.position.y = observable_target_y + y;
        obstacle_1.request.model_state.pose.position.z = observable_target_z;


        if (client.call(set_model_state_srv) && client.call(obstacle_1)) {
            // ROS_INFO("Successfully set the cube position and orientation.");
        } else {
            ROS_ERROR("Failed to call the set_model_state service.");
            return 1;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
