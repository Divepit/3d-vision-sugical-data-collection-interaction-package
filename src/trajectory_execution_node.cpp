#include <ros/ros.h>
#include <mrirac_msgs/TrajectoryPlan.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sdc_interaction/ExecuteOberservingPath.h>

double observable_target_x, observable_target_y, observable_target_z;

void target_coordinates_callback(const geometry_msgs::Point::ConstPtr &msg)
{
    observable_target_x = msg->x;
    observable_target_y = msg->y;
    observable_target_z = msg->z;
    // ROS_INFO("observable_target_x: %f", observable_target_x);
    // ROS_INFO("observable_target_y: %f", observable_target_y);
    // ROS_INFO("observable_target_z: %f", observable_target_z);
}

bool execute_observing_path(sdc_interaction::ExecuteOberservingPath::Request &req, sdc_interaction::ExecuteOberservingPath::Response &res)
{
    ros::NodeHandle nh;
    ros::ServiceClient plan_trajectory_client = nh.serviceClient<mrirac_msgs::TrajectoryPlan>("/mrirac_trajectory_planner/plan_trajectory");
    ros::ServiceClient execute_trajectory_client = nh.serviceClient<std_srvs::Empty>("/mrirac_trajectory_planner/execute_trajectory");

    mrirac_msgs::TrajectoryPlan plan_trajectory_req;
    geometry_msgs::Point point;

    // <---- Calculate Camera Pose
    double x_c, y_c, z_c;

    // // Hardcode Target Position
    // observable_target_x = 1.0;
    // observable_target_y = 0.0;
    // observable_target_z = 0.75;

    // ROS_INFO("observable_target_x: %f", observable_target_x);
    // ROS_INFO("observable_target_y: %f", observable_target_y);
    // ROS_INFO("observable_target_z: %f", observable_target_z);

    // Read desired camera pose from request
    x_c = req.input_point.x;
    y_c = req.input_point.y;
    z_c = req.input_point.z;

    // print desired camera pose x y and z to ros console
    ROS_INFO("desired camera pose x: %f", x_c);
    ROS_INFO("desired camera pose y: %f", y_c);
    ROS_INFO("desired camera pose z: %f", z_c);

    // Compute the direction vector from the camera to the target
    tf2::Vector3 direction_vec(observable_target_x - x_c, observable_target_y - y_c, observable_target_z - z_c);

    // Normalize the vector
    direction_vec.normalize();

    // Compute the rotation axis and angle
    tf2::Vector3 axis = tf2::Vector3(0, 0, 1).cross(direction_vec);
    double angle = std::acos(tf2::Vector3(0, 0, 1).dot(direction_vec));

    // Convert the axis-angle representation to a quaternion
    tf2::Quaternion camera_pose_quarternion;
    camera_pose_quarternion.setRotation(axis, angle);
    // ----> Calculate Camera Poser

    ROS_INFO("camera_pose_quarternion.x() %f", camera_pose_quarternion.x());
    ROS_INFO("camera_pose_quarternion.y() %f", camera_pose_quarternion.y());
    ROS_INFO("camera_pose_quarternion.z() %f", camera_pose_quarternion.z());
    ROS_INFO("camera_pose_quarternion.w() %f", camera_pose_quarternion.w());

    plan_trajectory_req.request.target_pose.position.x = x_c;
    plan_trajectory_req.request.target_pose.position.y = y_c;
    plan_trajectory_req.request.target_pose.position.z = z_c;
    plan_trajectory_req.request.target_pose.orientation.x = camera_pose_quarternion.x();
    plan_trajectory_req.request.target_pose.orientation.y = camera_pose_quarternion.y();
    plan_trajectory_req.request.target_pose.orientation.z = camera_pose_quarternion.z();
    plan_trajectory_req.request.target_pose.orientation.w = camera_pose_quarternion.w();

    if (!plan_trajectory_client.call(plan_trajectory_req))
    {
        ROS_ERROR("Failed to call plan_trajectory service");
        res.success = false;
        return false;
    }

    std_srvs::Empty execute_trajectory_req;
    if (!execute_trajectory_client.call(execute_trajectory_req))
    {
        ROS_ERROR("Failed to call execute_trajectory service");
        res.success = false;
        return false;
    }
    
    res.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_execution_node");
    ros::NodeHandle nh;

    // Create a subscriber for the target_coordinates topic
    ros::Subscriber target_coordinates_sub = nh.subscribe("target_coordinates", 10, target_coordinates_callback);

    ros::ServiceServer execute_observing_path_service = nh.advertiseService("execute_observing_path", execute_observing_path);

    ros::spin();

    return 0;
}
