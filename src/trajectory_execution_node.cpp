#include <ros/ros.h>
#include <mrirac_msgs/TrajectoryPlan.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <sdc_interaction/AcquireObservableCoordinate.h>


bool execute_observing_path(sdc_interaction::AcquireObservableCoordinate::Request &req, sdc_interaction::AcquireObservableCoordinate::Response &res)
{
    ros::NodeHandle nh;
    ros::ServiceClient plan_trajectory_client = nh.serviceClient<mrirac_msgs::TrajectoryPlan>("/mrirac_trajectory_planner/plan_trajectory");
    ros::ServiceClient execute_trajectory_client = nh.serviceClient<std_srvs::Empty>("/mrirac_trajectory_planner/execute_trajectory");

    mrirac_msgs::TrajectoryPlan plan_trajectory_req;
    geometry_msgs::Point point;
    
    point.x = req.input_point.x;
    point.y = req.input_point.y;
    point.z = req.input_point.z;

    // print observable target x y and z to ros console
    ROS_INFO("observable target x: %f", point.x);
    ROS_INFO("observable target y: %f", point.y);
    ROS_INFO("observable target z: %f", point.z);

    plan_trajectory_req.request.target_pose.position.x = -0.51;
    plan_trajectory_req.request.target_pose.position.y = 0.08;
    plan_trajectory_req.request.target_pose.position.z = 0.53;
    plan_trajectory_req.request.target_pose.orientation.x = 0.0;
    plan_trajectory_req.request.target_pose.orientation.y = 0.0;
    plan_trajectory_req.request.target_pose.orientation.z = 0.0;
    plan_trajectory_req.request.target_pose.orientation.w = 1.0;

    if (!plan_trajectory_client.call(plan_trajectory_req))
    {
        ROS_ERROR("Failed to call plan_trajectory service");
        return false;
    }

    std_srvs::Empty execute_trajectory_req;
    if (!execute_trajectory_client.call(execute_trajectory_req))
    {
        ROS_ERROR("Failed to call execute_trajectory service");
        return false;
    }
    
    res.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_execution_node");
    ros::NodeHandle nh;
    ros::ServiceServer execute_observing_path_service = nh.advertiseService("execute_observing_path", execute_observing_path);

    ros::spin();

    return 0;
}