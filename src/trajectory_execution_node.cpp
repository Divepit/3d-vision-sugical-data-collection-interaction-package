#include <ros/ros.h>
#include <mrirac_msgs/TrajectoryPlan.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <sdc_interaction/AcquireObservableCoordinate.h>

geometry_msgs::Point tracking_target_coordinate;

bool acquire_observable_coordinate(sdc_interaction::AcquireObservableCoordinate::Request &req, sdc_interaction::AcquireObservableCoordinate::Response &res)
{
    ros::NodeHandle nh;
    ros::Publisher observable_coordinate_pub = nh.advertise<geometry_msgs::Point>("observable_coordinate", 1);

    tracking_target_coordinate.x = req.input_point.x;
    tracking_target_coordinate.y = req.input_point.y;
    tracking_target_coordinate.z = req.input_point.z;
    

    ros::ServiceClient execute_observing_client = nh.serviceClient<std_srvs::Empty>("/acquire_observable_coordinate");
    std_srvs::Empty execute_observing_req;
    if (!execute_observing_client.call(execute_observing_req))
    {
        ROS_ERROR("Failed to call execute_observing_path service");
        return false;
    }

    observable_coordinate_pub.publish(tracking_target_coordinate);
    res.success = true;
    return true;
}


bool execute_observing_path(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ros::NodeHandle nh;
    ros::ServiceClient plan_trajectory_client = nh.serviceClient<mrirac_msgs::TrajectoryPlan>("/mrirac_trajectory_planner/plan_trajectory");
    ros::ServiceClient execute_trajectory_client = nh.serviceClient<std_srvs::Empty>("/mrirac_trajectory_planner/execute_trajectory");

    mrirac_msgs::TrajectoryPlan plan_trajectory_req;

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

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_execution_node");
    ros::NodeHandle nh;

    ros::ServiceServer acquire_observable_coordinate_service = nh.advertiseService("acquire_observable_coordinate", acquire_observable_coordinate);
    ros::ServiceServer execute_observing_path_service = nh.advertiseService("execute_observing_path", execute_observing_path);

    ros::spin();

    return 0;
}
