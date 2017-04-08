#include <ros/ros.h>
#include <ipu_msgs/EvaluateTargets.h>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_client_node");

    ros::NodeHandle node;

    ros::ServiceClient client = node.serviceClient<ipu_msgs::EvaluateTargets>("/ipu_server/eval_targets");
    ipu_msgs::EvaluateTargets srv;

    while(ros::ok()){

        if (client.call(srv))
            ROS_INFO("/ipu_server/eval_targets");
        else
            ROS_ERROR("Failed to call service /ipu_server/eval_targets");

        ros::Duration(0.3).sleep();
    }
    return 1;
}
