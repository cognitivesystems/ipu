#include <ros/ros.h>
#include "ipu_server/processor.h"
#include <ipu_msgs/EvaluateTargets.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
//#include <opencv/highgui.h>

ipu::Processor processor_handle;
ros::ServiceServer srv_eval_targets;
image_transport::Publisher pub;

void publish_debug_image(){
    processor_handle.update_current_image();
    cv_bridge::CvImage cv_img;
    cv_img.image=processor_handle.get_camera_image();
    sensor_msgs::ImageConstPtr msg;
    msg=cv_img.toImageMsg();
    pub.publish(msg);
//    cv::imshow("img", cv_img.image);
//    cv::waitKey(10);

}

bool eval_targets_callback( ipu_msgs::EvaluateTargets::Request& request,
                            ipu_msgs::EvaluateTargets::Response& response){

    ROS_INFO("eval_targets_callback");

    processor_handle.get_camera_image();
    publish_debug_image();

    response.success=true;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ipu_server_node");

    ros::NodeHandle node;

    srv_eval_targets = node.advertiseService("/ipu_server/eval_targets", &eval_targets_callback);

    image_transport::ImageTransport it(node);
    pub = it.advertise("camera/image", 1);

    ros::spin();

    return 1;
}
