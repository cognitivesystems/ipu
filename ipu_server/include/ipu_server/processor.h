#ifndef PROCESSOR_H
#define PROCESSOR_H


#include <opencv/cv.hpp>
#include <opencv/cvaux.hpp>
#include <opencv/cxmisc.h>
#include <opencv2/imgproc.hpp>
#include <ipu_msgs/Targets.h>

using namespace cv;
using namespace std;
using namespace ipu_msgs;

struct CuboidModelPoints{
    Point3f t1,t2,t3,t4;
    Point3f b1,b2,b3,b4;
};

struct CuboidModelDims{
    float x;
    float y;
    float z;
};

typedef std::vector<cv::Point3f > Points3f;
typedef std::vector<cv::Point2f > Points2f;

namespace ipu {
class Processor
{
public:
    Processor();

    void set_targets(const Targets t);

    void update_background(const cv::Mat in);

    cv::Mat process_foreground(const cv::Mat in);

    cv::Mat process_rgb2hsv(const cv::Mat in);

    void update_occlusion();

    bool is_occluded(const geometry_msgs::Pose pose);

    cv::MatND generate_2d_hist(const cv::Mat hsv, int h_bins=50, int s_bins=60);

private:
    Points3f generate_model_points(const geometry_msgs::Pose pose);

    Points2f generate_model_image_points(const Points3f p);

    double compute_distance_from_camera(const Target t);

private:
    Ptr<BackgroundSubtractor> pMOG2_;

    cv::Mat fg_mask_MOG2_;

    CuboidModelDims model_dimensions_;

    cv::Mat intrinsics_;

    cv::Mat rotation_;

    cv::Mat translation_;

    cv::Mat distortion_coeff_;

    cv::Mat cam_to_zero_;

    Targets targets_;
};


}

#endif // PROCESSOR_H
