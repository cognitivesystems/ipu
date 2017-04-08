#ifndef PROCESSOR_H
#define PROCESSOR_H


#include <opencv/cv.hpp>
#include <opencv/cvaux.hpp>
#include <opencv/cxmisc.h>
#include <opencv2/imgproc.hpp>
#include <ipu_msgs/Targets.h>
#include <geometry_msgs/PoseArray.h>
#include <ipu_msgs/EvaluateTargets.h>
#include <sstream>

using namespace cv;
using namespace std;
using namespace ipu_msgs;

typedef std::vector<cv::Point3f > Points3f;
typedef std::vector<cv::Point2f > Points2f;

struct CuboidModelPoints{
    Point3f t1,t2,t3,t4;
    Point3f b1,b2,b3,b4;
};

struct CuboidModelDims{
    float x;
    float y;
    float z;
};

struct ScanSpot{
    cv::Point3f pose;
    cv::RotatedRect rect;
    cv::Mat area;
};

typedef std::vector<ScanSpot > ScanZone;

struct IpuTarget{
    uint id;
    cv::Mat texture;
    cv::MatND histo;
};

typedef std::map<uint, IpuTarget > IpuTargetMap;

namespace ipu {
class Processor
{
public:
    Processor();

    void scan_foreground();

    void add_targets(const Targets ts);

    void remove_targets(const Targets ts);

    void update_background(const cv::Mat in);

    void process_foreground();

    void process_rgb2hsv();

    void update_occlusion();

    ipu_msgs::TargetEvalResult evaluate_targets(ipu_msgs::TargetEval ts);

    cv::Mat get_camera_image();

    void update_current_image();

private:
    Points3f generate_model_points(const geometry_msgs::Pose pose);

    Points2f generate_model_image_points(const Points3f p);

    double compute_distance_from_camera(const Target t);

    cv::MatND generate_2d_hist(const cv::Mat hsv, int h_bins=50, int s_bins=60);

    std::vector<float> compute_distance(geometry_msgs::PoseArray pose_array);

    cv::Mat get_pixels_from_rotated_rect(RotatedRect rect);

    IpuTarget construct_target_data(const Target t);

private:
    uint ipu_id_;

    Ptr<BackgroundSubtractor> pMOG2_;

    cv::Mat fg_mask_MOG2_;

    CuboidModelDims model_dimensions_;

    cv::Mat intrinsics_;

    cv::Mat rotation_;

    cv::Mat translation_;

    cv::Mat distortion_coeff_;

    cv::Mat cam_to_zero_;

    Targets targets_;

    std::map<int, ScanSpot> scan_zone_map_;

    VideoCapture camera_handle_;

    cv::Mat camera_curent_image_;

    cv::Mat hsv_curent_image_;

    IpuTargetMap ipu_target_map_;

};


}

#endif // PROCESSOR_H
