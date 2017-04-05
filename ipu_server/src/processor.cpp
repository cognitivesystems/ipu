#include "ipu_server/processor.h"

namespace ipu {
Processor::Processor()
    :intrinsics_(3, 3, cv::DataType<double>::type), rotation_(3, 1, cv::DataType<double>::type),
      translation_(3, 1, cv::DataType<double>::type), distortion_coeff_(5, 1, cv::DataType<double>::type),
      cam_to_zero_(4, 4, cv::DataType<double>::type)
{
    model_dimensions_.x=500.0;
    model_dimensions_.y=500.0;
    model_dimensions_.z=1800.0;

    pMOG2_ = createBackgroundSubtractorMOG2();
}

void Processor::set_targets(const Targets t)
{
    targets_=t;
}

void Processor::update_background(const cv::Mat in)
{

}

Mat Processor::process_foreground(const Mat in)
{
    pMOG2_->apply(in, fg_mask_MOG2_);

    return fg_mask_MOG2_;
}

cv::Mat Processor::process_rgb2hsv(const cv::Mat in)
{
    cv::Mat hsv;

    cv::Mat out;

    cv::cvtColor(in, hsv, CV_RGB2HSV);

    hsv.copyTo(out, fg_mask_MOG2_);

    return out;
}

void Processor::update_occlusion()
{
    std::map<int, RotatedRect> rect_map;
    std::map<int, double> dist_from_cam;

    for(size_t i=0;i<targets_.targets.size();++i){
        geometry_msgs::Pose pose = targets_.targets[i].pose.pose;
        rect_map[targets_.targets[i].id] = cv::minAreaRect(generate_model_image_points(generate_model_points(pose)));
        dist_from_cam[targets_.targets[i].id] = compute_distance_from_camera(targets_.targets[i]);
        targets_.targets[i].occluded=false;
    }

    //check occlusion
    for(size_t i=0;i<targets_.targets.size();++i){
        if(!targets_.targets[i].occluded){
            for(size_t t=0;t<targets_.targets.size();++t){
                if(targets_.targets[i].id!=targets_.targets[t].id){
                    std::vector<Point2f> int_points;
                    if(cv::rotatedRectangleIntersection(rect_map[targets_.targets[i].id], rect_map[targets_.targets[t].id], int_points)!=INTERSECT_NONE){
                        if(dist_from_cam[targets_.targets[i].id]>dist_from_cam[targets_.targets[t].id]){
                            targets_.targets[i].occluded=true;
                            break;
                        }
                        else{
                            targets_.targets[t].occluded=true;
                        }
                    }
                }
            }
        }
    }
}

bool Processor::is_occluded(const geometry_msgs::Pose pose)
{
    bool occluded=true;
    //project pose

    return occluded;
}

cv::MatND Processor::generate_2d_hist(const cv::Mat hsv, int h_bins, int s_bins)
{
    //int h_bins = 50; int s_bins = 60;

    cv::MatND histo_2d;
    int histSize[] = {h_bins, s_bins};
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    int channels[] = {0, 1};
    cv::calcHist( &hsv, 1, channels, cv::Mat(), // do not use mask
                  histo_2d, 2, histSize, ranges,
                  true, // the histogram is uniform
                  false );

    cv::normalize( histo_2d, histo_2d, 0, 1, NORM_MINMAX, -1, Mat() );

    return histo_2d;
}

Points3f Processor::generate_model_points(const geometry_msgs::Pose pose)
{
    static float halfWidth = model_dimensions_.x / 2.0f;
    static float halfHeight = model_dimensions_.y / 2.0f;
    static float halfDepth = model_dimensions_.z / 2.0f;

    float cx=pose.position.x;
    float cy=pose.position.y;
    float cz=pose.position.z;

    Points3f points;
    points.push_back(cv::Point3f(cx - halfWidth, cy - halfHeight, cz - halfDepth));
    points.push_back(cv::Point3f(cx - halfWidth, cy - halfHeight, cz + halfDepth));
    points.push_back(cv::Point3f(cx - halfWidth, cy + halfHeight, cz - halfDepth));
    points.push_back(cv::Point3f(cx - halfWidth, cy + halfHeight, cz + halfDepth));
    points.push_back(cv::Point3f(cx + halfWidth, cy - halfHeight, cz - halfDepth));
    points.push_back(cv::Point3f(cx + halfWidth, cy - halfHeight, cz + halfDepth));
    points.push_back(cv::Point3f(cx + halfWidth, cy + halfHeight, cz - halfDepth));
    points.push_back(cv::Point3f(cx + halfWidth, cy + halfHeight, cz + halfDepth));

    return points;
}

Points2f Processor::generate_model_image_points(const Points3f p)
{
    Points2f points_2d;

    cv::projectPoints(p, rotation_, translation_, intrinsics_, distortion_coeff_, points_2d);

    return points_2d;
}

double Processor::compute_distance_from_camera(const Target t)
{
    double distance=0;
    static cv::Mat cam_to_target(4, 4, cv::DataType<double>::type);
    static cv::Mat target_to_zero(4, 4, cv::DataType<double>::type);

    cam_to_target=cam_to_zero_*target_to_zero.inv();

    distance= sqrt(pow(cam_to_target.at<double>(0, 3), 2)
                   +pow(cam_to_target.at<double>(1, 3), 2)
                   +pow(cam_to_target.at<double>(2, 3), 2));

    return distance;
}

}
