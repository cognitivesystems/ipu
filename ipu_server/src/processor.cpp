#include "ipu_server/processor.h"

namespace ipu {
Processor::Processor()
    :ipu_id_(0), intrinsics_(3, 3, cv::DataType<double>::type), rotation_matrix_(3, 3, cv::DataType<double>::type),
      translation_(3, 1, cv::DataType<double>::type), distortion_coeff_(5, 1, cv::DataType<double>::type),
      cam_to_zero_(4, 4, cv::DataType<double>::type)
{
    load_configuration();

    std::cout << "Opening Camera." << std::endl;
//    camera_handle_ = cv::VideoCapture(ipu_id_);
    camera_handle_ = cv::VideoCapture("/home/nair/Desktop/cam0.avi");

    if(!camera_handle_.isOpened()){  // check if we succeeded{
        std::cout << "Error: cannot open camera" << std::endl;
        throw;
    }

    camera_handle_ >> camera_curent_image_;

    model_dimensions_.x=100.0;
    model_dimensions_.y=100.0;
    model_dimensions_.z=1800.0;

    pMOG2_ = createBackgroundSubtractorMOG2();

    scan_zone_map_.clear();
    float x_span_min=0.0;
    float x_span_max=1000.0;
    float y_span_min=0.0;
    float y_span_max=1000.0;
    float step=100.0f;

    std::cout << "Generating Scan Area data" << std::endl;

    int counter=0;
    for(float x=x_span_min; x < x_span_max; x=x+step ){
        for(float y=y_span_min; y < y_span_max; y=y+step ){
            ScanSpot spot;
            spot.id=counter++;
            spot.pose.x=0.0;
            spot.pose.y=counter;
            spot.pose.z=0.0;//+model_dimensions_.z/2.0f;
            geometry_msgs::Pose pose;
            pose.position.x=spot.pose.x;pose.position.y=spot.pose.y;pose.position.z=spot.pose.z;
            pose.orientation.x=0.0;pose.orientation.y=0.0;pose.orientation.z=0.0;pose.orientation.w=1.0;
            spot.rect = cv::minAreaRect(generate_model_image_points(generate_model_points(pose)));
            spot.area = get_pixels_from_rotated_rect(spot.rect);
            scan_zone_map_[spot.id]=spot;
            std::cout << x << " " << y << std::endl;
            std::cout << spot.rect.size.area() << std::endl;

        }
    }

//    ScanSpot spot;
//    spot.id=counter++;
//    spot.pose.x=0.0;
//    spot.pose.y=0.0;
//    spot.pose.z=0.0;
//    geometry_msgs::Pose pose;
//    pose.position.x=spot.pose.x;pose.position.y=spot.pose.y;pose.position.z=spot.pose.z;
//    pose.orientation.x=0.0;pose.orientation.y=0.0;pose.orientation.z=0.0;pose.orientation.w=1.0;
//    spot.rect = cv::minAreaRect(generate_model_image_points(generate_model_points(pose)));
//    spot.area = get_pixels_from_rotated_rect(spot.rect);
//    scan_zone_map_[spot.id]=spot;

    std::cout << "Generating Scan Area data done" << std::endl;

}

void Processor::load_configuration()
{
    //    std::string config_file_path=ros::package::getPath("/ipu_server_node")+"/config.yaml";
    std::string config_file_path="/home/nair/catkin_ipu/src/ipu_server/data/config.yaml";

    FileStorage fs(config_file_path, FileStorage::READ);

    ipu_id_=fs["ipu_id"];
    fs["K"] >> intrinsics_;
    fs["R"] >> rotation_matrix_;
    fs["T"] >> translation_;

    tr_cam_to_world_=cv::Mat::eye(4, 4, cv::DataType<double>::type);
    tr_cam_to_target_=cv::Mat::eye(4, 4, cv::DataType<double>::type);
    tr_target_to_world_=cv::Mat::eye(4, 4, cv::DataType<double>::type);

    distortion_coeff_.at<double>(0,0)=0.0;
    distortion_coeff_.at<double>(1,0)=0.0;
    distortion_coeff_.at<double>(2,0)=0.0;
    distortion_coeff_.at<double>(3,0)=0.0;
    distortion_coeff_.at<double>(4,0)=0.0;

    std::cout << "rotation_matrix_" << std::endl;
    std::cout << rotation_matrix_ << std::endl;

    rotation_matrix_.copyTo(tr_cam_to_world_(cv::Rect(0,0,3,3)));

    std::cout << "tr_cam_to_world_" << std::endl;
    std::cout << tr_cam_to_world_ << std::endl;

    std::cout << intrinsics_ << std::endl;
    std::cout << rotation_matrix_ << std::endl;
    std::cout << translation_ << std::endl;
}

void Processor::add_targets(const Targets ts)
{
    for(Target t:ts.targets){
        IpuTarget ipu_target=construct_target_data(t);
        ipu_target_map_[ipu_target.id]=ipu_target;
    }
}

void Processor::remove_targets(const Targets ts)
{
    for(Target t:ts.targets){
        ipu_target_map_.erase(t.id);
    }
}

void Processor::update_background(const cv::Mat in)
{

}

void Processor::process_foreground()
{
    pMOG2_->apply(camera_curent_image_, fg_mask_MOG2_);
}

void Processor::process_rgb2hsv()
{
    cv::Mat hsv;

    cv::cvtColor(camera_curent_image_, hsv, CV_RGB2HSV);

    hsv.copyTo(hsv_curent_image_, fg_mask_MOG2_);
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

TargetEvalResult Processor::evaluate_targets(TargetEval ts)
{
    TargetEvalResult result;


    return result;
}

Mat Processor::get_camera_image()
{
    return camera_curent_image_;
}

Mat Processor::get_output_image()
{
    return output_curent_image_;
}

void Processor::update_current_image()
{
    camera_handle_ >> camera_curent_image_;
    camera_curent_image_.copyTo(output_curent_image_);

}

void Processor::draw_detection_area()
{
    std::cout << "scan_zone_map_ size --> " << scan_zone_map_.size() << std::endl;
    Points3f ps;
    ps.clear();
    for(size_t i=0;i<scan_zone_map_.size();++i){
        ScanSpot spot=scan_zone_map_[i];
        Point3f p;
        p.x=spot.pose.x;
        p.y=spot.pose.y;
        p.z=spot.pose.z;
        ps.push_back(p);
    }
    Points2f ps_2d=generate_model_image_points(ps);

    for(Point2f p2:ps_2d){
        cv::circle(output_curent_image_, cv::Point((int)(p2.x), (int)(p2.y)), 2, cv::Scalar(0,255,0), 2);
    }
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

std::vector<float> Processor::compute_distance(geometry_msgs::PoseArray pose_array)
{
    std::vector<float> dist_array;

    return dist_array;
}

Mat Processor::get_pixels_from_rotated_rect(RotatedRect rect)
{
    // matrices we'll use
    Mat M, rotated, cropped;
    // get angle and size from the bounding box
    float angle = rect.angle;
    Size rect_size = rect.size;
    // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
    if (rect.angle < -45.) {
        angle += 90.0;
        swap(rect_size.width, rect_size.height);
    }
    // get the rotation matrix
    M = getRotationMatrix2D(rect.center, angle, 1.0);
    // perform the affine transformation
    warpAffine(camera_curent_image_, rotated, M, camera_curent_image_.size(), INTER_CUBIC);
    // crop the resulting image
    getRectSubPix(rotated, rect_size, rect.center, cropped);

    return cropped;
}

IpuTarget Processor::construct_target_data(const Target t)
{
    IpuTarget new_target;
    new_target.id=t.id;
    cv::Mat img = get_pixels_from_rotated_rect(cv::minAreaRect(generate_model_image_points(generate_model_points(t.pose.pose))));
    new_target.texture=img;
    cv::Mat hsv;
    cv::cvtColor(img, hsv, CV_BGR2HSV);
    new_target.histo=generate_2d_hist(hsv);
    std::stringstream filename;
    filename.clear();
    filename << "Target_"<<new_target.id<< "_Ipu_"<<ipu_id_<<".jpg";
    cv::imwrite(filename.str(), img);

    return new_target;
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

Points2f Processor::generate_model_image_points(const Points3f ps)
{
    Points2f points_2d;
    Points3f points_cam_3d;
    points_cam_3d.clear();

    for(Point3f p:ps){
        tr_target_to_world_.at<double>(0,3)=p.x;
        tr_target_to_world_.at<double>(1,3)=p.y;
        tr_target_to_world_.at<double>(2,3)=p.z;
        std::cout << "Before --> " << p.x << " " << p.y << " " << p.z << std::endl;

        tr_cam_to_target_=tr_cam_to_world_*tr_target_to_world_;
        p.x=tr_cam_to_target_.at<double>(0,3);
        p.y=tr_cam_to_target_.at<double>(1,3);
        p.z=tr_cam_to_target_.at<double>(2,3);
        points_cam_3d.push_back(p);

        std::cout << "After --> " << p.x << " " << p.y << " " << p.z << std::endl;

    }

    if(points_cam_3d.size()>0){
        cv::projectPoints(points_cam_3d, rotation_matrix_, translation_, intrinsics_, distortion_coeff_, points_2d);
    }

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
