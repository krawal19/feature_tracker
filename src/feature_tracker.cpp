#include "feature_tracker.h"

FeatureTracker::FeatureTracker(const ros::NodeHandle& nh):nh_(nh) {
    image_transport::ImageTransport it_(nh);
    // odom_compressed_sub_ = nh_.subscribe<nav_msgs::Odometry>("/dji/odom", 5, &FeatureTracker::visionOdomCallback, this);
    image_compressed_sub_ = nh_.subscribe<sensor_msgs::CompressedImage>("/camera/image/compressed", 1, &FeatureTracker::imageCallback, this);
    image_feature_pub_ = it_.advertise("/feature_tracker/image", 1);
}

FeatureTracker::~FeatureTracker() {}

// void FeatureTracker::visionOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//     ROS_WARN_THROTTLE(1, "No odom received in cb");
// }

void FeatureTracker::imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    try {
        cv_image_ptr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void FeatureTracker::TLD() {

    // cv::SiftFeatureDetector detector;
    // std::vector<cv::KeyPoint> keypoints;
    // detector.detect(cv_image_ptr->image, keypoints);
    // cv::drawKeypoints(cv_image_ptr->image, keypoints, cv_output_ptr->image);
}

void FeatureTracker::track() {
    ros::Rate rate(30);

    cv::Rect2d bbox(287, 23, 86, 320);
    cv::Ptr<cv::Tracker> tracker;
    tracker = cv::TrackerTLD::create();
    // cv::cap >> frame;
    // roi = cv::selectROI("tracker", frame);
    bool init_flag = true;
    bool start_flag = false;

    while (ros::ok()) {
        ros::spinOnce();
        if (cv_image_ptr != nullptr) {
            if (init_flag) {
                tracker->init(cv_image_ptr->image, bbox);
                init_flag = false;
                start_flag = true;
            }
            if (start_flag) {
                tracker->update(cv_image_ptr->image, bbox);
                cv::rectangle(cv_image_ptr->image, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );
            }
            image_feature_pub_.publish(cv_image_ptr->toImageMsg());
        } else 
            ROS_WARN_THROTTLE(1, "No image received");
        rate.sleep();
    }
    ros::shutdown();
}
