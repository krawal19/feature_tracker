#include "feature_tracker.h"

FeatureTracker::FeatureTracker(const ros::NodeHandle& nh):nh_(nh) {
    image_transport::ImageTransport it_(nh);
    // subscriber
    image_compressed_sub_ = nh_.subscribe<sensor_msgs::CompressedImage>
                            ("/camera/image/compressed", 1, &FeatureTracker::imageCallback, this);
    // publisher
    image_feature_pub_ = it_.advertise
                         ("/feature_tracker/image", 1);
}

FeatureTracker::~FeatureTracker() {}

void FeatureTracker::imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    try {
        // copying input image
        cv_image_ptr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void FeatureTracker::track() {
    // ros rate
    ros::Rate rate(30);
    // setting the bbox for image
    cv::Rect2d bbox(400,400,500,500);
    // initalizing the tracker object
    cv::Ptr<cv::Tracker> tracker;
    tracker = cv::TrackerTLD::create();
    bool init_flag = true;
    // main ros loop
    while (ros::ok()) {
        ros::spinOnce();
        // checking if the image is present
        if (cv_image_ptr != nullptr) {
            // initalizing the initial frame in tracker
            if (init_flag) {
                tracker->init(cv_image_ptr->image, bbox);
                init_flag = false;
                start_flag = true;
            } else {
                // updating the tracker frame 
                tracker->update(cv_image_ptr->image, bbox);
                // drawing box 
                cv::rectangle(cv_image_ptr->image, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );
            }
            image_feature_pub_.publish(cv_image_ptr->toImageMsg());
        } else 
            ROS_WARN_THROTTLE(1, "No image received");
        rate.sleep();
    }
    ros::shutdown();
}
