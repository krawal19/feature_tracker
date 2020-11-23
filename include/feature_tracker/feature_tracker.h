#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/tracking.hpp>

class FeatureTracker {
 public:
    FeatureTracker(const ros::NodeHandle& nh);
    ~FeatureTracker();
	void TLD();
    void track();

 private:
    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);

 private:
    ros::NodeHandle nh_;
	ros::Subscriber image_compressed_sub_; 
    image_transport::Publisher image_feature_pub_;
    cv_bridge::CvImagePtr cv_image_ptr;
};

#endif  // FEATURE_TRACKER_H_