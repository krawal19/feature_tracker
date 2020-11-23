/**
 * BSD 3-Clause License
 * Copyright (c) 2020, Kapil Rawal
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  @copyright (c) BSD
 *
 *  @file   main.cpp
 *
 *  @author   Kapil Rawal (kapilrawal1995@gmail.com)
 *
 *  @copyright   BSD License
 *
 *  @brief   FeatureTracker Class method implmentation
 *
 *  @section   DESCRIPTION
 *
 *  FeatureTracker class method implmentation
 *
 */

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
