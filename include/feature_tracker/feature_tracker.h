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
 *  @brief   FeatureTracker Class 
 *
 *  @section   DESCRIPTION
 *
 *  FeatureTracker class
 *
 */

#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/tracking.hpp>

/**
 * @brief Feature tracker class
 * 
 */
class FeatureTracker {
 public:
    /**
     * @brief Construct a new Feature Tracker object
     * 
     * @param nh input node handle
     */
    FeatureTracker(const ros::NodeHandle& nh);
    /**
     * @brief Destroy the Feature Tracker object
     * 
     */
    ~FeatureTracker();
    /**
     * @brief Feature tracker method
     * 
     */
    void track();

 private:
    /**
     * @brief Call back for compressed image  topic
     * 
     * @param msg  compressed oimage data
     */
    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);

 private:
    // Ros node handle
    ros::NodeHandle nh_;
    // Ros subscriber
	ros::Subscriber image_compressed_sub_;
    // Image transport publisher 
    image_transport::Publisher image_feature_pub_;
    // cvbridge variable object for input image
    cv_bridge::CvImagePtr cv_image_ptr;
};

#endif  // FEATURE_TRACKER_H_