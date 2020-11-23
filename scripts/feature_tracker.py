#!/usr/bin/env python0
'''
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
 *  @brief   FeatureTracker Class with method implmentation
 *
 *  @section   DESCRIPTION
 *
 *  FeatureTracker class with method implmentation, tracks features in the 
 *  selected region
 *
'''

import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage

class feature_tracker:
    def __init__(self):
        # subscribed topic
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.imageCallback,  queue_size = 1)
        # publishing topic
        self.image_pub = rospy.Publisher("/output/feature_image/compressed", CompressedImage)
        # initalized flag for tracker
        self.init_flag = True
        # Tracker object created
        self.tracker = cv2.TrackerKCF_create()
        # Region to track in input image
        self.bbox = (400,400,500,500)

    # @imageCallback callback for compressed image
    def imageCallback(self, ros_data):
        rospy.loginfo_throttle(1, "Image received on camera topic")

        # converting to cv2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # passing image to tracker function
        self.kcfTracker(input_image)

    def kcfTracker(self, frame):
        # setting initial value of bbox_obtained
        bbox_obtained = self.bbox
        # if inital flag is true then set the initial frame in the tracker
        if (self.init_flag):
            track_status = self.tracker.init(frame, self.bbox)
            self.init_flag = False
            if not track_status:
                rospy.logwarn("Unable to initalize tracker")
        else:
            # updating the frames in tracker
            track_status, bbox_obtained = self.tracker.update(frame)
            if track_status:
                p1 = (int(bbox_obtained[0]), int(bbox_obtained[1]))
                p2 = (int(bbox_obtained[0] + bbox_obtained[2]), int(bbox_obtained[1] + bbox_obtained[3]))
                cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
            else:
                cv2.putText(frame, "Tracking failure detected", (60,80), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)

        # Publish new feature image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        self.image_pub.publish(msg)

def main():
    # initalizing the tracker
    tracker = feature_tracker()
    # creating node
    rospy.init_node('feature_tracker', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting feature tracker node")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()