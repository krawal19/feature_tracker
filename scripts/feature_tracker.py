#!/usr/bin/env python
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