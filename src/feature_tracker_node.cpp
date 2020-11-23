#include "feature_tracker.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "feature_tracker_node");
    ROS_INFO_ONCE("Feature Tracker node started");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    FeatureTracker tracker(nhPrivate);
    tracker.track();
    return 0;
}
