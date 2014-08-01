#!/usr/bin/env python
import rospy
from rqt_graphprofiler.msg import *

def callback(data):
    latency = rospy.get_rostime() - data.window_stop 
    window = data.window_stop - data.window_start
    margin = window - latency
    if latency > window:
        rospy.logerr("Data from '%s' too old by %f secs"%(data.hostname,-margin.to_sec()))
    else:
        rospy.loginfo("Received data from '%s' with margin %f secs"%(data.hostname,margin.to_sec()))

if __name__ == "__main__":
    rospy.init_node('checkprofiler', anonymous=True)
    rospy.Subscriber('/graphprofile', HostProfile, callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
