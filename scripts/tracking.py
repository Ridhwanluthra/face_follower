#!/usr/bin/env python
import rospy
import cv2
import sys
import logging as log
import numpy
import message_filters

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError   
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg


class image_tracker:

    def __init__(self):
        rospy.init_node('image_tracker', anonymous=True)
        rospy.loginfo("Tracker Started")
        
        self.bridge = CvBridge()

        #Where to publish
        self._output_image_topic = "~image_topic_output"
        print rospy.has_param(self._output_image_topic)
        if rospy.has_param(self._output_image_topic):
            output_image_topic = rospy.get_param(self._output_image_topic)
            self.tracker_pub = rospy.Publisher(output_image_topic,Image, queue_size=10)
      
          
        #Where to subscribe
        self._input_image_topic = "~image_topic_input"
        # print rospy.has_param(self_input_image_topic)
        if rospy.has_param(self._input_image_topic):
            input_image_topic = rospy.get_param(self._input_image_topic)

        

    def callback(self, data):

        self.image_sub = message_filters.Subscriber(input_image_topic, Image)
        self.bbox = message_filters.Subscriber("coordinates", numpy_msg(Floats))
        bbox.registerCallback(callback)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, bbox], 10, 0.1, allow_headerless=True)
        ts.registerCallback(callback)

        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
        tracker_type = tracker_types[2]

        if tracker_type == 'BOOSTING':
            tracker = cv2.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'GOTURN':
            tracker = cv2.TrackerGOTURN_create()

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            ok = frame
        except CvBridgeError as e:
            print(e)
        

        # if not ok:
        #     print "Cannot read video file"
        #     sys.exit()
        ok = tracker.init(frame, bbox)

        if not ok:
            sys.exit()
        # Start timer
        timer = cv2.getTickCount()
        # Update tracker
        ok, bbox = tracker.update(frame)

        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

        try:
            self.tracker_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__' :
    # Set up tracker.
    # Instead of MIL, you can also use+
    rospy.loginfo("simple face tracker...")
    it = image_tracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()