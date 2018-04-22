#!/usr/bin/env python
import rospy
import cv2
import sys
import logging as log
import numpy
import message_filters

from face_follower.msg import rlist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError   
from message_filters import TimeSynchronizer, Subscriber

class image_tracker:

    def __init__(self):
        rospy.init_node('image_tracker', anonymous=True)
        rospy.loginfo("Tracker Started")
        
        self.bridge = CvBridge()
        self.last_bbox = []
        self.last_frame = []
        self.tracking = False
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
        tracker_type = tracker_types[2]
        # if tracker_type == 'BOOSTING':
        #     tracker = cv2.TrackerBoosting_create()
        # if tracker_type == 'MIL':
        #     tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            self.tracker = cv2.TrackerKCF_create()
        # if tracker_type == 'TLD':
        #     tracker = cv2.TrackerTLD_create()
        # if tracker_type == 'MEDIANFLOW':
        #     tracker = cv2.TrackerMedianFlow_create()
        # if tracker_type == 'GOTURN':
        #     tracker = cv2.TrackerGOTURN_create()
 
        #Where to publish
        self._output_image_topic = "~image_topic_output"
        print rospy.has_param(self._output_image_topic)
        if rospy.has_param(self._output_image_topic):
            output_image_topic = rospy.get_param(self._output_image_topic)
            self.tracker_pub = rospy.Publisher(output_image_topic,Image, queue_size=10)
        
        #Where to subscribe
        self._input_image_topic = "~image_topic_input"
        print rospy.has_param(self._input_image_topic)
        if rospy.has_param(self._input_image_topic):
            input_image_topic = rospy.get_param(self._input_image_topic)
            self.image_sub = message_filters.Subscriber(input_image_topic, Image)
            
        self._coordinates = "~coordinates"
        print rospy.has_param(self._coordinates)
        if rospy.has_param(self._coordinates):
            coordinates = rospy.get_param(self._coordinates)
            self.bbox = message_filters.Subscriber(coordinates, rlist)
        
        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.bbox], 10, 10, allow_headerless=True)
        ts.registerCallback(self.mycallback)


    def mycallback(self, input_image_topic, coordinates):
        try:
            frame = self.bridge.imgmsg_to_cv2(input_image_topic, "bgr8")
        except CvBridgeError as e:
            print(e)

        bbox = coordinates.data
        if bbox:
            self.last_bbox = bbox
            self.last_frame = frame

        if not self.last_bbox:
            rospy.loginfo('had not recieved any face data yet')
            return

        # rospy.loginfo(bbox)
        if self.tracking:
            ok, bbox = self.tracker.update(frame)
            if not ok:
                self.tracking = False
                bbox = self.last_bbox
        else:
            # self.tracker.clear()
            # self.tracker = cv2.TrackerKCF_create()
            ok = self.tracker.init(frame, tuple(self.last_bbox))
            self.tracking = True
            bbox = self.last_bbox
            frame = self.last_frame

        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else:
            rospy.loginfo("not ok")
            rospy.loginfo(self.tracking)

        try:
            self.tracker_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__' :

    rospy.loginfo("simple face tracker...")
    it = image_tracker()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()