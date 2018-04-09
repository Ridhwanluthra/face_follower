#!/usr/bin/env python
import rospy
import cv2
import sys
import logging as log
import numpy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError   
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg


class image_tracker:
    def __init__(self):
        rospy.init_node('image_tracker', anonymous=True)
        rospy.loginfo("Tracker Started")
        
        self.bridge = CvBridge()

        self._input_image_topic = "~image_topic_input"
        print rospy.has_param(self._input_image_topic)
        if rospy.has_param(self._input_image_topic):
            input_image_topic = rospy.get_param(self._input_image_topic)
            self.image_sub = rospy.Subscriber(input_image_topic, Image, self.callback)


    def callback(self,data):
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
            ok, frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Read video
        video = cv2.VideoCapture(1)

        # Exit if video not opened.
        if not video.isOpened():
            print "Could not open video"
            sys.exit()

        if not ok:
            print "Cannot read video file"
            sys.exit()

        bbox = rospy.Subscriber("floats", numpy_msg(Floats), callback)
        ok = tracker.init(frame, bbox)

        while True:
            # Read a new frame
            ok, frame = video.read()
            if not ok:
                break
            # Start timer
            timer = cv2.getTickCount()
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

            if ok:
                # Tracking success
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
            else :
                # Tracking failure
                cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            # Display tracker type on frame
            cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
        
            # Display FPS on frame
            cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);

            # Display result
            cv2.imshow("Tracking", frame)

    # # # Read video
    # # video = cv2.VideoCapture(1)

    # # # Exit if video not opened.
    # # if not video.isOpened():
    # #     print "Could not open video"
    # #     sys.exit()

    # # # Read first frame.
    # # ok, frame = video.read()
    # # if not ok:
    # #     print 'Cannot read video file'
    # #     sys.exit()
    
    # # Define an initial bounding box
    # # bbox = (287, 23, 86, 320)

    # # Uncomment the line below to select a different bounding box
    # bbox = cv2.selectROI(frame, False)

    # # Initialize tracker with first frame and bounding box
    # ok = tracker.init(frame, bbox)

    # while True:
    #     # Read a new frame
    #     ok, frame = video.read()
    #     if not ok:
    #         break
        
    #     # Start timer
    #     timer = cv2.getTickCount()

    #     # Update tracker
    #     ok, bbox = tracker.update(frame)

    #     # Calculate Frames per second (FPS)
    #     fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

    #     # Draw bounding box
    #     if ok:
    #         # Tracking success
    #         p1 = (int(bbox[0]), int(bbox[1]))
    #         p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
    #         cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    #     else :
    #         # Tracking failure
    #         cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    #     # Display tracker type on frame
    #     cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
    
    #     # Display FPS on frame
    #     cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);

    #     # Display result
    #     cv2.imshow("Tracking", frame)


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
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        sys.exit()