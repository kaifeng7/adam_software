
import rospy
import os
import sys

if __name__ == '__main__':
    rospy.init_node('video')
    cmd = 'gst-launch-1.0 -v v4l2src device=/dev/video1 ! nvvidconv ! omxh264enc ! \'video/x-h264, stream-format = byte-stream, width = 800, height = 600, framerate = 30/1\' ! h264parse ! rtph264pay ! udpsink host='+sys.argv[1]+' port=5000'
