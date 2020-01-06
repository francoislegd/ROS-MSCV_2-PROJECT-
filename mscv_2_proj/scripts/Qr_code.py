#!/usr/bin/env python

""" cv_bridge_demo.py - Version 1.1 2013-12-20

    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import webbrowser
import time
import pyzbar.pyzbar as pyzbar
import math
import urllib
import urllib2
from urlparse import urlparse

class cvBridgeDemo():
    def __init__(self):
        last_url = str()
        self.node_name = "cv_bridge_demo"
        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        self.rgb_image = None
            
        # Create the OpenCV display window for the RGB image
        self.rgb_window_name = self.node_name
        cv2.namedWindow(self.rgb_window_name, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.rgb_window_name, 25, 75)
                
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("input_rgb_image", Image)
        
        # Subscribe to the camera image and depth topics and set he appropriate callbacks
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        
        
        rospy.loginfo("Ready.")
        
        while not rospy.is_shutdown():
            if not self.rgb_image is None:
                cv2.imshow(self.rgb_window_name, self.rgb_image)
                      
                # Process any keyboard commands
                self.keystroke = cv2.waitKey(5)
                         
                if self.keystroke != -1:
                    cc = chr(self.keystroke & 255).lower()
                    if cc == 'q':
                        # The user has press the q key, so exit
                        rospy.signal_shutdown("User hit q key to quit.")
                
            
    def image_callback(self, ros_image):
	last_url = str()
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Process the frame using the process_image() function
        self.rgb_image, self.last_url = self.process_image(frame, last_url)        
       
          
    def process_image(self, frame, last_url):
      
  
        last_url = str()
		
    	decodedObjects = pyzbar.decode(frame)

   	for obj in decodedObjects:

	    	url = str(obj.data)	    	
		url = str.encode(url, encoding= 'utf-8') 
	    	url_parse = urlparse(url)


	    	if url != last_url and url_parse.scheme and url_parse.netloc:
			print("Got Data:", url)
			request = urllib2.Request(url)		
			response = urllib2.urlopen(url) # open link in the background
               		html = response.read
			last_url = url
			#last_url = webbrowser.open(last_url) #.open_new_tab
		else:
					
			continue					
				
		
       	return frame, last_url
	
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
																							 							
    
