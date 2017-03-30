#!/usr/bin/env python

"""  
This is a python script that helps with quick research into methods for the
person calibrator.  Below are some research findings about methods:

Making Headway...

Using SLIC with 10's of superpixels, and a connectivity (nc) of around 10 results
in segmenting entire legs, especially if the legs are of uniform color.  
I.e. better for lydia1.bag, as opposed to rocco1.bag Using this method of very 
few superpixels, it was possible for Lydia in the shadows to be fully segmented
just by superpixels, without the need for a second step of clustering by Meanshift.
This is better, because SLIC does do clustering with space in mind.

The method of cropping the image to only the center third helps, especially when
there are walls around or other objects that are big and contiguous.
Just adjust the number of superpixels appropriately.  Additional idea could be
using the 1/3 method, and a way to filter out floor (by creating a labeled dataset
of what is floor surfaces at Olin look like.  I was thinking we could drive around
Olin and get the bottom the center part of the bottom 1/5 of the image and call
that floor, for a segmentation algorithm to learn.  Then, most of the rest of the
image is legs, in which case we can get reasonable Color Histograms to find
upper/lowerbound limits for color thresholds for the HSV Person Follower to take.

Issues...

Segmenting Rocco's entire legs will be hard.  There's a lot of wrinkles and different
tones to his pants, which are very close to the same color as the walls.
Because of the wrinkles, the superpixels are not predictably going to be large,
rectangular segments corresponding to "thighs" and "calves". Working in the
10's of superpixels fails for Rocco.

  
""" 

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from skimage.segmentation import slic, mark_boundaries
from sklearn.cluster import MeanShift, estimate_bandwidth
import superpixelation

class PersonCalibrator(object):


    def __init__(self):
        rospy.init_node('person_calibrator')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('video_window')
        rospy.Subscriber("/image_raw", Image, self.process_image)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    	width = self.cv_image.shape[1]
	self.cv_image = self.cv_image[:,int(0.33*width):int(0.66*width)]
	gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        
        segments = slic(self.cv_image, n_segments=10, compactness=50)
        edit = mark_boundaries(self.cv_image, segments) 

        X = superpixelation.get_mean_values(self.cv_image, segments) 
	
        # The following bandwidth can be automatically detected using
	bandwidth = estimate_bandwidth(X, quantile=0.5)
	ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
	ms.fit(X)
	labels = ms.labels_
	cluster_centers = ms.cluster_centers_
	labels_unique = np.unique(labels)
	n_clusters_ = len(labels_unique)
	print("number of estimated clusters : %d" % n_clusters_)	

         
	final_segmentation = superpixelation.get_final_segmentation(gray_image, segments, labels, n_clusters_)

        for i, mask in enumerate(final_segmentation):
	    cv2.imshow(str(i), mask.astype(np.uint8)*255)
                                                 
        # cv2.imshow('video_window', self.cv_image)
        cv2.imshow('edit', edit)
        cv2.waitKey(5)

    def run(self):
        """ The main run loop"""
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = PersonCalibrator()
    node.run()
