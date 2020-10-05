#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import numpy as np
import cv2
import time

from features_extraction import features_extraction

DEBUG = False
class points_extraction(features_extraction):

    def __init__(self, intrinsic):
        
        features_extraction.__init__(self)

        # Vector of the visual features # OBSOLETE?!
        self.s = np.array([])
        
        # Setup of the opencv's SimpleBlobDetector parameters. 
        params = cv2.SimpleBlobDetector_Params()

        # For a description of the parameters, see https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        params.blobColor = 0; # 0 -> black 

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 10000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.5#0.750

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.01#85

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.1
        params.maxInertiaRatio = 1.0

        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            self.detector = cv2.SimpleBlobDetector(params)
        else: 
            self.detector = cv2.SimpleBlobDetector_create(params)
        
        # Color detection parameters, using HSV color space
        # Hue ranges in [0,179], green is around 60
        self.low_H = 30
        self.high_H = 90
        # Saturation ranges in [0,255]
        self.low_S = 50
        self.high_S = 255
        # Value ranges in [0,255]
        self.low_V = 30
        self.high_V = 255

        # Kernel used in the 'Opening' operation
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10,10), (1, 1))
        
        # Initialization of flags and counters
        self.track = False
        self.counter_not_tracking = 10

        # Initialize the center of the visual pattern with the center of the image
        self.xm_old = intrinsic['cx']
        self.ym_old = intrinsic['cy']  

        self.blobs = np.array([])
        
        self.target = np.array([])
        self.set_dummy_target(intrinsic)

        self.window_thresholding_name = "Thresholded image"
        #self.first = True
        
        # Computation time variables
        self.counter = 0
        self.time_elapsed_arr = np.array([])
    
    def set_dummy_target(self,intrinsic):

        offset = 50
        dummy_array = np.array([
                    intrinsic['cx']-offset, intrinsic['cy']-offset, intrinsic['cx']+offset, intrinsic['cy']-offset,
                    intrinsic['cx']+offset, intrinsic['cy']+offset, intrinsic['cx']-offset, intrinsic['cy']+offset])
        self.set_target(dummy_array)

    def set_target(self,arr_in):
        self.target = arr_in

    def run(self, img):

        time_start = time.time()
        
        #img_blurred = cv2.medianBlur(img,5)

        #if DEBUG:
        #    cv2.imshow('Blur',img_blurred)   
        
        # Color detection using HSV color space
        frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Trackbars do not seem to work properly
        #if self.first :
        #    cv2.namedWindow(self.window_thresholding_name)
        #    self.first = False
        # #cv2.createTrackbar("lowH", self.window_thresholding_name, self.low_H, high_H, lambda x : x)
        
        # Apply a thresholding on the image and negate to get an image with black blobs
        threshold_mask = cv2.inRange(frame_HSV, (self.low_H, self.low_S, self.low_V), (self.high_H, self.high_S, self.high_V))
        
        if DEBUG:
            cv2.imshow('Mask',threshold_mask)
        
        # Opening operation (erosion + dilation) to remove salt-and-pepper noise
        threshold_mask = cv2.morphologyEx(threshold_mask, cv2.MORPH_OPEN, self.kernel)
        
        if DEBUG:
            cv2.imshow('Opening',threshold_mask)
        
        # White blobs -> black blobs
        thresholded_img = cv2.bitwise_not(threshold_mask)

        if DEBUG:
            cv2.imshow(self.window_thresholding_name, thresholded_img)
        
        # Set-up the detector with default parameters.
        keypoints = self.detector.detect(thresholded_img)
        
        if len(keypoints)==4:

            xx = [k.pt[0] for k in keypoints]
            yy = [k.pt[1] for k in keypoints]
            rr = [0.5*k.size  for k in keypoints]

            # Compute the angle w.r.t the center of visual pattern
            xm = np.sum([np.max(xx),np.min(xx)])/2
            ym = np.sum([np.max(yy),np.min(yy)])/2
            xx4angles = [x - xm for x in xx]
            yy4angles = [y - ym for y in yy]
            angles = np.arctan2(yy4angles,xx4angles)
            
            # Compute the distance travelled by the center of the visual pattern
            distance = np.linalg.norm(np.array([xm,ym])-np.array([self.xm_old,self.ym_old]),2)
            
            # Keep memory of the previous visual pattern's center
            self.xm_old = xm
            self.ym_old = ym

            self.blobs = np.array([(angles[0], (xx[0],yy[0]), rr[0]),
                    (angles[1], (xx[1],yy[1]), rr[1]),
                    (angles[2], (xx[2],yy[2]), rr[2]),
                    (angles[3], (xx[3],yy[3]), rr[3])],
                    dtype=[('angle','f4'), ('center',np.float64, (2,)), ('ray','f4')])
            
            # If you did not loose the detection at the previous iteration and the pattern did not move much,
            # track the blobs with the same id than before, decided on the base of the euclidean distance
            if self.track == True and distance < 50:
                blobs_aux = np.copy(self.blobs)
                for (i,f) in enumerate(self.blobs):
                    dist = np.linalg.norm(f['center']-self.blobs_old['center'],2,1)
                    idx = np.argmin(dist)
                    blobs_aux[idx] = f
                self.blobs = blobs_aux
            # Otherwise, sort the blobs by their angle
            else:
                self.blobs = np.sort(self.blobs,None,'quicksort','angle') 

            self.blobs_old = self.blobs

            self.track = True
            self.counter_not_tracking = 0
            color_track = (255,50,0)

        else:

            if self.counter_not_tracking < 10:
                self.track = True
                self.counter_not_tracking +=1
                color_track = (255,255,0)
            else:
                self.track = False
                color_track = (255,255,230)

        # Draw the blobs on the image and fill the features vector 's'
        self.s = np.array([])
        #self.s = np.array([b['center'] for b in self.blobs])  
        for (i,f) in enumerate(self.blobs):
            
            x = int(f['center'][0])
            y = int(f['center'][1])
            ray = int(f['ray'])

            self.s = np.append(self.s, x)
            self.s = np.append(self.s, y)
            
            pt = (x,y)
            #cv2.circle(gray_c, pt, ray, color_track,  1, cv2.LINE_AA)
            cv2.circle(img, pt, 8, color_track, -1, cv2.LINE_AA)
            
            pt2 = (x-4,y+4)#(x+15,y+15)
            cv2.putText(img, str(i), pt2, cv2.FONT_HERSHEY_PLAIN, 0.8, (255,190,120), 1, cv2.LINE_AA)        

            if self.target.size>2*i+1:
                x_red = int(self.target[2*i])
                y_red = int(self.target[2*i+1])
                red_color = (0,10,255)
                cv2.circle(img, (x_red,y_red), 8, red_color, 1, cv2.LINE_AA)
                cv2.putText(img, str(i), (x_red-4,y_red+4), cv2.FONT_HERSHEY_PLAIN, 0.8, red_color, 1, cv2.LINE_AA)    

        # Measuring time
        time_elapsed = (time.time() - time_start)
        self.time_elapsed_arr = np.append(self.time_elapsed_arr, time_elapsed) 
        
        if self.counter >= 100:
            self.counter = 0
            average_time = round(np.average(self.time_elapsed_arr),3)
            average_freq = round(1/average_time,1)
            print('Image processing computation time: ' + str(average_time) + " s (" + str(average_freq) + ' Hz)')
            self.time_elapsed_arr = np.array([])
        self.counter += 1
            
        self.image = img

    def print_features(self):
        
        print('--------------------------------------------------')
        for k in range(0,len(self.s),2):
            print('s_'+str(int(0.5*k))+': ('+str(self.s[k])+', '+str(self.s[k+1])+')')
