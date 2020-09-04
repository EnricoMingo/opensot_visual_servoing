#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2

class features_extraction:

    def __init__(self):
        pass

    def get_image(self):
        return self.image

    def get_features(self):
        return self.s

class points_extraction(features_extraction):

    def __init__(self):
        
        features_extraction.__init__(self)

        self.blobs = np.array([])

        # Vector of the visual features
        self.s = np.array([])
        
        # Setup of the opencv's SimpleBlobDetector parameters. 
        self.params = cv2.SimpleBlobDetector_Params()

        # For a description of the parameters, see https://www.learnopencv.com/blob-detection-using-opencv-python-c/
        self.params.blobColor = 0; # 0 -> black 

        # Change thresholds
        self.params.minThreshold = 0
        self.params.maxThreshold = 255

        # Filter by Area.
        self.params.filterByArea = True
        self.params.minArea = 50
        self.params.maxArea = 10000

        # Filter by Circularity
        self.params.filterByCircularity = True
        self.params.minCircularity = 0.5#0.750

        # Filter by Convexity
        self.params.filterByConvexity = True
        self.params.minConvexity = 0.01#85

        # Filter by Inertia
        self.params.filterByInertia = True
        self.params.minInertiaRatio = 0.1
        self.params.maxInertiaRatio = 1.0

        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            self.detector = cv2.SimpleBlobDetector(self.params)
        else: 
            self.detector = cv2.SimpleBlobDetector_create(self.params)
        
        # Initialization of flags and counters
        self.track = False
        self.counter_not_tracking = 10

    def run(self,img):

        # Convert in gray scale image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # And get a version in colors (where to show the colored features)
        gray_c = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Set-up the detector with default parameters.
        keypoints = self.detector.detect(gray_c)

        
        if len(keypoints)==4:

            xx = [k.pt[0] for k in keypoints]
            yy = [k.pt[1] for k in keypoints]
            rr = [0.5*k.size  for k in keypoints]

            # Compute the angle w.r.t the center of visual pattern
            xm = np.sum([np.max(xx),np.min(xx)])/2
            ym = np.sum([np.max(yy),np.min(yy)])/2
            xx4angles = [x - xm for x in xx] # 0.5*width_frame
            yy4angles = [y - ym for y in yy] #0.5*height_frame 
            angles = np.arctan2(yy4angles,xx4angles)
            
            #TODO: Fix this with arbitray number of features (not necessarily 4)
            self.blobs = np.array([(angles[0], (xx[0],yy[0]), rr[0]),
                    (angles[1], (xx[1],yy[1]), rr[1]),
                    (angles[2], (xx[2],yy[2]), rr[2]),
                    (angles[3], (xx[3],yy[3]), rr[3])],
                    dtype=[('angle','f4'), ('center',np.float64, (2,)), ('ray','f4')])
            
            # If you lost the detection at the previous iteration, sort the blob by the angle
            # otherwise track the blobs with the same id than before, decided on the base of the euclidean distance
            if self.track == True:
                blobs_aux = np.copy(self.blobs)
                for (i,f) in enumerate(self.blobs):
                    dist = np.linalg.norm(f['center']-self.blobs_old['center'],2,1)
                    idx = np.argmin(dist)
                    blobs_aux[idx] = f
                self.blobs = blobs_aux
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
        for (i,f) in enumerate(self.blobs):
            
            x = int(f['center'][0])
            y = int(f['center'][1])
            ray = int(f['ray'])

            self.s = np.append(self.s, x)
            self.s = np.append(self.s, y)
            
            pt = (x,y)
            cv2.circle(gray_c, pt, ray, color_track,  1, cv2.LINE_AA)
            cv2.circle(gray_c, pt,   2, color_track, 10, cv2.LINE_AA)
            
            pt2 = (x+15,y+15)
            cv2.putText(gray_c, str(i), pt2, cv2.FONT_HERSHEY_PLAIN, 0.8, color_track, 1, cv2.LINE_AA)        

        self.image = gray_c

    def print_features(self):
        
        print('--------------------------------------------------')
        for k in range(0,len(self.s),2):
            print('s_'+str(int(0.5*k))+': ('+str(self.s[k])+', '+str(self.s[k+1])+')')
