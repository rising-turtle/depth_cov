#!/usr/bin/python
# Oct. 21 2017, He Zhang, hxzhang1@ualr.edu 
# 
# receive a list of points (x,y), and then polyfit a curve. For each point in this curve, we compute its gradient orientation. The averaged gradient direction points toward the head orientation. 
#

import numpy as np
import cv2 
import matplotlib.image as mpimg

def point_pipeline(px, py):
    line_fit = np.polyfit(px, py, 2)
    # Generate x and y values for plotting
    plotx = np.linspace(0, 199, 200)
    ploty = line_fit[0]*plotx**2 + line_fit[1]*plotx + line_fit[2]

    gx = 2*line_fit[0]*plotx + line_fit[1]

    # 3 estimate angle 
    # gradient value 
    yt = 1./np.sqrt(np.square(gx) + 1)
    xt = -gx * yt
    angle = np.arcsin(xt) * 180./np.pi
    return np.mean(angle)

def image_pipeline(img):
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    return point_pipeline(nonzerox, nonzeroy)
    
if __name__ == '__main__':
    fname = '141.png'
    img = mpimg.imread(fname)
    angle = image_pipeline(img)
    print 'angle: ',angle 




