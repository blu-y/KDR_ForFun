
import setup_path
import airsim
import WP_Parser
import os
import sys
import cv2
import time
from math import cos, sin, degrees
import matplotlib.pyplot as plt
import numpy as np

MAX_dist = 30

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

while(1):
    dep_, = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),])
    dep = airsim.list_to_2d_float_array(dep_.image_data_float, dep_.width, dep_.height)
    dep = dep.reshape(dep_.height, dep_.width, 1)
    dep = np.interp(dep, (0, MAX_dist), (0, 255)).astype(np.uint8)
    seg_ = client.simGetImage("0", airsim.ImageType.Segmentation)
    seg = cv2.imdecode(airsim.string_to_uint8_array(seg_), cv2.IMREAD_UNCHANGED)
    seg[seg[:,:,0]!=83] = 0
    seg[:,:,3] = 255
    dep[seg[:,:,0]==0] = 255
    cv2.imshow("seg", seg)
    cv2.imshow("depth", dep)
    if cv2.waitKey(1) == 27: break
    # plt.figure("seg")
    # plt.imshow(seg)
    # plt.figure("depth")
    # plt.imshow(dep)
    # plt.show()