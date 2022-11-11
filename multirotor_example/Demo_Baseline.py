
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

# Desired Speed in m/s
desired_speed  = 5
MAX_dist = 30

# WayPoints Data Path
docs = os.path.join(sys.path[0], "WayPoints.txt")

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# TakeOff
print("Taking Off")
client.takeoffAsync().join()
print("Initializing")
way_points = []


# Create WayPoint Parser
WPP = WP_Parser.WP_Data(docs, None)

# If Found WayPoint Data
if WPP.IsFileOpen:
    print("GOGO")

    # LOOP
    con = 1
    while(1):

        # Ignore WaPoint #17
        if con == 17: con+=1
        # Get WayPoint Data index = con
        new = WPP.ReadData(con, "WP")

        # Proceed If Next WayPoint Exist
        if new:
            con += 1
            way_points.append([int(new.Xoff), int(new.Yoff), int(new.Zoff)*-1])
            client.moveToPositionAsync(float(new.X)/100, float(new.Y)/100, float(new.Z)/100*-1, 5).join()
            client.rotateToYawAsync(int(new.ZR)).join()
            client.moveByVelocityZAsync(cos(degrees(int(new.ZR))), sin(degrees(int(new.ZR))), float(new.Z)/100*-1, 0.3)

        else:
            break
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

    # Return To First WayPoint
    new = WPP.ReadData(1, "WP")
    way_points.append([int(new.Xoff), int(new.Yoff), int(new.Zoff)*-1])
    client.moveToPositionAsync(int(new.Xoff), int(new.Yoff), int(new.Zoff)*-1, 5).join()

else:
    print("Failed To open WayPoint File")
client.hoverAsync().join()
client.landAsync().join()


'''
client.moveByVelocityZAsync(-10, 0, -5, 3) 
                            x, vy, z, duration

'''