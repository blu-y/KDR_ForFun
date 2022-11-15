
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

def get_frame(client, MAX_dist=30):
    dep_, = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),])
    dep = airsim.list_to_2d_float_array(dep_.image_data_float, dep_.width, dep_.height)
    dep = dep.reshape(dep_.height, dep_.width, 1)
    dep = np.interp(dep, (0, MAX_dist), (0, 255)).astype(np.uint8)
    seg_ = client.simGetImage("0", airsim.ImageType.Segmentation)
    seg = cv2.imdecode(airsim.string_to_uint8_array(seg_), cv2.IMREAD_UNCHANGED)
    seg[seg[:,:,0]!=83] = 0
    seg[:,:,3] = 255
    dep[seg[:,:,0]==0] = 255
    return seg, dep

def os(old, new):
    dist = np.sqrt((new.x-old.x)**2+(new.y-old.y)**2+(new.z-old.z)**2)
    x = (new.x-old.x)/dist
    y = (new.y-old.y)/dist
    z = (new.z-old.z)/dist
    return x, y, z

# Desired Speed in m/s
speed  = 5

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
client.moveByVelocityZAsync(-5, 0, -4, 3)

# Initialize
print("Initializing")
way_points = []

# Create WayPoint Parser
WPP = WP_Parser.WP_Data(docs, None)

class coordinate():
    def __init__(self, old):
        self.old = old
        self.new = old
    def read(self, con):
        self.new = WPP.ReadData(con, "WP")

        (self.new.x - self.old.x)

        self.X()
        self.Y()
        self.Z()
        self.DX()
        self.DY()
        return self.new
    def X(self):
        
        self.x = float(self.new.X)/100
    def Y(self):
        
        self.y = float(self.new.X)/100
    def Z(self):

        self.z = -float(self.new.X)/100
    def DX(self):
        self.dx = cos(degrees(int(self.new.ZR)))
    def DY(self):
        self.dy = sin(degrees(int(self.new.ZR)))

# If Found WayPoint Data
if WPP.IsFileOpen:
    print("GOGO")
    coor = coordinate(WPP.ReadData(1, "WP"))
    # LOOP
    seg, dep = get_frame(client)
    con = 1
    while(1):

        # Ignore WaPoint #17
        if con == 17: con+=1
        # Get WayPoint Data index = con
        new = coor.read(con)
        # Proceed If Next WayPoint Exist
        if new:
            con += 1
            way_points.append([int(new.Xoff), int(new.Yoff), int(new.Zoff)*-1])
            client.moveToPositionAsync(coor.x, coor.y, coor.z, speed).join()
            client.rotateToYawAsync(int(new.ZR)).join()
            coor.old = coor.new
            #client.moveByVelocityZAsync(coor.dx, coor.dy, coor.z, 0.3)
        else:
            break
        time.sleep(2)
        seg, dep = get_frame(client)
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
                            vx, vy, z, duration

'''