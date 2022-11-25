
import setup_path
import airsim
import WP_Parser
import os
import sys
import cv2
import time
from math import cos, sin
from math import radians as rad
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
    dep[dep>100] = 255
    return seg, dep

def get_offset(img):
    _, bin = cv2.threshold(img, 0, 255, cv2.THRESH_OTSU)
    cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(bin)
    try:
        if len(stats)>0: 
            dst = np.zeros(img.shape)
            dy = stats[-1][0] + stats[-1][2]/2 - 256/2
            dz = stats[-1][1] + stats[-1][3]/2 - 144/2
            cv2.rectangle(dst, (stats[-1][0], stats[-1][1], stats[-1][2], stats[-1][3]), (255, 255, 255))
        else: print('fail')
    except: 
        print('no list')
        return 0, 0, dst
    return dy, dz, dst

# Desired Speed in m/s
desired_speed  = 5
cp = 0.5
C = 0.15
Cz = 0.1

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
    old = WPP.ReadData(0, "WP")
    con = 1
    while(1):

        # Ignore WaPoint #17
        if con == 17: con+=1
        # Get WayPoint Data index = con
        new = WPP.ReadData(con, "WP")
        try: next = WPP.ReadData(con+1, "WP")
        except: pass
        # Proceed If Next WayPoint Exist
        if new:
            x = int(old.Xoff)+(int(new.Xoff)-int(old.Xoff))*cp
            y = int(old.Yoff)+(int(new.Yoff)-int(old.Yoff))*cp
            client.moveToPositionAsync(x, y, int(new.Zoff)*-1, 5).join()
            time.sleep(5)
            seg, dep = get_frame(client)
            dy, dz, dst = get_offset(dep)
            #plt.imshow(dep)
            #plt.show()
            cv2.imshow("seg", seg)
            cv2.imshow("depth", dep)
            cv2.imshow("dst", dst)
            cv2.imwrite(str(con)+"dep.png", dep)
            cv2.waitKey(1)
            dx = cos(rad(int(next.ZR)))*dy*C
            dy = sin(rad(int(next.ZR)))*dy*C
            dz = -dz*Cz
            print(dx, dy, dz)
            # 사진으로 계산 y,z(상대) 차이를 계산하고, ZR로 절대 x_o, y_o, z_o를 계산

            con += 1
            '''
            print(new.X)
            print(new.Y)
            print(new.Z)
            print(new.Xoff)
            print(new.Zoff)
            print(new.Yoff, "\n")
            '''
            way_points.append([int(new.Xoff), int(new.Yoff), int(new.Zoff)*-1])
            client.moveToPositionAsync(int(new.Xoff)+dx, int(new.Yoff)+dy, (int(new.Zoff)+dz)*-1, 5).join()
            client.rotateToYawAsync(int(next.ZR)).join()
            old = new

        else:
            break

    # Return To First WayPoint
    new = WPP.ReadData(1, "WP")
    way_points.append([int(new.Xoff), int(new.Yoff), int(new.Zoff)*-1])
    client.moveToPositionAsync(int(new.Xoff), int(new.Yoff), int(new.Zoff)*-1, 5).join()
   
else:
    print("Failed To open WayPoint File")
   
client.hoverAsync().join()
client.landAsync().join()