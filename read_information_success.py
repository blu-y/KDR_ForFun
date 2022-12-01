#174fb8deea220eccedbd981c1779e62f84452ba6c94219a276e6a5d2c2e6a07b
import airsim
import numpy as np
from math import asin, acos, degrees, atan2
import cv2
import time

c = airsim.MultirotorClient()
c.enableApiControl(True)
c.simGetVehiclePose()
c.takeoffAsync().join()

'''
data = list(c.simGetObjectPose('KDR_Ring_2'))
data = list(data[0])
c.moveToPositionAsync(data[0], data[1], data[2], 7)
'''

with open("drone_ring_list.txt", "r") as f:
    ring = f.readlines()  # ['첫 번째 줄\n', '두 번째 줄\n', '세 번째 줄'] 저장
    ring = list(map(lambda s: s.strip(), ring))

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

angle_list = [0] * 72
for i in range(0, 72):
    backward = list(c.simGetObjectPose(ring[i]))
    backward = list(backward[0])
    try:
        forward = list(c.simGetObjectPose(ring[i+1]))
        forward = list(forward[0])
    except:
        forward = list(c.simGetObjectPose(ring[0]))
        forward = list(forward[0])
    dist = ((forward[0] - backward[0]) ** 2 + (forward[1] - backward[1]) ** 2) ** 0.5
    theta = acos((forward[0] - backward[0]) / dist)
    angle_list[i] = degrees(theta)
    #theta = 180 - (asin((forward[1] - backward[1]) / dist)) * 180 / np.pi
    #angle_list[i] = theta
    #if (forward[1] - backward[1]) <= 0:
    #    angle_list[i] -= 180

def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1) * 180 / np.pi
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2) * 180 / np.pi
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4) * 180 / np.pi
    return [yaw, pitch, roll]

def fly(a, b, v):
    for i in range(a, b):
        data = list(c.simGetObjectPose(ring[i]))
        #print(yaw, pitch, roll)
        data = list(data[0])
        c.moveToPositionAsync(data[0], data[1], data[2], v).join()

        #c.rotateToYawAsync(angle_list[i+1])
        try:
            next = list(c.simGetObjectPose(ring[i+1]))
        except:
            next = list(c.simGetObjectPose(ring[0]))
        [yaw, pitch, roll] = quaternion_to_euler(list(next[1]))
        c.rotateToYawAsync(yaw-90)
        time.sleep(0.3)
        print(i, yaw-90, angle_list[i+1])
        seg, dep = get_frame(c)
        name = "img/dep" + str(i) + '.jpg'
        cv2.imwrite(name, dep)
        if i == b: break


fly(0, 2, 7.5)
fly(2, 5, 5.5)
fly(5, 11, 6.0)
fly(11, 12, 4.5)
fly(12, 18, 5.2)
fly(18, 24, 6.5)
fly(24, 30, 5.0)
fly(30, 40, 7.0)
fly(40, 43, 5.5)
fly(43, 45, 3.5)
fly(45, 47, 6.5)
fly(47, 49, 5.5)
fly(49, 61, 7.5)
fly(61, 64, 6.5)
fly(64, 70, 4.5)
fly(70, 72, 4.0)
c.moveByVelocityAsync(-7.0, 0, -1.0, 1).join()
#c.moveByVelocityAsync(0.0, 0, 0.0, 1).join()
c.landAsync().join()
