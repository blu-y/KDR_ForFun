#174fb8deea220eccedbd981c1779e62f84452ba6c94219a276e6a5d2c2e6a07b
import airsim
import numpy as np
from math import asin, acos, degrees, atan2, cos, sin, tan, radians
import cv2
import time
from sympy import Symbol, solve
import copy

c = airsim.MultirotorClient()
c.confirmConnection()
c.enableApiControl(True)
c.armDisarm(True)
c.simGetVehiclePose()

with open("drone_ring_list.txt", "r") as f:
    ring = f.readlines()  # ['첫 번째 줄\n', '두 번째 줄\n', '세 번째 줄'] 저장
    ring = list(map(lambda s: s.strip(), ring))
# Ring 73개 => 72번째 Ring뒤에 1번째 Ring 추가함
ring.append(ring[0])

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

def Euler(q):
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

def dist_theta():
    distance = [0] * len(ring)
    angle_list = [0] * len(ring)
    for i in range(0, len(ring) - 1):
        backward = list(c.simGetObjectPose(ring[i]))
        backward = list(backward[0])
        forward = list(c.simGetObjectPose(ring[i+1]))
        forward = list(forward[0])
        
        dist = ((forward[0] - backward[0]) ** 2 + (forward[1] - backward[1]) ** 2) ** 0.5
        distance[i] = dist
        
        theta = 180 - (asin((forward[1] - backward[1]) / dist)) * 180 / np.pi
        angle_list[i] = theta
        if (forward[0] - backward[0]) >= 0 and (forward[1] - backward[1]) >= 0:
            angle_list[i] -= 90
        elif (forward[0] - backward[0]) > 0 and (forward[1] - backward[1]) < 0:
            angle_list[i] += 90
    return angle_list, distance
dist_theta = list(dist_theta())
angle_list, distance = dist_theta[0], dist_theta[1]
del angle_list[72]
del distance[72]


def about_ring():
    x, y, z = [0]*72, [0]*72, [0]*72
    for i in range(72):
        data = list(c.simGetObjectPose(ring[i]))
        data = list(data[0])
        x[i], y[i], z[i] = data[0], data[1], data[2]
    return [x, y, z]
ring_information = list(about_ring())
x_coor , y_coor, z_coor = ring_information[0], ring_information[1], ring_information[2]
x_coor.append(0.0)
y_coor.append(0.0)
z_coor.append(-1.0)
# Ring의 X, Y, Z좌표 도착지점까지 총 73개

def yaw():
    Yaw = [0] * 72
    for i in range(72):
        data = list(c.simGetObjectPose(ring[i + 1]))
        [yaw, pitch, roll] = Euler(list(data[1]))
        Yaw[i] = yaw
    return Yaw
yaw = yaw()
first_ring = list(c.simGetObjectPose(ring[0]))
[yaw0, pitch0, roll0] = Euler(list(first_ring[1]))
yaw.append(yaw0)

# def my_yaw():
#     Yaw = [0] * 72
#     for i in range(72):
#         data = list(c.simGetObjectPose(ring[i + 1]))
#         [yaw, pitch, roll] = Euler(list(data[1]))
#         Yaw[i] = yaw
#         if 90 < yaw < 180 :
#             Yaw[i] = 90 - (180 - yaw)
#         elif -180 < yaw < -90:
#             Yaw[i] = 90 + (180 + yaw)
#         elif -90 < yaw < 0:
#             Yaw[i] = 180 + (180 + yaw)
#         elif -180 < yaw < -90:
#             Yaw[i] = 90 + (180 + yaw)

def delta_yaw():
    delta_yaw = [0] * 72
    for i in range(72):
        delta_yaw[i] = abs(yaw[i+1] - yaw[i])
        if delta_yaw[i] >= 180:
            delta_yaw[i] = abs(360 - abs((yaw[i+1] - yaw[i])))
    return delta_yaw
delta_yaw = delta_yaw()

def max_delta_yaw():
    max_delta = 0
    for i in range(72):
        if delta_yaw[i] > max_delta:
            max_delta = delta_yaw[i]
    return max_delta
max_delta_yaw = max_delta_yaw()
print(delta_yaw)
print(max_delta_yaw)


#for i in range(72):
#    if delta_yaw_abs[i] != delta_yaw_raw[i]:
#        print(i)
#def delta_delta_yaw():

#def curvature_center(x1, y1, x2, y2):


def move(velocity):
    for i in range(73):
        c.moveToPositionAsync(x_coor[i], y_coor[i], z_coor[i], velocity)

sumation = 0
for i in range(72):
    sumation += distance[i]


#print(sumation/72)
#c.takeoffAsync().join()
