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
    ring.append(ring[0])
# Ring 73개
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
    distance = [0] * 72
    angle_list = [0] * 72
    for i in range(0, 72):
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
    angle_list.append(0)
    distance.append(10)
    return angle_list, distance
dist_theta = list(dist_theta())
angle_list, distance = dist_theta[0], dist_theta[1]

def about_ring():
    x, y, z = [0]*72, [0]*72, [0]*72
    for i in range(72):
        data = list(c.simGetObjectPose(ring[i]))
        data = list(data[0])
        x[i], y[i], z[i] = data[0], data[1], data[2]
    x.append(0.0)
    y.append(0.0)
    z.append(-1.0)
    return [x, y, z]
about_ring = list(about_ring())
x_coor , y_coor, z_coor = about_ring[0], about_ring[1], about_ring[2]
# Ring의 X, Y, Z좌표 도착지점까지 총 73개

def yaw():
    Yaw = [0] * 72
    for i in range(72):
        data = list(c.simGetObjectPose(ring[i + 1]))
        [yaw, pitch, roll] = Euler(list(data[1]))
        Yaw[i] = yaw
    first_ring = list(c.simGetObjectPose(ring[0]))
    [yaw0, pitch0, roll0] = Euler(list(first_ring[1]))
    Yaw.append(yaw0)
    return Yaw
yaw = yaw()

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

def velocity_distance():
    max_velocity = 9.5
    min_velocity = 5.5
    max_distance = max(distance)
    min_distance = min(distance)
    delta_v = max_velocity - min_velocity
    delta_dist = max_distance - min_distance
    final_v = [0] * 72
    for i in range(72):
        quality = (distance[i] - min_distance)/delta_dist
        final_v[i] = quality * delta_v + min_velocity
        if delta_yaw[i] >= 30 and 10 <distance[i] <= 14:
            final_v[i] -= 1.2
        elif delta_yaw[i] >= 30 and distance[i] <= 10:
            final_v[i] -= 2
    final_v.append(10)
    return final_v
velocity_distance = velocity_distance()
# 73 개

def time_sleep():
    maximum = 0.4
    minimum = 0.2
    delta = maximum - minimum
    time = [0] * 72
    for i in range(72):
        quality = delta_yaw[i] / 50
        time[i] = quality * delta + minimum
    return time
    time.append(0)
time_sleep = time_sleep()
# 73개

def move():
    c.takeoffAsync().join()
    velocity_distance.insert(0, 9)
    for i in range(73):
        c.moveToPositionAsync(x_coor[i], y_coor[i], z_coor[i], velocity_distance[i]).join()
        c.rotateToYawAsync(yaw[i]-90)
        time.sleep(time_sleep[i])
        if i >= 67 :
            c.moveToPositionAsync(x_coor[i], y_coor[i], z_coor[i], 4).join()
            c.rotateToYawAsync(yaw[i]-90)
            time.sleep(time_sleep[i]) 

#move()
#velocity_distance.insert(0, 9)
for i in range(73):
    print(i, f"{x_coor[i]:.3f}, {y_coor[i]:.3f}, {z_coor[i]:.3f}, {yaw[i] - 90:.3f}, {x_coor[i+1]:.3f}, {y_coor[i+1]:.3f}, {z_coor[i+1]:.3f}, {yaw[i+1] - 90:.3f}, {velocity_distance[i]:.3f}")

# x = rcos(theta)
# y = rsin(theta)
