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
def Circle_centroid(data0, data1, next0, next1, yaw0, yaw1):
    x = Symbol('x')
    y = Symbol('y')
    slope0 = -1 / tan((yaw0 - 90) * np.pi / 180)
    slope1 = -1 / tan((yaw1 - 90) * np.pi / 180)
    eq1 = slope0 * x - data0 * slope0 + data1 - y
    eq2 = slope1 * x - next0 * slope1 + next1 - y
    Result = solve((eq1, eq2), dict=True)
    Result = Result[0]
    new = list(Result.values())
    x_coor = new[0]
    y_coor = new[1]
    radius = ((x_coor - data0)**2 + (y_coor - data1)** 2)**0.5
    return [x_coor, y_coor, radius]
def arc_circle(data0, data1, next0, next1):
    # 원의 방정식 : (centroid_x - x_i)**2 + (centroid_y - y_i)**2 = (centroid_x - data0)**2
    x = Symbol('x')
    y = Symbol('y')
    circle_centroid = Circle_centroid(x_coor[0], y_coor[0], x_coor[1], y_coor[1], yaw[0] - 90, yaw[1] - 90)
    #print(circle_centroid[0], circle_centroid[1], circle_centroid[2], '중심좌표, 중심좌표, 반지름')
    if abs(next0 - data0) >= abs(next1 - data1):
        x = (next0 - data0) / 10
        if next1 - data1 > 0:
            y = circle_centroid[1] + (circle_centroid[2] ** 2 - (x - circle_centroid[0])**2)**0.5
        elif next1 - data1 < 0:
            y = circle_centroid[1] - (circle_centroid[2] ** 2 - (x - circle_centroid[0])**2)**0.5
    elif abs(next0 - data0) < abs(next1 - data1):
        y = (next0 - data0) / 10
        if next0 - data0 > 0:
            x = circle_centroid[0] + (circle_centroid[2] ** 2 - (y - circle_centroid[1])**2)**0.5
        elif next1 - data1 < 0:
            x = circle_centroid[0] - (circle_centroid[2] ** 2 - (y - circle_centroid[1])**2)**0.5
    return x, y
#arc_circle = list(arc_circle())    
#arc_x, arc_y = arc_circle[0], arc_circle[1]
#print(Circle_centroid(x_coor[0], y_coor[0], x_coor[1], y_coor[1], yaw[0] - 90, yaw[1] - 90))
#print(arc_circle(x_coor[0], y_coor[0], x_coor[1], y_coor[1]))

def velocity_distance():
    max_velocity = 10.0
    min_velocity = 5.5
    max_distance = max(distance)
    min_distance = min(distance)
    delta_v = max_velocity - min_velocity
    delta_dist = max_distance - min_distance
    final_v = [0] * 72
    for i in range(72):
        quality = (distance[i] - min_distance)/delta_dist
        final_v[i] = quality * delta_v + min_velocity
        if distance[i] >20 and abs(z_coor[i+1] - z_coor[i]) < 1.5:
            final_v[i] += 0.4
        if distance[i] >20 and delta_yaw[i] < 15:
            final_v[i] += 0.4
        if distance[i] <= 15:
            final_v[i] -= 0.4
        elif delta_yaw[i] >= 30 and 10 <distance[i] <= 14:
            final_v[i] -= 1.5
        elif delta_yaw[i] >= 30 and distance[i] <= 10:
            final_v[i] -= 2
    final_v.append(10)
    return final_v
velocity_distance = velocity_distance()
def Three_points_circle(x, y, x_i, y_i, x_j, y_j):
    d1 = (x_i - x) / (y_i - y)
    d2 = (x_j - x_i) / (y_j - y_i)
    cx=((y_j-y)+(x_i+x_j)*d2-(x+x_i)*d1)/(2*(d2-d1))
    cy=-d1*(cx-(x+x_i)/2)+(y+y_i)/2
    r = ((x-cx)**2+(y-cy)**2)**0.5
    return [cx, cy, r]
def Arc_circle(x1, y1, x2, y2, x3, y3, t):
    three_points_circle = Three_points_circle(x1, y1, x2, y2, x3, y3)
    if abs(x2 - x1) >= abs(y2 - y1):
        x = (x2 - x1) / 10 * t + x1
        if y2 - y1 > 0:
            y = three_points_circle[1] + (three_points_circle[2] ** 2 - (x - three_points_circle[0])**2)**0.5
        else:
            y = three_points_circle[1] - (three_points_circle[2] ** 2 - (x - three_points_circle[0])**2)**0.5
    elif abs(x2 - x1) < abs(y2 - y1):
        y = (y2 - y1) / 10 * t + y1
        if x2 - x1 > 0:
            x = three_points_circle[0] + (three_points_circle[2] ** 2 - (y - three_points_circle[1])**2)**0.5
        else:
            x = three_points_circle[0] - (three_points_circle[2] ** 2 - (y - three_points_circle[1])**2)**0.5
    return [x, y]
def move2():
    c.takeoffAsync().join()
    velocity_distance.insert(0, 9)
    for i in range(71):
        three_points_circle = Three_points_circle(x_coor[i], y_coor[i], x_coor[i+1], y_coor[i+1], x_coor[i+2], y_coor[i+2])
        for t in range(1, 10):
            points = Arc_circle(x_coor[i], y_coor[i], x_coor[i+1], y_coor[i+1], x_coor[i+2], y_coor[i+2], t)
            c.moveToPositionAsync(points[0], points[1], z_coor[i], velocity_distance[i])
def phi():
    phi = [0] * 73
    for i in range(72):
        dist = (distance[i]**2 + (z_coor[i+1] - z_coor[i])**2)**0.5
        phi[i] = acos(distance[i] / dist)
    return phi
phi = phi()
def velocity_element():
    vx, vy, vz = [0]*72, [0]*72, [0]*72
    for i in range(72):
        vx[i] = velocity_distance[i] * cos(phi[i]) * cos(angle_list[i])
        vy[i] = velocity_distance[i] * cos(phi[i]) * sin(angle_list[i])
        vz[i] = velocity_distance[i] * cos(phi[i])
    return [vx, vy, vz]
velocity_element = velocity_element()
def move():
    c.takeoffAsync().join()
    velocity_distance.insert(0, 9)
    my_pose_x = c.simGetVehiclePose().position.x_val
    my_pose_y = c.simGetVehiclePose().position.y_val
    my_pose_z = c.simGetVehiclePose().position.z_val
    for i in range(73):
        if i >= 64 :
            c.moveToPositionAsync(x_coor[i], y_coor[i], z_coor[i], 4.5).join()
            c.moveByVelocityAsync(-velocity_element[0][i], -velocity_element[1][i], -velocity_element[2][i], 1)
        else:
            c.moveToPositionAsync(x_coor[i], y_coor[i], z_coor[i], velocity_distance[i]).join()
            c.moveByVelocityAsync(-1000*velocity_element[0][i], -1000*velocity_element[1][i], -1000*velocity_element[2][i], 1)
        c.rotateToYawAsync(yaw[i]-90)
        time.sleep(time_sleep[i])
move()

def my(x1, y1, yaw1):
    matrix = [0] * 72
    for i in range(72):
        A = np.array([[x1[i]**3, x1[i]**2, x1[i], 1], [x1[i+1]**3, x1[i+1]**2, x1[i+1], 1], [3*x1[i]**2, 2*x1[i], 1, 0], [3*x1[i+1]**2, 3*x1[i+1], 1, 0]])
        B = np.array([[y1[i]], [y1[i+1]], [yaw1[i] - 90], [yaw1[i+1] - 90]])
        A_inv = np.linalg.inv(A)
        matrix[i] = np.matmul(A_inv, B)
        coefficient = matrix[i]
    return matrix
coefficient = my(x_coor, y_coor, yaw)
def middle_coor(x1):
    x = [0] * 72
    y = [0] * 72
    for i in range(72):
        x[i] = (x1[i+1]-x1[i])/2 +x1[i]
        y[i] = coefficient[i][0]*(x[i]**3) + coefficient[i][1]*(x[i]**2) + coefficient[i][2]*x[i] + coefficient[i][3]
    return [x,y]
middle_coor = middle_coor(x_coor)
def move3():
    c.takeoffAsync().join()
    velocity_distance.insert(0, 9)
    for i in range(73):
        if i >= 67 :
            c.moveToPositionAsync(x_coor[i], y_coor[i], z_coor[i], 4.5).join()
            c.moveToPositionAsync(middle_coor[0][i], middle_coor[1][i][0], z_coor[i], 4.5).join()
        else:
            c.moveToPositionAsync(x_coor[i], y_coor[i], z_coor[i], velocity_distance[i]).join()
            c.moveToPositionAsync(middle_coor[0][i], middle_coor[1][i][0], z_coor[i], velocity_distance[i]).join()
        c.rotateToYawAsync(yaw[i]-90)
        time.sleep(time_sleep[i])