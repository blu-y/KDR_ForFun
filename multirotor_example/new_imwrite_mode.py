import airsim
import numpy as np
from math import asin, acos, degrees, atan2, cos, sin, tan
import cv2
import time
from sympy import Symbol, solve
c = airsim.MultirotorClient()
c.confirmConnection()
c.enableApiControl(True)
c.armDisarm(True)
c.simGetVehiclePose()
c.takeoffAsync().join()

with open("multirotor_example/drone_ring_list.txt", "r") as f:
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

distance = [0] * 72
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
    distance[i] = dist
    #theta = acos((forward[0] - backward[0]) / dist)
    #angle_list[i] = degrees(theta)
    theta = 180 - (asin((forward[1] - backward[1]) / dist)) * 180 / np.pi
    angle_list[i] = theta
    if (forward[0] - backward[0]) >= 0 and (forward[1] - backward[1]) >= 0:
        angle_list[i] -= 90
    elif (forward[0] - backward[0]) > 0 and (forward[1] - backward[1]) < 0:
        angle_list[i] += 90
    #print(angle_list[i])

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

def get_offset(img):
    _, bin = cv2.threshold(img, 0, 255, cv2.THRESH_OTSU)
    cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(bin)
    # print(centroids)

    try:
        if len(stats) > 0:
            dst = np.zeros(img.shape)
            # print("stats",stats)
            dy = stats[-1][0] + stats[-1][2] / 2 - 256 / 2
            dz = stats[-1][1] + stats[-1][3] / 2 - 144 / 2
            # cv2.rectangle(dst, (int(stats[-1][0]), int(stats[-1][1])), (int(stats[-1][2]), int(stats[-1][3])), (255, 255, 255))

            dst = cv2.circle(dst, (int(centroids[0][0]), int(centroids[0][1])), 2, (255, 255, 255), -1)
            # seg = cv2.circle(seg, (int(centroids[0][0]),int(centroids[0][1])), 2, (255,0,0), -1)
            off_y = int(centroids[0][0]) - int(dst.shape[1] / 2)
            off_z = int(centroids[0][1]) - int(dst.shape[0] / 2)

        else:
            print('fail')
    except:
        print('no list')
        return 0, 0
    return off_y, off_z

def fly(a, b, v):
    for i in range(a, b):
        data = list(c.simGetObjectPose(ring[i]))
        data = list(data[0])
        c.moveToPositionAsync(data[0], data[1], data[2], v).join()
        try:
            next = list(c.simGetObjectPose(ring[i+1]))
        except:
            next = list(c.simGetObjectPose(ring[0]))
            pass
        [yaw, pitch, roll] = quaternion_to_euler(list(next[1]))
        c.rotateToYawAsync(yaw-90)
        time.sleep(1)

        data = list(c.simGetObjectPose(ring[i]))
        data = list(data[0])
        c.moveToPositionAsync(data[0], data[1], data[2], v).join()

        seg, dep = get_frame(c)
        name = "img/sep" + str(i) + '.jpg'
        cv2.imwrite(name, dep)
        if i == b: break

def system_of_equation(data0, data1, next0, next1, yaw0, yaw1):
    x = Symbol('x')
    y = Symbol('y')
    slope0 = tan(yaw0 * np.pi / 180)
    slope1 = tan(yaw1 * np.pi / 180)
    eq1 = slope0 * x - data0 * slope0 + data1 - y
    eq2 = slope1 * x - next0 * slope1 + next1 - y
    Result = solve((eq1, eq2), dict=True)
    Result = Result[0]
    new = list(Result.values())
    x_coor = new[0]
    y_coor = new[1]
    return [x_coor, y_coor]

def cap_fly(a, b, v):
    for i in range(a, b):
        data = list(c.simGetObjectPose(ring[i]))
        data = list(data[0])
        c.moveToPositionAsync(data[0], data[1], data[2], v).join()
        try:
            next = list(c.simGetObjectPose(ring[i+1]))
        except:
            next = list(c.simGetObjectPose(ring[0]))
            pass
        now = list(c.simGetObjectPose(ring[i]))
        [yaw0, pitch0, roll0] = quaternion_to_euler(list(now[1]))
        [yaw1, pitch1, roll1] = quaternion_to_euler(list(next[1]))
        c.rotateToYawAsync(yaw1 - 90)
        time.sleep(0.5)

        next_0 = list(next[0])
        new_position = system_of_equation(data[0], data[1], next_0[0], next_0[1], yaw0, yaw1)
        c.moveToPositionAsync(float(new_position[0]), float(new_position[1]), data[2], 3).join()
        time.sleep(1)


        seg, dep = get_frame(c)
        name = "img/dep" + str(i) + '.jpg'
        cv2.imwrite(name, dep)
        x, y = get_offset(dep)
        gt.append([x, y])
        print(gt)
        if i == b: break


gt = []
cap_fly(0, 71, 4.0)
gt = np.array(gt)
np.save('gt', gt)
c.moveByVelocityAsync(-7.0, 0, -1.0, 1).join()
c.landAsync().join()