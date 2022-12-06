#174fb8deea220eccedbd981c1779e62f84452ba6c94219a276e6a5d2c2e6a07b
import airsim
import numpy as np
from math import asin, acos, degrees, atan2, cos, sin, tan, radians
import cv2
import time
from sympy import Symbol, solve
import copy
import matplotlib.pyplot as plt

c = airsim.MultirotorClient()
c.confirmConnection()
c.enableApiControl(True)
c.armDisarm(True)
c.simGetVehiclePose()

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

with open("drone_ring_list.txt", "r") as f:
    ring = f.readlines()  # ['첫 번째 줄\n', '두 번째 줄\n', '세 번째 줄'] 저장
    ring = list(map(lambda s: s.strip(), ring))
# Ring 73개 => 72번째 Ring뒤에 1번째 Ring 추가함
ring.append(ring[0])

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

def ferguson_plot(x_coor, y_coor, z_coor, yaw, velocity = 10):
    for i in range(0, 71):
        plt.figure(i)
        for t in np.linspace(0, 1, 10):
            # x(t) = x0 + u0t + [3(x1-x0)-2u0-u1]t^2+[2(x0-x1)+u0+u1]t^3
            T = np.array([[1,t,t**2,t**3]])
            C = np.array([[1,0,0,0],[0,0,1,0],[-3,3,-2,-1],[2,-2,1,1]])
            TC = np.matmul(T, C)
            u0 = 2 * cos(radians(yaw[i]))
            u1 = velocity * cos(radians(yaw[i+1]))
            v0 = 2 * sin(radians(yaw[i]))
            v1 = velocity * sin(radians(yaw[i+1]))
            Sx = np.transpose(np.array([x_coor[i],x_coor[i+1],u0,u1]))
            Sy = np.transpose(np.array([y_coor[i],y_coor[i+1],v0,v1]))
            x_position = np.matmul(TC,Sx)
            y_position = np.matmul(TC,Sy)
            z_position = float((z_coor[i + 1] - z_coor[i]) * t + z_coor[i])
            plt.plot(x_position, y_position)
        plt.show()
            
def ferguson(x_coor, y_coor, z_coor, yaw, velocity):
    for i in range(0, 71):
        t= 0.1
        while t < 1:
            my_pose_x = float(c.simGetVehiclePose().position.x_val)
            my_pose_y = float(c.simGetVehiclePose().position.y_val)
            my_pose_z = float(c.simGetVehiclePose().position.z_val)
            # x(t) = x0 + u0t + [3(x1-x0)-2u0-u1]t^2+[2(x0-x1)+u0+u1]t^3
            T = np.array([[1,t,t**2,t**3]])
            C = np.array([[1,0,0,0],[0,0,1,0],[-3,3,-2,-1],[2,-2,1,1]])
            TC = np.matmul(T, C)
            u0 = 2 * cos(radians(yaw[i]))
            u1 = velocity * cos(radians(yaw[i+1]))
            v0 = 2 * sin(radians(yaw[i]))
            v1 = velocity * sin(radians(yaw[i+1]))
            Sx = np.transpose(np.array([x_coor[i],x_coor[i+1],u0,u1]))
            Sy = np.transpose(np.array([y_coor[i],y_coor[i+1],v0,v1]))
            x_position = np.matmul(TC,Sx)
            y_position = np.matmul(TC,Sy)
            #x_position = float(x_coor[i] + velocity * t - velocity * cos(yaw[i] * 180 / np.pi) * (t ** 2) + 4 * velocity * cos(yaw[i+1] * 180 / np.pi) * (t ** 3))
            #y_position = float(y_coor[i] + y_coor[i+1] * t - velocity * sin(yaw[i] * 180 / np.pi) * (t ** 2) + 4 * velocity * sin(yaw[i+1] * 180 / np.pi) * (t ** 3))
            z_position = float((z_coor[i + 1] - z_coor[i]) * t + z_coor[i])
            dist = ((my_pose_x - x_coor[i]) ** 2 + (my_pose_y - y_coor[i]) ** 2 + (my_pose_z - z_coor[i]) ** 2 ) ** 0.5
            dist_ = ((my_pose_x - x_position) ** 2 + (my_pose_y - y_position) ** 2 + (my_pose_z - z_position) ** 2 ) ** 0.5
            if dist_ <= 2:
                t += 0.1
                #print(dist, dist_, x_position, y_position, u0, u1, v0, v1)
            if dist <= 0.1 : break
            else :
                #print(float(x_position), float(y_position), z_position)
                c.moveByVelocityAsync(float(x_position)-my_pose_x, float(y_position)-my_pose_y, z_position-my_pose_z, 0.2)
                #c.moveToPositionAsync(float(x_position), float(y_position), z_position, velocity)
                #time.sleep(0.1)
            
c.takeoffAsync().join()
#ferguson(x_coor, y_coor, z_coor, yaw, 7)
#ferguson_plot(x_coor, y_coor, z_coor, yaw, velocity=7)

fly = 2
p = 4
d = 3
tmax = 101

plt.plot(-y_coor[0], -x_coor[0], 'o')
c.moveToPositionAsync(x_coor[0], y_coor[0], z_coor[0], 10).join()
for i in range(1, 72):
    dist = ((x_coor[1] - x_coor[0])**2 + (y_coor[1]-y_coor[1])**0.5) 
    t = np.linspace(0, 1, tmax)
    # x(t) = x0 + u0t + [3(x1-x0)-2u0-u1]t^2+[2(x0-x1)+u0+u1]t^3
    x0 = x_coor[i]
    x1 = x_coor[i+1]
    y0 = y_coor[i]
    y1 = y_coor[i+1]
    u0 = 0.01*dist * cos(radians(yaw[i]-90))
    u1 = 0.12*dist * cos(radians(yaw[i+1]-90))
    v0 = 0.01*dist * sin(radians(yaw[i]-90))
    v1 = 0.12*dist * sin(radians(yaw[i+1]-90))
    x = x0 + u0*t + t**2 * (3*(x1-x0)-2*u0-u1) + t**3 * (2*(x0-x1)+u0+u1)
    y = y0 + v0*t + t**2 * (3*(y1-y0)-2*v0-v1) + t**3 * (2*(y0-y1)+v0+v1)
    z = z_coor[i]+(z_coor[i+1]-z_coor[i])*t/(tmax)
    ti = 0
    dx0 = 0
    dy0 = 0
    coor = np.stack([np.transpose(x),np.transpose(y), np.transpose(z)], 1).tolist()
    if fly == 2: c.moveOnPathAsync(coor, 5)
    while fly == 1:
        cx = float(c.simGetVehiclePose().position.x_val)
        cy = float(c.simGetVehiclePose().position.y_val)
        cz = float(c.simGetVehiclePose().position.z_val)
        dx = x[ti]-cx
        dy = y[ti]-cy
        zc = z_coor[i]+(z_coor[i+1]-z_coor[i])*ti/(tmax)
        dz = zc-cz
        #c.moveToPositionAsync(x[ti], y[ti], zc, 5)
        c.moveByVelocityAsync(p*dx+d*(dx-dx0)/0.05, p*dy+d*(dy-dy0)/0.05, p*dz, 0.1)
        time.sleep(0.01)
        dx0 = dx
        dy0 = dy
        dist_ = (dx ** 2 + dy ** 2) ** 0.5
        #print(dx, dy, dz, dist_)
        if dist_<1.5: ti+=1
        if ti == tmax: 
            dx0 = 0
            break
    #for i in range()
    plt.plot(-y, -x)

plt.show()