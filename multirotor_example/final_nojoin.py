import airsim
import numpy as np
from math import asin, degrees, atan2, sin, tan
import cv2
import time
import joblib
clf = joblib.load('test(200,).pkl')
class processing():
    def __init__(self, th=1, no_join=False):
        self.ym = airsim.YawMode(is_rate=False)
        self.th = th
        self.no_join = no_join
        self.c = airsim.MultirotorClient()
        self.c.confirmConnection()
        self.c.enableApiControl(True)
        self.c.armDisarm(True)
        with open("drone_ring_list.txt", "r") as f:
            self.ring = f.readlines()
            self.ring = list(map(lambda s: s.strip(), self.ring))
            self.ring.append(self.ring[0])
    
    def get_frame(self, client, MAX_dist=30):
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
    
    def get_offset(self, dep, seg):
        _, bin = cv2.threshold(dep, 0, 255, cv2.THRESH_OTSU)
        cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(bin)
        try:
            if len(stats)>0: 
                off_y = int(centroids[0][0]) - int(dep.shape[1] / 2)
                off_z = int(centroids[0][1]) - int(dep.shape[0] / 2)
        except: 
            return 0, 0
        return off_y, off_z
    
    def is_arrived(self, x, y, z):
        x0 = float(self.c.simGetVehiclePose().position.x_val)
        y0 = float(self.c.simGetVehiclePose().position.y_val)
        z0 = float(self.c.simGetVehiclePose().position.z_val)
        d = ((x-x0)**2+(y-y0)**2+(z-z0)**2)**0.5
        if d < self.th: return True
        else: return False

    def Euler(self, ring):
        x, y, z, w = ring[0], ring[1], ring[2], ring[3]
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = degrees(atan2(t0, t1))
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2) * 180 / np.pi
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4) * 180 / np.pi
        return [yaw, roll, pitch]

    def coordinate(self):
        self.x_coor, self.y_coor, self.z_coor = [0]*72, [0]*72, [0]*72
        for i in range(72):
            data = list(self.c.simGetObjectPose(self.ring[i]))
            data = list(data[0])
            self.x_coor[i], self.y_coor[i], self.z_coor[i] = data[0], data[1], data[2]
        self.x_coor.append(0.0)
        self.y_coor.append(0.0)
        self.z_coor.append(-1.0)

    def Distance(self):
        self.distance, self.angle_list = [0] * 72, [0] * 72
        for i in range(0, 72):
            now = list(self.c.simGetObjectPose(self.ring[i]))
            now = list(now[0])
            next = list(self.c.simGetObjectPose(self.ring[i+1]))
            next = list(next[0])
            
            dist = ((next[0] - now[0]) ** 2 + (next[1] - now[1]) ** 2 + (next[2] - now[2]) ** 2) ** 0.5
            self.distance[i] = dist
            
            theta = 180 - (asin((next[1] - now[1]) / dist)) * 180 / np.pi
            self.angle_list[i] = theta
            if (next[0] - now[0]) >= 0 and (next[1] - now[1]) >= 0:
                self.angle_list[i] -= 90
            elif (next[0] - now[0]) > 0 and (next[1] - now[1]) < 0:
                self.angle_list[i] += 90
        self.angle_list.append(0)
        self.distance.append(30)
    
    def Yaw(self):
        self.yaw = [0] * 72
        for i in range(72):
            data = list(self.c.simGetObjectPose(self.ring[i + 1]))
            [Yaw, pitch, roll] = self.Euler(list(data[1]))
            self.yaw[i] = Yaw
        first_ring = list(self.c.simGetObjectPose(self.ring[0]))
        [yaw0, pitch0, roll0] = self.Euler(list(first_ring[1]))
        self.yaw.append(yaw0)
        self.yaw.insert(0,-90)
    
    def d_yaw(self):
        self.Yaw()
        self.delta_yaw = [0] * 72
        for i in range(72):
            self.delta_yaw[i] = abs(self.yaw[i+1] - self.yaw[i])
            if self.delta_yaw[i] >= 180:
                self.delta_yaw[i] = abs(360 - abs((self.yaw[i+1] - self.yaw[i])))
    
    def time_sleep(self):
        maximum, minimum = 0.4, 0.2
        delta = maximum - minimum
        self.t = [0] * 72
        for i in range(72):
            quality = self.delta_yaw[i] / 50
            self.t[i] = quality * delta + minimum
        self.t.append(0)
    
    def velocity_distance(self):
        self.d_yaw()
        self.Distance()
        max_velocity, min_velocity = 10, 6
        max_distance, min_distance = max(self.distance), min(self.distance)
        delta_v = max_velocity - min_velocity
        delta_d = max_distance - min_distance
        self.velocity = [0] * 72
        for i in range(72):
            quality = (self.distance[i] - min_distance)/delta_d
            self.velocity[i] = quality * delta_v + min_velocity
            if self.delta_yaw[i] >= 30 and 10 < self.distance[i] <= 14:
                self.velocity[i] -= 0.8
            elif self.delta_yaw[i] >= 30 and self.distance[i] <= 10:
                self.velocity[i] -= 1.5
        self.velocity.insert(0, 10)

    def move(self):
        self.Distance()
        self.coordinate()
        self.velocity_distance()
        self.c.takeoffAsync().join()
        self.time_sleep()
        self.d_yaw()
        dy, dz = 0, 0
        for i in range(73):
            self.ym.yaw_or_rate = self.yaw[i]-90
            if i >= 66 :
                if self.no_join:
                    self.c.moveToPositionAsync(self.x_coor[i], self.y_coor[i], self.z_coor[i], 4.5,
                        yaw_mode=self.ym)
                    print(f"{dy:.3f}, {dz:.3f}, {self.distance[i]:.3f}, {self.delta_yaw[i]:.3f}, {self.z_coor[i]:.3f}, {self.velocity[i]:.3f}")
                    while not self.is_arrived(self.x_coor[i], self.y_coor[i], self.z_coor[i]):
                        seg, dep = self.get_frame(self.c)
                        dy, dz = self.get_offset(dep, seg)
                else:
                    self.c.moveToPositionAsync(self.x_coor[i], self.y_coor[i], self.z_coor[i], 4.5,
                    yaw_mode=self.ym).join()
            else:
                if self.no_join:
                    self.c.moveToPositionAsync(self.x_coor[i], self.y_coor[i], self.z_coor[i], self.velocity[i],
                        yaw_mode=self.ym)
                    print(f"{dy:.3f}, {dz:.3f}, {self.distance[i]:.3f}, {self.delta_yaw[i]:.3f}, {self.z_coor[i]:.3f}, {self.velocity[i]:.3f}")
                    while not self.is_arrived(self.x_coor[i], self.y_coor[i], self.z_coor[i]):
                        seg, dep = self.get_frame(self.c)
                        dy, dz = self.get_offset(dep, seg)
                else:
                    self.c.moveToPositionAsync(self.x_coor[i], self.y_coor[i], self.z_coor[i], self.velocity[i],
                    yaw_mode=self.ym).join()
            

if __name__== "__main__":
    drone = processing(th=1, no_join=True)
    drone.move()