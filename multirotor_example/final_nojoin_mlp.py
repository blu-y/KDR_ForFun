import airsim
import numpy as np
from math import asin, degrees, atan2, sin, tan
import cv2
import time
import joblib
import matplotlib.pyplot as plt
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
    def get_ring(self):
        with open("drone_ring_list.txt", "r") as f:
            self.ring = f.readlines()
            self.ring = list(map(lambda s: s.strip(), self.ring))
            self.ring.append(self.ring[0])
    def cameraproc(self, client):
        dep, seg = self.GetImages(client, 30)
        dy, dz = self.detect_box(self.depproc(dep, seg))
        #plt.imshow(seg)
        #plt.show()
        return dy, dz
    def coordinate(self):
        self.x_coor, self.y_coor, self.z_coor = [0]*72, [0]*72, [0]*72
        for i in range(72):
            data = list(self.c.simGetObjectPose(self.ring[i]))
            data = list(data[0])
            self.x_coor[i], self.y_coor[i], self.z_coor[i] = data[0], data[1], data[2]
        self.x_coor.append(0.0)
        self.y_coor.append(0.0)
        self.z_coor.append(-1.0)
    def is_arrived(self, x, y, z):
        x0 = float(self.c.simGetVehiclePose().position.x_val)
        y0 = float(self.c.simGetVehiclePose().position.y_val)
        z0 = float(self.c.simGetVehiclePose().position.z_val)
        d = ((x-x0)**2+(y-y0)**2+(z-z0)**2)**0.5
        if d < self.th: return True
        else: return False
    def detect_box(self, dep):
        dy, dz = 0, 0
        _, bin = cv2.threshold(dep, 0, 255, cv2.THRESH_OTSU)
        _, _, stat, c = cv2.connectedComponentsWithStats(bin)
        try:
            if len(stat)>0: 
                dy = int(c[0][0]) - int(dep.shape[1] / 2)
                dz = int(c[0][1]) - int(dep.shape[0] / 2)
        except: pass
        return dy, dz
    def depproc(self, dep, seg, segment=83, th=100):
        seg[seg[:,:,0]!=segment] = 0
        seg[:,:,3] = 255
        dep[seg[:,:,0]==0] = 255
        dep[dep>th] = 255
        return dep
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
    def GetImages(self, client, th):
        dep_, = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),])
        dep = airsim.list_to_2d_float_array(dep_.image_data_float, dep_.width, dep_.height)
        dep = dep.reshape(dep_.height, dep_.width, 1)
        dep = np.interp(dep, (0, th), (0, 255)).astype(np.uint8)
        seg_ = client.simGetImage("0", airsim.ImageType.Segmentation)
        seg = cv2.imdecode(airsim.string_to_uint8_array(seg_), cv2.IMREAD_UNCHANGED)
        return dep, seg
    def Distance(self):
        self.distance = [0] * 72
        for i in range(72):
            now = list(self.c.simGetObjectPose(self.ring[i]))
            now = list(now[0])
            next = list(self.c.simGetObjectPose(self.ring[i+1]))
            next = list(next[0])
            dist = ((next[0] - now[0]) ** 2 + (next[1] - now[1]) ** 2 + (next[2] - now[2]) ** 2) ** 0.5
            self.distance[i] = dist
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
    def move(self):
        self.get_ring()
        self.coordinate()
        self.Yaw()
        self.c.takeoffAsync().join()
        dy, dz, vel = 0, 0, [9]
        for i in range(73):
            self.ym.yaw_or_rate = self.yaw[i]-90
            vel.append(clf.predict([[dy, dz, self.x_coor[i], self.y_coor[i], self.z_coor[i], self.yaw[i]-90, 
                       self.x_coor[i+1], self.y_coor[i+1], self.z_coor[i+1], self.yaw[i+1]-90]])[0])
            vel[i] = (vel[i]- 6.5)/6.5 * 2.5 + 7
            if self.no_join:
                self.c.moveToPositionAsync(self.x_coor[i], self.y_coor[i], self.z_coor[i], vel[i]-1,
                                           yaw_mode=self.ym)
                while not self.is_arrived(self.x_coor[i], self.y_coor[i], self.z_coor[i]):
                    dy, dz = self.cameraproc(self.c)
            else:
                self.c.moveToPositionAsync(self.x_coor[i], self.y_coor[i], self.z_coor[i], vel[i]-1,
                                           yaw_mode=self.ym).join()
                dy, dz = self.cameraproc(self.c)

if __name__== "__main__":
    drone = processing(th=1, no_join=True)
    drone.move()

