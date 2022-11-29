import airsim
import numpy as np
import schedule
import time

c = airsim.MultirotorClient()
c.enableApiControl(True)
c.takeoffAsync().join()

def move(ring):
    data = list(c.simGetObjectPose(ring[0]))
    data = list(data[0])
    return c.moveToPositionAsync(data[0], data[1], data[2], 7)

with open("drone_ring_list.txt", "r") as f:
    ring = f.readlines()  # ['첫 번째 줄\n', '두 번째 줄\n', '세 번째 줄'] 저장
    ring = list(map(lambda s: s.strip(), ring))

schedule.every(5).seconds.move(ring)

'''
data = list(c.simGetObjectPose('KDR_Ring_2'))
data = list(data[0])
c.moveToPositionAsync(data[0], data[1], data[2], 7)

# 마지막에 앞으로 조금 가줘야함
c.moveByVelocityAsync(-5.0, 0, -1.0, 2)
c.landAsync().join()
'''