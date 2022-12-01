#174fb8deea220eccedbd981c1779e62f84452ba6c94219a276e6a5d2c2e6a07b
import airsim
import numpy as np

c = airsim.MultirotorClient()
c.enableApiControl(True)
c.simGetVehiclePose()
c.takeoffAsync().join()

with open("drone_ring_list.txt", "r") as f:
    ring = f.readlines()  # ['첫 번째 줄\n', '두 번째 줄\n', '세 번째 줄'] 저장
    ring = list(map(lambda s: s.strip(), ring))

# 1번째부터 64번째까지 Ring간의 거리가 좀 멀어서 좀 빠른 속도
for i in range(0, 3):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 6.5).join()

for i in range(3, 5):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 5.5).join()

for i in range(5, 12):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 6.5).join()

for i in range(12, 18):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 5).join()

for i in range(18, 25):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 6.5).join()

for i in range(25, 30):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 5.5).join()

for i in range(30, 40):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 6.5).join()

for i in range(40, 49):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 5.8).join()

for i in range(49, 64):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 7).join()

# 64번째부터 73번재까지 Ring간의 거리가 좀 가깝고 급격한 방향전환이 필요해서 느린 속도
for i in range(64, 70):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 5).join()

for i in range(70, 72):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 4).join()

# 마지막에 앞으로 조금 가줘야 기록 측정완료 후 Landing
c.moveByVelocityAsync(-7.0, 0, -1.0, 2)
c.landAsync().join()