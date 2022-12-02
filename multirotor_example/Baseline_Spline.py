#174fb8deea220eccedbd981c1779e62f84452ba6c94219a276e6a5d2c2e6a07b
import airsim
import numpy as np

c = airsim.MultirotorClient()
c.confirmConnection()
c.enableApiControl(True)
c.simGetVehiclePose()
c.takeoffAsync().join()

'''
data = list(c.simGetObjectPose('KDR_Ring_2'))
data = list(data[0])
c.moveToPositionAsync(data[0], data[1], data[2], 7)
'''

with open("multirotor_example/drone_ring_list.txt", "r") as f:
    ring = f.readlines()  # ['첫 번째 줄\n', '두 번째 줄\n', '세 번째 줄'] 저장
    ring = list(map(lambda s: s.strip(), ring))

# 1번째부터 64번째까지 Ring간의 거리가 좀 멀어서 좀 빠른 속도
for i in range(64):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 6.5).join()

# 64번째부터 73번재까지 Ring간의 거리가 좀 가깝고 급격한 방향전환이 필요해서 느린 속도
for i in range(64, 72):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 5).join()

# 마지막에 앞으로 조금 가줘야 기록 측정완료 후 Landing
c.moveByVelocityAsync(-5.0, 0, -1.0, 2)
c.landAsync().join()


# 30 ~ 45 : 빠르게 가능
# 49 ~ 65 : 빠르게 가능

