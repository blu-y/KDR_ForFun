import airsim as a

import airsimneurips as airsim
c = airsim.MultirotorClient()
c.confirmConnection()
c.enableApiControl()
c.arm()
c.takeoffAsync().join()

with open("drone_ring_list.txt", "r") as f:
    ring = f.readlines()  # ['첫 번째 줄\n', '두 번째 줄\n', '세 번째 줄'] 저장
    ring = list(map(lambda s: s.strip(), ring))
for i in range(64):
    data = list(c.simGetObjectPose(ring[i]))
    data = list(data[0])
    c.moveToPositionAsync(data[0], data[1], data[2], 6.5).join()