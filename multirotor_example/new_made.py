import airsim
import numpy as np
import time
from datetime import datetime
from threading import Event
# 174fb8deea220eccedbd981c1779e62f84452ba6c94219a276e6a5d2c2e6a07b
c = airsim.MultirotorClient()
c.enableApiControl(True)
c.takeoffAsync().join()
'''
c.simListSceneObjects()  #전체 정보 불러오기
c.moveByVelocityAsync(5.0, 0, 0, 5)  x방향으로 5의 속도로 5초동안 움직임
c.moveToPositionAsync(10, 10, -10, 2) x, y, z의 방향으로 2m/s의 속도로 움직임
c.simSetSegmentationObjectID('[\w]*', 0, True)  # 나머지 전체를 검정색
c.simSetSegmentationObjectID('KDR_Ring_2[\w]*', 255, True) # 선택한 부분만 하얀색으로 Segmentation창에 띄움

KDR_Ring_1 ~ KDR_Ring_9
KDR_Ring10 ~ KDR_Ring_73
'''


'''ring = [0, 0, 0]*72
for i in range(2, 10, 1):
    ring[i] = list(c.simGetObjectPose(f'KDR_Ring_{i}'))
    ring[i] = list(ring[i][0])
    print(ring[i])
data = list(c.simGetObjectPose('KDR_Ring_2'))
data = list(data[0])
print(data)
c.moveToPositionAsync(data[0], data[1], data[2], 10)
'''

while True:
    start = datetime.now()
    init = list(c.simGetObjectPose('KDR_Ring_2'))
    init = list(init[0])
    c.moveToPositionAsync(init[0], init[1], init[2], 5)
    end = datetime.now()
    delta = end - start
    time.sleep(6 - delta.total_seconds())

    start = datetime.now()
    init = list(c.simGetObjectPose('KDR_Ring2_5'))
    init = list(init[0])
    c.moveToPositionAsync(init[0], init[1], init[2], 5)
    end = datetime.now()
    delta = end - start
    time.sleep(4 - delta.total_seconds())
    time.sleep(1)
    for i in range(3, 12):
        #start = datetime.now()
        data = list(c.simGetObjectPose(f'KDR_Ring{i}'))
        data = list(data[0])
        #while c.getMultirotorState() !=
        c.moveToPositionAsync(data[0], data[1], data[2], 5)
        #end = datetime.now()
        #delta = end - start
        #time.sleep(3 - delta.total_seconds())

    start = datetime.now()
    init = list(c.simGetObjectPose('KDR_Ring12_17'))
    init = list(init[0])
    c.moveToPositionAsync(init[0], init[1], init[2], 5)
    end = datetime.now()
    delta = end - start
    time.sleep(4 - delta.total_seconds())
    time.sleep(1)

    for i in range(13, 17):
        start = datetime.now()
        data = list(c.simGetObjectPose(f'KDR_Ring{i}'))
        data = list(data[0])
        c.moveToPositionAsync(data[0], data[1], data[2], 5)
        end = datetime.now()
        delta = end - start
        time.sleep(3 - delta.total_seconds())

    '''start = datetime.now()
    init = list(c.simGetObjectPose('KDR_Ring17_83'))
    init = list(init[0])
    c.moveToPositionAsync(init[0], init[1], init[2], 5)
    end = datetime.now()
    delta = end - start
    time.sleep(4 - delta.total_seconds())
    time.sleep(1)'''

    for i in range(18, 74):
        start = datetime.now()
        data = list(c.simGetObjectPose(f'KDR_Ring{i}'))
        data = list(data[0])
        c.moveToPositionAsync(data[0], data[1], data[2], 5)
        end = datetime.now()
        delta = end - start
        time.sleep(3 - delta.total_seconds())