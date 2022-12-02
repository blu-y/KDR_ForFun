#174fb8deea220eccedbd981c1779e62f84452ba6c94219a276e6a5d2c2e6a07b
import airsim
import numpy as np
from time import sleep, time
from math import asin, atan2
import pprint
c = airsim.MultirotorClient()
c.confirmConnection()
state = c.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

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
d = 0.1
i = 0
t0 = time()
c.enableApiControl(True)
c.armDisarm(True)
c.takeoffAsync().join()
i = 0
while True:
    i += 1
    y = airsim.YawMode(yaw_or_rate=-10)
    c.moveByVelocityAsync(-1, 0, 0, d*1.1, yaw_mode=y)
    sleep(d)
    if i == 50: break
'''
while True:
    x = c.simGetVehiclePose().position.x_val
    y = c.simGetVehiclePose().position.y_val
    z = c.simGetVehiclePose().position.z_val
    qx = c.simGetVehiclePose().orientation.x_val
    qy = c.simGetVehiclePose().orientation.y_val
    qz = c.simGetVehiclePose().orientation.z_val
    qw = c.simGetVehiclePose().orientation.w_val
    [y, p, r] = quaternion_to_euler([qx, qy, qz, qw])
    state = c.getMultirotorState()
    rc = state.rc_data
    if i == 50: print(rc)
    t = time()
    print(f"{rc.timestamp:.3f} RC r:{rc.roll:.3f}, p:{rc.pitch:.3f}, y:{rc.yaw:.3f}, t:{rc.throttle:.3f} x:{x:.3f}, y:{y:.3f}, z:{z:.3f}, r:{r:.3f}, p:{p:.3f}, y:{y:.3f}")
    sleep(0.09)
    i += 1
    if i == 100: break
'''