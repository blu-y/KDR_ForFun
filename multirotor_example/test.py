"""
For connecting to the AirSim drone environment and testing API functionality
"""
import setup_path 
import airsim

import os
import tempfile
import pprint
import time
# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)
client.takeoffAsync().join()
x=-15
t = 0.01
P = 1
D = 0.3
ex0 = 0
while True:
    x0 = client.simGetVehiclePose().position.x_val
    ex = x-x0
    dex = (ex - ex0)/t
    print(x0)
    client.moveByVelocityAsync(P*ex+D*dex,0,-0.1, duration=t)
    time.sleep(t*0.5)
    ex0 = ex
    if abs(-15-x0) < 0.1: break

client.hoverAsync()
time.sleep(3)
print('a')

client.moveByManualAsync(vx_max = 1E6, vy_max = 1E6, z_min = -1E6, duration = 1E10)
#airsim.wait_key('Manual mode is setup. Press any key to send RC data to takeoff')
t = 1
while 0:
    t +=1
    client.moveByRC(rcdata = airsim.RCData(pitch = 0.001, throttle = 1.0, is_initialized = True, is_valid = True))
    if t>10000 : break
t = 1
while 0:
    t+=1
    time.sleep(0.001)
    client.moveByRC(rcdata = airsim.RCData(pitch = 0.3, throttle = 1.0, is_initialized = True, is_valid = True))
    if t>100 : break
#time.sleep(10)
#airsim.wait_key('Set Yaw and pitch to 0.5')
#client.moveByRC(rcdata = airsim.RCData(roll = 0.5, throttle = 1.0, yaw = 0.5, is_initialized = True, is_valid = True))