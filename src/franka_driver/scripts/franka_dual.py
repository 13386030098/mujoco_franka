#!/usr/local/anaconda3/envs/mujoco/bin/python3.5
from mujoco_py import load_model_from_path, MjSim, MjViewer, functions, cymj, MjSimState,const
from robot_msgs.msg import ik
import math
import time
import rospy
import os
import numpy as np
from std_msgs.msg import Float64

def callback_omega_1(data):
    sim.data.ctrl[0] = data.data[0]
    sim.data.ctrl[1] = data.data[1]
    sim.data.ctrl[2] = data.data[2]
    sim.data.ctrl[3] = -1.0 + data.data[3]
    sim.data.ctrl[4] = data.data[4]
    sim.data.ctrl[5] = 1.5 + data.data[5]
    sim.data.ctrl[6] = 0.9 + data.data[6]

def callback_omega_2(data):

    sim.data.ctrl[7]  = data.data[0]
    sim.data.ctrl[8]  = data.data[1]
    sim.data.ctrl[9]  = data.data[2]
    sim.data.ctrl[10] = -1.2 + data.data[3]
    sim.data.ctrl[11] = data.data[4]
    sim.data.ctrl[12] = 2 + data.data[5]
    sim.data.ctrl[13] = 0.9 + data.data[6]



def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('omega1/ik', ik, callback_omega_1)
    rospy.Subscriber('omega2/ik', ik, callback_omega_2)
    pub = rospy.Publisher('touch_sensor_data', Float64, queue_size=10)

    # rospy.Timer(rospy.Duration(0.02), acquisition)#250hz

    rate = rospy.Rate(1000)
    t = 0
    while not rospy.is_shutdown():
        t += 1
        sim.step()
        viewer.render()
        sensor_data = sim.data.get_sensor("touch_sensor")
        pub.publish(sensor_data)
        print(sensor_data)

        if t > 100 and os.getenv('TESTING') is not None:
            break
        rate.sleep()

if __name__ == '__main__':
    model = load_model_from_path("/home/zzz/mujoco_franka/src/mujoco_description/franka_dual.xml")

    sim = MjSim(model)
    viewer = MjViewer(sim)
    try:
        listener()
    except rospy.ROSInterruptException: pass



























