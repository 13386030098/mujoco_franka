#!/usr/local/anaconda3/envs/mujoco/bin/python3.5
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal
import functools

from mujoco_py import load_model_from_path, MjSim, MjViewer, functions, cymj, MjSimState,const
from robot_msgs.msg import ik
from std_msgs.msg import Float64MultiArray
import math
import time
import rospy
import os


def callback_omega_1(data):

    sim.data.ctrl[0] = data.data[0]
    sim.data.ctrl[1] = data.data[1]
    sim.data.ctrl[2] = data.data[2]
    sim.data.ctrl[3] = data.data[3]
    sim.data.ctrl[4] = data.data[4]
    sim.data.ctrl[5] = data.data[5]
    sim.data.ctrl[6] = data.data[6]
    print(data.data[0], data.data[1], data.data[2], -1.3 + data.data[3], data.data[4], 1.5 + data.data[5], 0.9 + data.data[6])

def listener():
    rospy.init_node('franka_driver', anonymous=True)
    rospy.Subscriber('/ik', ik, callback_omega_1)
    pub = rospy.Publisher('joint_pos_vector', Float64MultiArray, queue_size=1)
    t = 0
    rate = rospy.Rate(1000)
    sim.data.ctrl[3] = -1.3
    sim.data.ctrl[5] = 1.5
    sim.data.ctrl[6] = 0.9

    while not rospy.is_shutdown():
        sim_state = sim.get_state()
        panda0_joint1 = sim.data.get_sensor("panda0_joint1")
        panda0_joint2 = sim.data.get_sensor("panda0_joint2")
        panda0_joint3 = sim.data.get_sensor("panda0_joint3")
        panda0_joint4 = sim.data.get_sensor("panda0_joint4")
        panda0_joint5 = sim.data.get_sensor("panda0_joint5")
        panda0_joint6 = sim.data.get_sensor("panda0_joint6")
        panda0_joint7 = sim.data.get_sensor("panda0_joint7")
        joint_pos_vector = Float64MultiArray()
        joint_pos_vector.data = [panda0_joint1, panda0_joint2, panda0_joint3,
        panda0_joint4, panda0_joint5, panda0_joint6, panda0_joint7]
        pub.publish(joint_pos_vector)

        # start = time.time()
        t += 1
        sim.step()
        viewer.render()

#        print(panda0_joint6)
        if t > 100 and os.getenv('TESTING') is not None:
            break
        rate.sleep()
        # stop = time.time()
        # print(str((stop - start) * 1000) + "ms")

if __name__ == '__main__':
    model = load_model_from_path("/home/zzz/mujoco_franka/src/mujoco_description/franka_dual.xml")

    sim = MjSim(model)
    viewer = MjViewer(sim)
    try:
        listener()
    except rospy.ROSInterruptException: pass



























