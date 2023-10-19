#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Imu
import os
import sys
import time
import smbus
import numpy as np

from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman 

def composeMessage(count, angular_z, accel_x, accel_y):
  
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = "base_link"
  msg.header.seq = count

  msg.linear_acceleration.x = accel_x
  msg.linear_acceleration.y = accel_y
  
  msg.angular_velocity.z = angular_z

  imu_sensor.publish(msg)

if __name__ == "__main__":

  rospy.init_node('imu_sensor')
  r = rospy.Rate(60)
  imu_sensor = rospy.Publisher('/imu_sensor_link', Imu, queue_size=10)
  msg = Imu()
  address = 0x68
  bus = smbus.SMBus(1)
  imu = MPU9250.MPU9250(bus, address)
  imu.begin()
  sensorfusion = kalman.Kalman()
  imu.loadCalibDataFromFile("/home/ubuntu/Roboprint/Autriger/python/calib1.json")
  imu.readSensor()
  imu.computeOrientation()

  while not rospy.is_shutdown():
    count = 0
    currTime = time.time()
    imu.readSensor()
    imu.computeOrientation()
    newTime = time.time()
    dt = newTime - currTime
    currTime = newTime

    sensorfusion.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],\
                          imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)
 
    composeMessage(-count,imu.GyroVals[2] ,-imu.AccelVals[1], -imu.AccelVals[0])
    count += 1
    r.sleep()
