#!/usr/bin/python3
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
import os
import sys
import time
import smbus
import numpy as np

from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman 

def composeMessage(count, roll, pitch, yaw, accel_x, accel_y, accel_z):
  
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = "base_link"
  msg.header.seq = count

  msg.twist.twist.linear.x = accel_x
  msg.twist.twist.linear.y = accel_y
  msg.twist.twist.linear.z = accel_z

  msg.twist.twist.angular.x = roll
  msg.twist.twist.angular.y = pitch
  msg.twist.twist.angular.z = yaw
  msg.twist.covariance = [0.001, 0, 0, 0, 0, 0,
                          0, 0.001, 0, 0, 0, 0,
                          0, 0, 0.001, 0, 0, 0,
                          0, 0, 0, 0.001, 0, 0,
                          0, 0, 0, 0, 0.001, 0,
                          0, 0, 0, 0, 0, 0.001]
  imu_sensor.publish(msg)

if __name__ == "__main__":

  rospy.init_node('imu_sensor')
  r = rospy.Rate(60)
  imu_sensor = rospy.Publisher('/imu_sensor_link', TwistWithCovarianceStamped, queue_size=10)
  msg = TwistWithCovarianceStamped()
  address = 0x68
  bus = smbus.SMBus(1)
  imu = MPU9250.MPU9250(bus, address)
  imu.begin()
  sensorfusion = kalman.Kalman()
  imu.loadCalibDataFromFile("/home/ubuntu/Roboprint/Autriger/python/calib1.json")
  imu.readSensor()
  imu.computeOrientation()
  sensorfusion.roll = imu.roll
  sensorfusion.pitch = imu.pitch
  sensorfusion.yaw = imu.yaw

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
 
    composeMessage(count, sensorfusion.pitch, sensorfusion.roll, imu.yaw+40,imu.AccelVals[1], imu.AccelVals[0], imu.AccelVals[2])
    count += 1
    r.sleep()