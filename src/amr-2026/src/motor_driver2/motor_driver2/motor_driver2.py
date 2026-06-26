#!/usr/bin/env python3.8
from distutils.command import check
import rclpy
from std_msgs.msg import String
import json
import yaml
import math 
import datetime
from datetime import timedelta
from time import time, sleep
from datetime import date, time, datetime, timezone

import serial
import struct
from sqlite3 import Error
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

setting_location = os.path.join(
	get_package_share_directory('storagy'),
	'param/setting.yaml'
)

Vx = 0.0
Wz = 0.0
ser = None
Emergency = 0x00
Relay = 0x00
LED = 0x00
AccelData = 0
DecelData = 0
ser_connect = False

vcu_msg = String()
battery_msg = String()
charging_msg = String()
motor_msg = String()
acc_dec_msg = String()
safearea_msg = String()
error_msg = String()
LED_State = 0
Last_LED_State = 0
odom_pub = None
Emergency_cmd = False
prev_yaw = 0.0

cnt =0
cnt1=0
cnt2=0
odom = Odometry()
t = TransformStamped()

def CmdVelCb(data):
  global Vx, Wz, ser, Emergency_cmd, Relay, LED_State, ser_connect, cnt
  cnt=cnt+1
  if Emergency_cmd:
    Vx = 0
    Wz = 0
  else:
    Vx = data.linear.x
    Wz = data.angular.z

  Vxr = bytearray(struct.pack("f", Vx))
  # print([ "0x%02x" % b for b in Vxr ])
  Wzr = bytearray(struct.pack("f", Wz))
  #print(data.linear.x)
  try:
    ser = serial.Serial('/dev/ttyS4', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
    values = bytearray([0x03, Vxr[0], Vxr[1], Vxr[2], Vxr[3],  Wzr[0],Wzr[1],Wzr[2],Wzr[3], LED_State, Relay])
    checksum1 =0
    for x in range(11):
      checksum1 += values[x] 
      checksum1 = checksum1 % 256  
    values1 = bytearray([0x03, Vxr[0], Vxr[1], Vxr[2], Vxr[3],  Wzr[0],Wzr[1],Wzr[2],Wzr[3], LED_State, Relay, checksum1])  
    # print([ "0x%02x" % b for b in values ])
    ser.write(values1)
    ser_connect = True
  except Error as e:
    ser_connect = False
    print(e)


def robot_syscommandCb(data):
  global Emergency_cmd
  if data.data == 'brake':
    Emergency_cmd = True 
  if data.data == 'release':  
    Emergency_cmd = False

def robot_brightnessCb(data):
  global LED_State,Last_LED_State, Vx, Wz, Relay
  bridata = data.data
  if bridata.isnumeric():
    Relay = int(int(data.data)*90/100) 
    Vxr = bytearray(struct.pack("f", Vx))
    Wzr = bytearray(struct.pack("f", Wz))    
    print(LED_State)
    try:
      ser = serial.Serial('/dev/ttyS4', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
      values = bytearray([0x03, Vxr[0], Vxr[1], Vxr[2], Vxr[3],  Wzr[0],Wzr[1],Wzr[2],Wzr[3], LED_State, Relay])
      checksum1 =0
      for x in range(11):
        checksum1 += values[x] 
        checksum1 = checksum1 % 256  
      values1 = bytearray([0x03, Vxr[0], Vxr[1], Vxr[2], Vxr[3],  Wzr[0],Wzr[1],Wzr[2],Wzr[3], LED_State, Relay, checksum1])  
      # print([ "0x%02x" % b for b in values ])
      ser.write(values1)
      ser_connect = True
    except Error as e:
      ser_connect = False
      print(e)  

def safeareaCb(data):
  global safearea_msg
  data_string = data.data
  safearea_msg.data = data_string
  json_object = json.loads(data_string) 

def robot_stateCb(data):
  global LED_State,Last_LED_State, Vx, Wz, Relay
  
  Last_LED_State = LED_State
  if data.data == 'READY':
    LED_State = 1
  elif data.data == 'DRIVING':
    LED_State = 2
  elif data.data == 'OBSTACLE_STOP':
    LED_State = 3    
  elif data.data == 'ARRIVE':
    LED_State = 4
  elif data.data == 'BATTERY_UNDER10':
    LED_State = 5
  elif data.data == 'ERROR':
    LED_State = 6  

  if LED_State != Last_LED_State:
    Vxr = bytearray(struct.pack("f", Vx))
    Wzr = bytearray(struct.pack("f", Wz))    
    print(LED_State)
    try:
      ser = serial.Serial('/dev/ttyS4', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
      values = bytearray([0x03, Vxr[0], Vxr[1], Vxr[2], Vxr[3],  Wzr[0],Wzr[1],Wzr[2],Wzr[3], LED_State, Relay])
      checksum1 =0
      for x in range(11):
        checksum1 += values[x] 
        checksum1 = checksum1 % 256  
      values1 = bytearray([0x03, Vxr[0], Vxr[1], Vxr[2], Vxr[3],  Wzr[0],Wzr[1],Wzr[2],Wzr[3], LED_State, Relay, checksum1])  
      # print([ "0x%02x" % b for b in values ])
      ser.write(values1)
      ser_connect = True
    except Error as e:
      ser_connect = False
      print(e)  


def serialcomming(data_in):
    global odom_pub, cnt2, prev_x, prev_y, prev_yaw, init, odom, t
    checksum = 0x00
    for x in range(22):
      checksum += data_in[x] 
    checksum = checksum % 256  
    if checksum == data_in[22]:  
      cnt2= cnt2+1

    #/*Board -> PC: 20Hz 20byte*/
    #/*-----------------------------------------------------------------------------------------
    #|  start  | emergency |    Px    |    Py     |     Yaw     | Voltage     |  checksum | end
    #-------------------------------------------------------------------------------------------
    #| 1 bytes |  1 bytes  | 4 bytes  |  4 bytes  |   4 bytes   |  4 bytes    | 1 byte    | 1 byte
    #------------------------------------------------------------------------------------------
    #|     0   |      1    | 2 3 4 5  | 6 7 8 9   | 10 11 12 13 | 14 15 16 17 |    18     |  19 
    #-------------------------------------------------------------------------------------------*/

      # Emergency
      Emergency = data_in[1]
      # Px
      dbt = bytearray([data_in[2],data_in[3],data_in[4],data_in[5]])
      Px = struct.unpack('f', dbt)
      # Py
      dbt = bytearray([data_in[6],data_in[7],data_in[8],data_in[9]])
      Py = struct.unpack('f', dbt)
      # yaw
      dbt = bytearray([data_in[10],data_in[11],data_in[12],data_in[13]])
      yawt = struct.unpack('f', dbt)
      yaw=yawt[0]


      New_Px = 0.0
      New_Py = 0.0
      dif = yaw - prev_yaw
      outlier = False

      if init == True:
        absdis = math.sqrt(((Px[0] - prev_x)*(Px[0] - prev_x))+((Py[0] - prev_y)*(Py[0] - prev_y)))
        if abs(dif) > 0.1 or absdis > 0.1:
          outlier = True 
        prev_yaw = yaw
        prev_x = Px[0]
        prev_y = Py[0]
        New_Px = Px[0]
        New_Py = Py[0]
      else:
        init = True
        prev_yaw = yaw
        prev_x = Px[0]
        prev_y = Py[0]
        New_Px = Px[0]
        New_Py = Py[0]

      #Battery Voltage
      dbt = bytearray([data_in[14],data_in[15],data_in[16],data_in[17]])
      Gy = struct.unpack('f', dbt)
      # print("Px:", Px[0], " Py:",Py[0]," Wz:", yawt[0]," Vb:", Gy[0])
      battery_msg.data = str(Gy[0])
      # print(str(Gy[0]))

      #Weight
      dwg = bytearray([data_in[18],data_in[19],data_in[20],data_in[21]])
      wt = struct.unpack('f', dwg)

      # print("Px:", Px[0], " Py:",Py[0]," Wz:", yawt[0]," Vb:", Gy[0]," Wt:", wt[0])
      # battery_msg.data = str(Gy[0])
      # print(str(wt[0]))

      if outlier == False:
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = "base_footprint"

        roll =0.0
        pitch = 0.0

        odom_quat = [0,0,0,0]
        odom_quat[0] = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        odom_quat[1] = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        odom_quat[2] = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        odom_quat[3] = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) 

        t.transform.translation.x = New_Px
        t.transform.translation.y = New_Py
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]

        

        
        odom.header.frame_id = "odom"
        odom.header.stamp = node.get_clock().now().to_msg()

        # set the position
        odom.pose.pose.position.x = New_Px
        odom.pose.pose.position.y = New_Py
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        odom_pub.publish(odom)
        node.tf_static_broadcaster.sendTransform(t)   

def main(args=None):
  global node, ser, ser_connect, cnt1, cnt, cnt2, odom_pub, Emergency_cmd
  global safearea_msg, acc_dec_msg, LED_State ,Last_LED_State, Vx, Wz, Relay
  global prev_yaw, init, odom, t

  prev_yaw = 0.0
  init = False
  Emergency_cmd = False

  rclpy.init(args=args)
  node = rclpy.create_node('motor_driver2')

  Robot_state_msg=String()
  # Robot_state_msg.data = "BOOTING"
  # publisher = node.create_publisher(String, 'robot_state', 10)
  # error_publisher = node.create_publisher(String, 'error_state', 10)

  VCU_state_publisher = node.create_publisher(String, 'vcu_error_state', 10) 
  battery_publisher = node.create_publisher(String, 'battery_voltage', 10)
  charging_publisher = node.create_publisher(String, 'charging_state', 10)
  motor_publisher = node.create_publisher(String, 'motor_state', 10)
  acc_dec_publisher = node.create_publisher(String, 'accel_decel_state', 10)
  safearea_publisher = node.create_publisher(String, 'safearea_state', 10)
  odom_pub = node.create_publisher(Odometry, '/odom', 1)

  node.create_subscription(Twist, '/cmd_vel', CmdVelCb,1)
  node.create_subscription(String, '/robot_state', robot_stateCb,1)
  node.create_subscription(String, '/emergency', robot_syscommandCb,1)
  node.create_subscription(String, '/brightness', robot_brightnessCb,1)
  
  with open(setting_location, 'r') as stream:
      data_loaded = yaml.safe_load(stream)

  Relay = int((data_loaded["LED_brighness"]/2)*45/100)

  Vx = 0.0
  Wz = 0.0
  Vxr = bytearray(struct.pack("f", Vx))
  # print([ "0x%02x" % b for b in Vxr ])
  Wzr = bytearray(struct.pack("f", Wz))
  try:
    LED_State = 1
    ser = serial.Serial('/dev/ttyS4', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
    values = bytearray([0x03, Vxr[0], Vxr[1], Vxr[2], Vxr[3],  Wzr[0],Wzr[1],Wzr[2],Wzr[3], LED_State, Relay])
    checksum1 =0
    for x in range(11):
      checksum1 += values[x] 
      checksum1 = checksum1 % 256  

    values1 = bytearray([0x03, Vxr[0], Vxr[1], Vxr[2], Vxr[3],  Wzr[0],Wzr[1],Wzr[2],Wzr[3], LED_State, Relay, checksum1])  
    # print([ "0x%02x" % b for b in values ])
    ser.write(values1)
    ser_connect = True
    sleep(0.1)
    ser.write(values1)
    sleep(0.1)
    ser.write(values1)
    
  except Error as e:
    ser_connect = False
    print(e)  


  # Last_robot_state = "OFF"
  # set_robot_state("BOOTING")   
  
  # now = node.get_clock().now()
  # prev = node.get_clock().now()



  vcu_msg.data = "OK"  #OK/ERROR
  battery_msg.data = "0.0" 
  charging_msg.data = "False" #True/False
  temp_json =   {
                  "motor_left": {
                        "current": "10.00",
                        "torque": "10.00"
                  },
                  "motor_right": {
                    "current": "10.00",
                    "torque": "10.00"
                  }
                }

  motor_msg.data = json.dumps(temp_json)
  acc_dec_msg.data = "{\"accel\": \"2\", \"decel\": \"2\"}"
  safearea_msg.data = "{\"length\": \"2\", \"width\": \"2\"}"
  
  temp_json_vcu = {
                "motor_left": "OK",
                "motor_right": "OK",
                "vcu": "OK",
                "imu": "OK",
              }
              
  vcustate = json.dumps(temp_json_vcu)


  now = node.get_clock().now()
  prev = now
  prev_test = now
  cnt = 0
  cnt2=0
  data_combine=[]
  data_start = False
  new_data_in = []

  node.tf_static_broadcaster = StaticTransformBroadcaster(node)
  print("start now")

  while rclpy.ok():
    now = node.get_clock().now()
    dur = now - prev   
    d =  dur.nanoseconds/1000000000
    #print(1)
    if d>0.25:
      prev = now
      vcu_msg.data = vcustate
      VCU_state_publisher.publish(vcu_msg)
      battery_publisher.publish(battery_msg)
      charging_publisher.publish(charging_msg)
      motor_publisher.publish(motor_msg)
      acc_dec_publisher.publish(acc_dec_msg)
      safearea_publisher.publish(safearea_msg)
      # error_state_publisher.publish(error_msg)

    dur = now - prev_test   
    d =  dur.nanoseconds/1000000000
    if d>0.033333333:  
      prev_test = now
      # print(cnt)
      # print("this every second ",cnt," ", cnt1," ", cnt2)
      cnt=0
      cnt1=0
      cnt2=0
      # odom_pub.publish(odom)
      # node.tf_static_broadcaster.sendTransform(t)    


    if ser_connect == True:
      # ser = serial.Serial('/dev/ttyS4', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
      ser.timeout=0.05
      data_intre = ser.read(24)
      # print(len(data_intre))
      print(data_intre) 
      for c in data_intre:
        if data_start:
          new_data_in.append(c)
          if len(new_data_in)>=24:
            # print(new_data_in)
            cnt1= cnt1+1
            data_start= False
            serialcomming(new_data_in)
            break
        if c == 0x03 and not data_start:
          data_start = True
          new_data_in=[]
          new_data_in.append(c)
         

    rclpy.spin_once(node, timeout_sec=0)
  

if __name__ == "__main__":
    main()
