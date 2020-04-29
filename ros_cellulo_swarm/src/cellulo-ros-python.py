#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is a simple example of a Webots controller running a Python ROS node thanks to rospy.
The robot is publishing the value of its front distance sensor and receving motor commands (velocity).
"""

import rospy
from std_msgs.msg import Float64, Empty, String
import geometry_msgs.msg
from geometry_msgs.msg import Vector3
import os
import math 
import csv
import numpy as np
import random 
import tf2_ros
import tf
import sys
from visualization_msgs.msg import Marker

class Cellulo ():

    vx=0
    vy=0
    w=0

    vxMeasured=0
    vyMeasured=0
    wMeasured=0
    
    # Pose 
    poseThetaRad=0
    poseX=0
    poseY=0
    
    prevPoseThetaRad=0
    prevPoseX=0
    prevPoseY=0

    timestamp=0
        
    def __init__(self):

        self.timeStep = 0.1 # 100 ms 
        self.timestamp= 0
        self.name = sys.argv[1]
        self.poseX= float(sys.argv[2])
        self.poseY= float(sys.argv[3])
        
        self.poseThetaRad = 0
        # self.poseX = self.translation[0]*1000  #scale m to mm 
        # self.poseY = self.translation[1]*1000  #scale m to mm
        
        self.prevPoseX = self.poseX
        self.prevPoseY = self.poseY
        self.prevPoseThetaRad = self.poseThetaRad
        
        #Subscriber: 
        self.velocitySubscriber = rospy.Subscriber("/cellulo_node_"+self.name+'/setGoalVelocity', Vector3, self.velocityCallBack)
        #Publisher: 
        self.velocityPublisher = rospy.Publisher("/cellulo_node_"+self.name+'/velocity',Vector3,queue_size=10)
        self.publisher_marker_robot=rospy.Publisher("/cellulo_node_"+self.name+'/marker',Marker,queue_size=10)
        self.touchPublisher=rospy.Publisher("/cellulo_node_"+self.name+'/longTouchKey', String, queue_size=10)

    def setPoseFromVelGoal(self):
        self.poseX=self.poseX+self.timeStep*self.vx
        self.poseY=self.poseY+self.timeStep*self.vy
        #print(self.vx,self.vy,self.poseX,self.poseY)
         

    def velocityCallBack(self,v):
        #print('Received velocity value: ' + str(v))
        self.vx = v.x
        self.vy = v.y
        self.w = v.z
    
    def estimateVelocities(self):
        #Calculate the raw values
        self.vxMeasured = (self.poseX - self.prevPoseX)*1000/self.timeStep
        self.vyMeasured = (self.poseY - self.prevPoseY)*1000/self.timeStep
        self.wMeasured = self.poseThetaRad  - self.prevPoseThetaRad
        
        #print("++++++++++++++++++"+str(self.poseX)+"                          "+str(self.prevPoseX))
        if(self.wMeasured < -math.pi):
            self.wMeasured += 2*math.pi
        elif(self.wMeasured > math.pi):
            self.wMeasured -= 2*math.pi
      
        self.wMeasured = self.wMeasured*1000/self.timeStep
  
        self.prevPoseX = self.poseX
        self.prevPoseY = self.poseY
        self.prevPoseThetaRad = self.poseThetaRad
        vel=Vector3(self.vx,self.vy,self.w)
        self.velocityPublisher.publish(vel)
            
    def updatePose(self):

        transformStamped = geometry_msgs.msg.TransformStamped()
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = "paper_world"
        transformStamped.child_frame_id = self.name
        
        transformStamped.transform.translation.x = self.poseX
        transformStamped.transform.translation.y = self.poseY
        transformStamped.transform.translation.z = 0
        
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        transformStamped.transform.rotation.x = quat[0]
        transformStamped.transform.rotation.y = quat[1]
        transformStamped.transform.rotation.z = quat[2]
        transformStamped.transform.rotation.w = quat[3]

        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(transformStamped)


        mrobot=Marker()
        mrobot.type = Marker.MESH_RESOURCE
        mrobot.header.frame_id = self.name
        mrobot.scale.x = 1
        mrobot.scale.y = 1
        mrobot.scale.z = 1
        mrobot.lifetime = rospy.Duration(20)
        mrobot.header.stamp = rospy.Time.now()
        mrobot.action = Marker.ADD
        mrobot.ns = "ros_cellulo"
        mrobot.color.r = 0.8
        mrobot.color.g = 0.8
        mrobot.color.b = 0.8
        mrobot.color.a = 1.0

        mrobot.pose.orientation.w = 1.0
        mrobot.pose.position.x = 0
        mrobot.pose.position.y = 0
        mrobot.pose.position.z = 0

        mrobot.mesh_resource = "package://ros_cellulo_swarm/meshes/cellulo.STL"
        self.publisher_marker_robot.publish(mrobot)
                     
robot=Cellulo()
message = ''
rospy.init_node('cellulo', anonymous=True)
robot.name = rospy.get_param('~macAddr', '1')
print('Running the control loop:'+robot.name)          
### ros rate
r = rospy.Rate(1/robot.timeStep)  
with open("poses_"+robot.name+'.csv', mode='w') as poses_file:  
    poses_file = csv.writer(poses_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    poses_file.writerow(['timestep','poseX','poseY','poseThetaRad','vx','vy','w'])
    while not rospy.is_shutdown():
        robot.timestamp+=robot.timeStep
        robot.updatePose()
        robot.setPoseFromVelGoal()
        robot.estimateVelocities()
        poses_file.writerow([robot.timeStep,robot.poseX,robot.poseY,robot.poseThetaRad,robot.vxMeasured, robot.vyMeasured,robot.wMeasured])
        
        r.sleep()
    
