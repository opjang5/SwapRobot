#!/usr/bin/env python
# coding:utf-8
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from math import pi
import time
import threading
import math
from actionlib_msgs.msg import *
waypoints=[
    [(0,0,0.0),(0.0,0.0,0)],
    [(0,0.05,0.0),(0.0,0.0,0)],
    [(0,0.10,0.0),(0.0,0.0,0)],
    [(0,0.15,0.0),(0.0,0.0,0)]
]
class Robot(threading.Thread):
    def __init__(self,client):
        threading.Thread.__init__(self)
        self.waypos = [(0.0,0.0,0.0),(0.0,0.0,0.0)]
        self.client = client
        self.stopFlag = False;
        self.tf_listener = tf.TransformListener()
	self.LastTime = 0
	self.LastYFlag = 1
	self.Zoom = 2.5
	self.changeCounter = 0
	self.endMappingFlag = 0
	self.endWayFlag = 0
	self.endZoom = 1
	self.result = True
	self.endCount = 0
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

    def get_pos(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
        euler = tf.transformations.euler_from_quaternion(rot)
        #print euler[2] / pi * 180

        x = trans[0]
        y = trans[1]
        th = euler[2] / pi * 180
        return (x, y, th)

    def stop(self):
        self.stopFlag = True

    def run(self):
        lastTime = time.time()
        lastPos = self.get_pos()
	startFlag = 0
        while (self.stopFlag == False):
            Now = time.time()
            NowPos = self.get_pos()
            duration = Now - lastTime    
            if ((duration)>=1.0):
                lastTime = Now
                speed = [0,0]
                speed[0] = math.sqrt(math.pow(NowPos[0]-lastPos[0],2)+math.pow(NowPos[1]-lastPos[1],2))/duration
                speed[1] = (NowPos[1]-lastPos[1])/duration
                distance = math.sqrt(math.pow(NowPos[0]-self.waypos[0][0],2)+math.pow(NowPos[1]-self.waypos[0][1],2))
                lastPos = NowPos
		#print(distance)
		print(Now-self.LastTime)
                if (((math.fabs(speed[0]+speed[1])<=1e-3 and Now-self.LastTime>=2 and Now-self.LastTime<=30) or startFlag == 0 or (math.fabs(speed[0])<=1e-3 and Now-self.LastTime<90 and Now-self.LastTime>=2)) and self.endMappingFlag == 0):
      		    self.LastTime = Now
		    if(Now-self.LastTime<2.5 or self.result==False):
			self.changeCounter = self.changeCounter + 1
                    	if(self.LastYFlag == 0):
				self.LastYFlag = 1
			else:
				self.LastYFlag = 0
		    if(self.changeCounter!=0 and self.changeCounter%2 == 0):
			self.Zoom = self.Zoom + 1
		    self.waypos[0] = (self.waypos[0][0]+self.Zoom*(1-self.LastYFlag),self.waypos[0][1]+self.Zoom*self.LastYFlag,self.waypos[0][2])
                    print(self.waypos)
		    print('Next point')
		    startFlag = 1
                    goal=goal_pose(self.waypos)
                    self.client.send_goal(goal)
		    #success = self.client.wait_for_result(rospy.Duration(15))
		    state = self.client.get_state() 
		    self.result = False
	  	    if state == GoalStatus.SUCCEEDED:
                    	# We made it!
            		self.result = True
                    #res = self.client.wait_for_result()
	            print(self.result)
		elif(Now-self.LastTime>=90 or (self.endMappingFlag == 1 and Now-self.LastTime>2.0)):
		    self.endMappingFlag = 1
	            if(Now-self.LastTime>=60 or (math.fabs(speed[0])<=1e-3)):
			self.endCount = self.endCount + 1
			self.LastTime = Now
			self.client.cancel_goal()
			if(self.endWayFlag == 0):
				self.waypos[0] = (-self.waypos[0][0]*self.endZoom,self.waypos[0][1]*self.endZoom,self.waypos[0][2])
			elif(self.endWayFlag == 1):
				self.waypos[0] = (self.waypos[0][0]*self.endZoom,-self.waypos[0][1]*self.endZoom,self.waypos[0][2])
			elif(self.endWayFlag == 2):
				self.waypos[0] = (-self.waypos[0][0]*self.endZoom,self.waypos[0][1]*self.endZoom,self.waypos[0][2])
			elif(self.endWayFlag == 3):
				self.waypos[0] = (self.waypos[0][0]*self.endZoom,-self.waypos[0][1]*self.endZoom,self.waypos[0][2])
			self.endWayFlag = self.endWayFlag + 1
			if(self.endWayFlag>=4):
				self.endWayFlag = 0
			self.endZoom = self.endZoom*0.98
			print(self.waypos)
			goal=goal_pose(self.waypos)
			self.client.send_goal(goal)
		    if(math.fabs(speed[0])<=1e-3 and self.endCount>=5):
			os.system('rosrun map_server map_saver -f /home/denny/catkin_ws/src/ROS-Academy-for-Beginners/slam_sim_demo/scripts/my_map/')
			self.client.cancel_goal()
			print('End!!!!!!!!!!!!!!')
			break
                #print(speed)
                #print(distance)
            time.sleep(1.0)

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id="map"
    goal_pose.target_pose.pose.position.x=pose[0][0]
    goal_pose.target_pose.pose.position.y=pose[0][1]
    goal_pose.target_pose.pose.position.z=pose[0][2]
    # r, p, y  欧拉角转四元数
    x,y,z,w=tf.transformations.quaternion_from_euler(pose[1][0],pose[1][1],pose[1][2])
    goal_pose.target_pose.pose.orientation.x=x
    goal_pose.target_pose.pose.orientation.y=y
    goal_pose.target_pose.pose.orientation.z=z
    goal_pose.target_pose.pose.orientation.w=w
    return goal_pose
if __name__ == '__main__':
    print("正在初始化......")
    #节点初始化
    rospy.init_node('patrol')
    #创建MoveBaseAction client
    client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #等待MoveBaseAction server启动
    client.wait_for_server()
    robot = Robot(client)
    robot.start()
    print("启动成功!!!")
    while not rospy.is_shutdown():
        pass

