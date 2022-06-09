# TurtleBot3 navigate to goal
# ref: https://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import transforms3d as t3d
import numpy as np
import math
import time

class Pose:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
    def update_pose(self, x=0,y=0,theta=0):
        self.x = x
        self.y = y
        self.theta = theta

class PID:
    def __init__(self, k_p=0, k_i=0, k_d=0):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.reset()
    
    def reset(self):
        """ reset stored error and time values """
        self.e_sum = 0.0
        self.e_prev = 0.0
        self.t_prev = time.time()

    def get_control(self, e):
        """ obtain control input corresponding to the current error """
        t_now = time.time()
        t_del = t_now - self.t_prev
        self.t_prev = t_now
        self.e_sum += e*t_del
        e_del = e - self.e_prev
        self.e_prev = e

        control = self.k_p*e + self.k_i*self.e_sum + self.k_d*e_del/t_del
        return control

class TurtleAgent:
    def __init__(self, k = np.array([[0.75, 0., 0.],[1.25, 0., 0.]])):
        rospy.init_node("tb3_goal", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.PID_lin = PID(k_p=k[0,0], k_i=k[0,1], k_d=k[0,2])
        self.PID_ang = PID(k_p=k[1,0], k_i=k[1,1], k_d=k[1,2])

    def wrap_angle(self, theta):
        """ wraps angles to the range [-PI,PI] """
        while theta > math.pi:
            theta = theta - 2*math.pi
        while theta < -1*math.pi:
            theta = theta + 2*math.pi
        return theta

    def update_pose(self, msg):
        """callback function from subscriber to update pose """
        self.pose.x = round(msg.pose.pose.position.x, 3)
        self.pose.y = round(msg.pose.pose.position.y, 3)
        
        o_x = msg.pose.pose.orientation.x
        o_y = msg.pose.pose.orientation.y
        o_z = msg.pose.pose.orientation.z
        o_w = msg.pose.pose.orientation.w
        q = [o_x, o_y, o_z, o_w]
        E = t3d.euler.quat2euler(q)
        self.pose.theta = round(E[0], 3)
        self.pose.theta = self.wrap_angle(self.pose.theta)

    def stop_bot(self):
        """ send 0 velocity commands to stop bot """
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.pub.publish(vel_msg)

    def clip_vel(self, lin_vel, ang_vel, thresh = 0.90, max_lin=0.22, max_ang=2.84):
        """ clip velocity commands sent to the bot"""
        eff_max_lin = thresh * max_lin
        eff_max_ang = thresh * max_ang
        if lin_vel > eff_max_lin:
            lin_vel = eff_max_lin
        elif lin_vel < -1*eff_max_lin:
            lin_vel = -1*eff_max_lin
        if ang_vel > eff_max_ang:
            ang_vel = eff_max_ang
        elif ang_vel < -1*eff_max_ang:
            ang_vel = -1*eff_max_ang
        return lin_vel, ang_vel

    def dist_to_goal(self, goal_pose):
        """ compute L2 norm to goal from current pose """
        return np.linalg.norm([goal_pose.x - self.pose.x, \
            goal_pose.y - self.pose.y]) 
            
    def get_vel(self, goal_pose):
        """ compute velocity input to TurtleBot to reach goal """
        er_lin = self.dist_to_goal(goal_pose)
        er_ang = math.atan2(goal_pose.y - self.pose.y, \
            goal_pose.x - self.pose.x) - self.pose.theta
        er_ang = self.wrap_angle(er_ang)

        lin_vel = self.PID_lin.get_control(er_lin)
        ang_vel = self.PID_ang.get_control(er_ang)
        lin_vel, ang_vel = self.clip_vel(lin_vel, ang_vel)

        vel_msg = Twist()
        vel_msg.linear.x = lin_vel
        vel_msg.angular.z = ang_vel
        return vel_msg
        
    def go_to_goal(self):
        """ navigate TurtleBot to reach goal """
        print("Input desired goal coordinates")
        goal_pose = Pose()
        goal_pose.update_pose(round(float(input("x: ")), 3), 
            round(float(input("y: ")), 3))
        tol = round(float(input("Tolerance: ")), 3)

        while self.dist_to_goal(goal_pose) >= tol and not rospy.is_shutdown():
            vel_msg = self.get_vel(goal_pose)
            self.pub.publish(vel_msg)
            self.rate.sleep()
        if self.dist_to_goal(goal_pose) < tol:
            self.stop_bot()
            print("Goal reached.")

    def test_pub(self):
        """ test publisher by sending constant vel """
        vel_msg = Twist()
        vel_msg.linear.x = 0.1
        vel_msg.angular.z = 0.1
        self.pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        agent = TurtleAgent()
        agent.go_to_goal()
    except rospy.ROSInterruptException:
        pass