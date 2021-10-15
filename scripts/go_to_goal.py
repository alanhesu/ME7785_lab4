#!/usr/bin/env python
# license removed for brevity
import rospy
from greenbutter_chase_object.msg import ObjAngle
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np

direct = 0

class pid_controller:
    def __init__(self, kp, ki, kd, umax=np.inf):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.accum = 0.0
        self.e_old = 0.0
        self.t_old = rospy.get_time()
        self.umax = umax

    def get_response(self, e):
        t = rospy.get_time()
        dt = t - self.t_old
        e_dot = e - self.e_old
        self.accum += e*dt

        rospy.loginfo('e: {:.4f} p: {:.4f} i: {:.4f} d: {:.4f}'.format(e, self.kp*e, self.ki*self.accum, self.kd*e_dot/dt))
        rval = self.kp*e + self.ki*self.accum + self.kd*e_dot/dt

        # deal with integral windup by not adding to the accumulator
        if (rval > self.umax):
            self.accum -= e*dt

        self.t_old = t
        self.e_old = e
        return rval

    def reset_t_old(self):
        self.t_old = rospy.get_time()

    def reset_accum(self):
        self.t_old = rospy.get_time()
        self.accum = 0.0
        self.e_old = 0.0

def callback_vel(data):
    global dist
    global velocity_cmd
    global vel_pid
    global detected
    e = data.data - .22

    if (not detected):
        print('you shouldnt be here')
        vel_pid.reset_accum() # Reset accumulated values and t_old
        return
    velocity_cmd = vel_pid.get_response(e)

def callback_direct(data):
    global angle_cmd
    global detected
    global angle_pid
    global vel_pid

    e = data.theta
    detected = data.detected
    if (not detected):
        vel_pid.reset_accum() # Reset accumulated values and t_old
        angle_pid.reset_accum()
        return
    angle_cmd = angle_pid.get_response(e)

def callback_killvel():
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    pub.publish(vel_msg)

def cmd_vel():
    rospy.init_node('chase_object', anonymous=True)

    global pub
    global vel_msg
    global velocity_cmd
    velocity_cmd = 0
    global angle_cmd
    angle_cmd = 0
    global detected
    detected = False
    global vel_pid
    global angle_pid
    vel_pid = pid_controller(.3, .05, .0, umax=0.4)
    angle_pid = pid_controller(2, .5, .5)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rate = rospy.Rate(20) # 20hz

    direct_sub = rospy.Subscriber("/detected_angle", ObjAngle, callback_direct)
    range_sub = rospy.Subscriber("/object_range", Float32, callback_vel)

    vel_msg = Twist()
    while not rospy.is_shutdown():
        if (not detected):
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.0
        else:
            vel_msg.angular.z = angle_cmd
            vel_msg.linear.x = velocity_cmd

        # print('command: {} {}'.format(velocity_cmd, angle_cmd))
        pub.publish(vel_msg)
        rate.sleep()
        rospy.on_shutdown(callback_killvel)


if __name__ == '__main__':
    try:
        cmd_vel()
    except rospy.ROSInterruptException:
        pass
