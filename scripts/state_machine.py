#!/usr/bin/env python
import rospy
import smach
import smach_ros
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from greenbutter_navigate_to_goal.msg import ObjDetect
import threading
# import pid_controller
from pid_controller import pid_controller
from Rotation_Script import update_Odometry

waypoints = np.array([[1.5, 0], \
                    [1.5, 1.4], \
                    [0.0, 1.4], \
                    [0.0, 0.0]])
# waypoints = np.array([[1.5, 0], [1.5, 1.5]])

vel_msg = Twist()
glob_position = Point()
curgoal = 0

def get_dist(p1, p2):
    """return the euclidean distance between two 2d points"""
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def quat2euler(q):
    return np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def wrap_angle(theta):
    if (theta > np.pi):
        theta -= 2*np.pi
    if (theta < -np.pi):
        theta += 2*np.pi
    return theta

def zero_vel():
    global vel_msg
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    return vel_msg

def reset_odom():
    reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    time_beg = rospy.get_time()
    while rospy.get_time() - time_beg < 0.5:
        reset_odom.publish(Empty())

def theta_diff(th1, th2):
    diff = th2 - th1
    if (diff > np.pi):
        diff -= 2*np.pi
    elif (diff < -np.pi):
        diff += 2*np.pi

    return diff

def callback_killvel():
    global vel_pub
    global vel_msg
    zero_vel()
    vel_pub.publish(vel_msg)

def callback_odom(data):
    global glob_position
    global glob_theta
    glob_position = data.pose.pose.position
    glob_theta = quat2euler(data.pose.pose.orientation)

def callback_detect(data):
    global obj_detect
    obj_detect = data

class go_to_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['follow', 'avoid', 'stop', 'finish'], output_keys=['follow_direct'])
        self.dist_thresh = .05

    def execute(self, userdata):
        global curgoal
        global waypoints
        global rate
        print('state go_to_goal {}'.format(waypoints[curgoal,:]))
        #@TODO: implement go to waypoints

        res = self.go_waypoint(waypoints[curgoal,:], vel_pub, rate)
        if (res == 'detected'):
            userdata.follow_direct = 1
            return 'follow'
        # reached a waypoint - stop and increment to next waypoint or finish
        curgoal += 1
        if (curgoal >= len(waypoints[:,0])):
            return 'finish'
        else:
            return 'stop'

    def go_waypoint(self, wp, vel_pub, rate):
        global glob_position
        global glob_theta
        global vel_msg
        global obj_detect
        vel_pid = pid_controller(.3, .05, .0, umax=0.1, max_cmd=0.1)
        angle_pid = pid_controller(.75, 0.002, .1)
        e_dist = get_dist(wp, [glob_position.x, glob_position.y])

        # get angle error
        # calculate angle of vector to waypoint in global frame
        dx = wp[0] - glob_position.x
        dy = wp[1] - glob_position.y
        theta_wp = np.arctan2(dy, dx)
        e_angle = theta_diff(glob_theta, theta_wp)
        #@TODO: implement better stop condition?
        while (e_dist >= self.dist_thresh):
            rate.sleep()

            e_dist = get_dist(wp, [glob_position.x, glob_position.y])
            vel_cmd = vel_pid.get_response(e_dist)
            # print('error dist: {} {}'.format(e_dist, vel_cmd))

            dx = wp[0] - glob_position.x
            dy = wp[1] - glob_position.y
            theta_wp = np.arctan2(dy, dx)
            e_angle = theta_diff(glob_theta, theta_wp)
            # print('{}, {}, {}'.format(theta_wp, glob_theta, e_angle))
            # if (e_dist < 0.5):
                # angle_pid.set_kp(min(4, e_dist*0.3))
            # else:
                # angle_pid.set_kp(4)
            angle_cmd = angle_pid.get_response(e_angle)

            #@TODO: implement angle control
            #@TODO: return early if object detected
            vel_msg.linear.x = vel_cmd
            vel_msg.angular.z = angle_cmd
            vel_pub.publish(vel_msg)

            if (obj_detect.cone_detected):
                return 'detected'

class stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go'])

    def execute(self, userdata):
        global rate
        global vel_pub
        print('state stop')
        #@TODO: implement stop at waypoint for 10s
        # wait for 10 seconds to elapse
        t_beg = rospy.get_time()
        vel_pub.publish(zero_vel())
        while rospy.get_time() - t_beg < 2:
            rate.sleep()
        return 'go'

# class pause(smach.State):
    # def __init__(self):
        # smach.State.__init__(self, outcomes=['go', 'follow', 'avoid'])

    # def execute(self, userdata):
        # global rate
        # global obj_detect
        # while (obj_detect.cone_detected):
            # rate.sleep()
        # return 'go'

class follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go', 'avoid'], input_keys=['follow_direct'])

    def execute(self, userdata):
        global rate
        self.direct = userdata.follow_direct
        print('direct = {}'.format(self.direct))
        #@TODO:
        res = self.follow_wall(waypoints[curgoal,:], vel_pub, rate)
        if (res == 'exit'):
            return 'go'

    def follow_wall(self, wp, vel_pub, rate):
        global vel_msg
        angle_pid = pid_controller(1.0, 0, .1)

        vel_msg.linear.x = 0.05
        while (True):
            rate.sleep()

            e_angle = self.get_angle_error()
            angle_cmd = angle_pid.get_response(e_angle)
            vel_msg.angular.z = angle_cmd
            vel_pub.publish(vel_msg)
            if self.exit_follow(wp):
                return 'exit'

    def get_angle_error(self):
        angle_obj = obj_detect.th1
        angle_goal = angle_obj + np.sign(self.direct)*np.pi/2
        angle_goal = wrap_angle(angle_goal)

        e_angle = angle_goal
        return e_angle

    def exit_follow(self, wp):
        angle_obj = obj_detect.th1
        dx = wp[0] - glob_position.x
        dy = wp[1] - glob_position.y
        angle_wp = np.arctan2(dy, dx)
        angle_obj += glob_theta
        angle_obj = wrap_angle(angle_obj)
        print(angle_obj, angle_wp, np.abs(angle_wp - angle_obj))
        return np.abs(angle_wp - angle_obj) > np.pi/2

def main():
    global rate
    global vel_pub
    global vel_msg
    global obj_detect

    obj_detect = ObjDetect()
    rospy.init_node('state_machine', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    odom_sub = rospy.Subscriber("/odom", Odometry, callback_odom)
    obj_sub = rospy.Subscriber("/detected_obj", ObjDetect, callback_detect)
    rate = rospy.Rate(20)

    # reset odometry
    #@TODO: test?
    reset_odom()

    # construct state machine
    sm = smach.StateMachine(outcomes=['finish'])
    sm.userdata.follow_direct = 0

    with sm:
        smach.StateMachine.add('go_to_goal', go_to_goal(), transitions={'follow':'follow', 'avoid':'stop', 'stop':'stop'}, remapping={'follow_direct':'follow_direct'})
        smach.StateMachine.add('stop', stop(), transitions={'go':'go_to_goal'})
        # smach.StateMachine.add('pause', pause(), transitions={'go':'go_to_goal'})
        smach.StateMachine.add('follow', follow(), transitions={'go':'go_to_goal', 'avoid':'go_to_goal'}, remapping={'follow_direct':'follow_direct'})


    # introspection server
    # sis = smach_ros.IntrospectionServer('sm_server', sm, '/SM_ROOT')
    # sis.start()
    # outcome = sm.execute
    # rospy.spin()
    # sis.stop()

    # this threading stuff so ctrl-c will kill the state machine
    #@TODO: make this work
    # smach_thread = threading.Thread(target=sm.execute)
    # smach_thread.start()
    outcome = sm.execute()
    print('finish')
    vel_pub.publish(zero_vel())
    # rospy.spin()
    # sm.request_preempt()
    # smach_thread.join()
    rospy.on_shutdown(callback_killvel)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
