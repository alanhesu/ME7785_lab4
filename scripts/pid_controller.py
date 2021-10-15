import rospy
import numpy as np

class pid_controller:
    def __init__(self, kp, ki, kd, umax=np.inf, debug=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.accum = 0.0
        self.e_old = 0.0
        self.t_old = rospy.get_time()
        self.umax = umax
        self.debug = debug

    def get_response(self, e):
        t = rospy.get_time()
        dt = t - self.t_old
        e_dot = e - self.e_old
        self.accum += e*dt

        if (self.debug >= 1):
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
