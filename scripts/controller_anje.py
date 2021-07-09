#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
import math

import roslib
import rospy
import rospkg

from std_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

import tf
from tf.transformations import euler_from_quaternion

# for pure pursuit controller
from optional import get_target_point

# Calculate distance
def calc_dist(tx, ty, ix, iy):
    return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

# Normalize angle [-pi, +pi]
def normalize_angle(angle):
    if angle > math.pi:
        norm_angle = angle - 2*math.pi
    elif angle < -math.pi:
        norm_angle = angle + 2*math.pi
    else:
        norm_angle = angle
    return norm_angle

# Global2Local
def global2local(ego_x, ego_y, ego_yaw, x_list, y_list):
    # Translational transform
    x_list = np.array(x_list)
    y_list = np.array(y_list)
    x_list = x_list - ego_x
    y_list = y_list - ego_y

    # Rotational transform
    rot_theta = -ego_yaw
    c_theta = np.cos(rot_theta)
    s_theta = np.sin(rot_theta)

    rot_mat = np.array([[c_theta, -s_theta],
                        [s_theta, c_theta]])

    output_xy_list = np.matmul(rot_mat, np.array([x_list, y_list]))
    output_x_list = output_xy_list[0,:]
    output_y_list = output_xy_list[1,:]

    return output_x_list, output_y_list

# Find nearest point
def find_nearest_point(ego_x, ego_y, x_list, y_list):
    dist = np.zeros(len(x_list))
    for i in range(len(x_list)):
        dist[i] = calc_dist(x_list[i], y_list[i], ego_x, ego_y)
    
    near_ind = np.argmin(dist)
    near_dist = dist[near_ind]

    return near_dist, near_ind

# Calculate Error
def calc_error(ego_x, ego_y, ego_yaw, x_list, y_list, wpt_ind, wpt_look_ahead=0):
    # Global to Local coordinate
    local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)

    # Calculate yaw error
    target_wpt_ind = (wpt_ind + wpt_look_ahead)%x_list.shape[0] # look ahead
    error_yaw = math.atan2(local_y_list[(target_wpt_ind+1) % len(local_x_list)] - local_y_list[target_wpt_ind], \
                            local_x_list[(target_wpt_ind+1) % len(local_x_list)] - local_x_list[target_wpt_ind])
    # Calculate errors
    error_y   = local_y_list[target_wpt_ind]
    error_yaw = normalize_angle(error_yaw)

    return error_y, error_yaw

class WaypointFollower():
    def __init__(self):
        # ROS init
        rospy.init_node('wpt_follwer')
        self.rate = rospy.Rate(100.0)

        # Params
        self.target_speed = 10/3.6
        self.MAX_STEER    = np.deg2rad(17.75)

        # vehicle state
        self.ego_x   = 0
        self.ego_y   = 0
        self.ego_yaw = 0
        self.ego_vx  = 0

        self.wpt_look_ahead = 0   # [index]

        # Pub/Sub
        self.pub_command = rospy.Publisher('/control', AckermannDriveStamped, queue_size=5)
        self.sub_odom    = rospy.Subscriber('/simulation/bodyOdom', Odometry, self.callback_odom)

    def callback_odom(self, msg):
        """
        Subscribe Odometry message
        ref: http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        """
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vx = msg.twist.twist.linear.x
        # get euler from quaternion
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        _, _, self.ego_yaw = euler_from_quaternion(q_list)

    # Controller
    def steer_control(self, error_y, error_yaw, integral_prior_y,integral_prior_yaw, wpts):
        """
        Steering control
        TODO-1 : Tuning your steering controller (Currently, P controller is implemented.).
        TODO-2 : Implement PI controller for steering angle control.
        """

        """
        # P controller 1,0.3
        kp_y   = 1 # P gain w.r.t. cross track error
        kp_yaw = 0.3 # P gain w.r.t. yaw error
        
        steer = kp_y*error_y + kp_yaw*error_yaw 
        """

        ################################################
        # I term of PI controller
        """
        ki_y = 1
        ki_yaw = 0.3
        iteration_time = 0.1
        integral_y = integral_prior_y + error_y*iteration_time
        integral_yaw = integral_prior_yaw + error_yaw*iteration_time

        steer += ki_y*integral_y + ki_yaw*integral_yaw

        integral_prior_y = integral_y
        integral_prior_yaw = integral_yaw
        """
        #################################################
        """
        # Pure pursuit method
        k_ld = 0.1
        wpts_x = wpts[:,0]
        wpts_y = wpts[:,1]
        ld = k_ld*self.ego_vx
        local_wpts_x, local_wpts_y = global2local(self.ego_x, self.ego_y, self.ego_yaw, wpts_x, wpts_y)
        local_wpts = tuple(zip(local_wpts_x,local_wpts_y))
        target_pt = get_target_point(ld, local_wpts)

        if target_pt is None:
            return 0
        alpha = np.arctan2(target_pt[1], target_pt[0])

        steer = np.arctan((2*1*np.sin(alpha))/ld)
        """
        #####################################################
        
        # Stanley Control
        ks = 0.5
        steer = error_yaw + np.arctan2(ks*error_y,self.ego_vx)
        
        #################################################
        # Control limit
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)

        return steer,integral_prior_y,integral_prior_yaw

    def speed_control(self, error_v):
        """
        Speed control
        TODO-3: Tuning your speed controller (Currently, P controller is implemented.).
        """
        kp_v = 2
                
        return kp_v * error_v

    def publish_command(self, steer, accel):
        """
        Publish command as AckermannDriveStamped
        ref: http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
        """
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steer / np.deg2rad(17.75)
        msg.drive.acceleration = accel
        self.pub_command.publish(msg)

def main():
    # Load Waypoint
    rospack = rospkg.RosPack()
    WPT_CSV_PATH = rospack.get_path('waypoint_follower') + "/wpt_data/wpt_data_dense.csv"
    csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
    wpts = csv_data.values
    wpts_x = wpts[:,0]
    wpts_y = wpts[:,1]

    print("loaded wpt :", wpts_x.shape, wpts_y.shape)

    # Define controller
    wpt_control = WaypointFollower()
    
    ####################################
    # For I term of PI controller
    integral_prior_y = 0
    integral_prior_yaw = 0
    ####################################
    while not rospy.is_shutdown():
        # Get current state
        ego_x = wpt_control.ego_x
        ego_y = wpt_control.ego_y
        ego_yaw = wpt_control.ego_yaw
        ego_vx = wpt_control.ego_vx

        # Find the nearest waypoint
        _, near_ind = find_nearest_point(ego_x, ego_y, wpts_x, wpts_y)
        wpt_ind = near_ind

        # Lateral error calculation (cross-track error, yaw error)
        error_y, error_yaw = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, wpt_ind, 
                                         wpt_look_ahead=wpt_control.wpt_look_ahead)

        # Longitudinal error calculation (speed error)
        error_v = wpt_control.target_speed - ego_vx
        
        # Control
        steer_cmd,integral_prior_y,integral_prior_yaw = wpt_control.steer_control(error_y, error_yaw, 
                                                                                   integral_prior_y, 
                                                                                   integral_prior_yaw, 
                                                                                   wpts)
                                                             
        throttle_cmd = wpt_control.speed_control(error_v)

        # Publish command
        wpt_control.publish_command(steer_cmd, throttle_cmd)

        rospy.loginfo("Commands: (steer=%.3f, accel=%.3f). Errors: (CrossTrackError=%.3f, YawError=%.3f, SpeedError=%.3f)." 
                        %(steer_cmd, throttle_cmd, error_y, error_yaw, error_v))
        wpt_control.rate.sleep()


if __name__ == '__main__':
    main()