#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import numpy as np
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

#LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 75 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5

class WaypointUpdater(object):
    def __init__(self):
        #rospy.init_node('waypoint_updater')
        rospy.init_node('waypoint_updater',log_level=rospy.DEBUG)

        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx  = -1
        self.waypoints_2d = None
        self.waypoint_tree = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # no need to add /obstacle_waypoint subscriber since there will be no traffic
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()


    def loop(self):
        #rate = rospy.Rate(50)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        # co-ordinates of the car
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # query for 1 waypoint that is closest to the car
        result = self.waypoint_tree.query([x, y], 1)
        closest_dist = result[0]
        closest_idx = result[1]

        # check if the closest point in ahead or behind the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # We have the closest and the previous to closest co-ordinates now
        # we will construct vectors out of these and decide whether the waypoint is
        # ahead or behind depending on the dot product being positive or negative
        cl_vector = np.array(closest_coord)
        prev_vector = np.array(prev_coord)
        pos_vector = np.array([x,y])

        val = np.dot((cl_vector - prev_vector), (pos_vector - cl_vector))

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx


    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        #sdevikar: experimental
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        #rospy.logdebug('\nsdevikar: generate_lane called - closest_idx:%d, farthest_idx:%d, stopline_wp_idx:%d', closest_idx, farthest_idx, self.stopline_wp_idx)
        # keep the base waypoints as final_waypoints if the traffic light is
        # not in sight or too far
        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            # or else, decelerate_waypoint velocities accordingly
            #rospy.logdebug('\nsdevikar: decelerating!!')
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        #rospy.logdebug('\nsdevikar: Enter decelerate_waypoints with waypoints length:%d ', len(waypoints))
        temp = []
        for i, wp in enumerate(waypoints):

            p = Waypoint()
	        #position of the waypoint stays the same as base waypoint
            p.pose = wp.pose

            # calculate a stop waypoint so that car's nose stops at the stop waypoint
            wpts_count_before_stopline = self.stopline_wp_idx - closest_idx
            wpts_count_before_stopline_to_nose = wpts_count_before_stopline - 3
            stop_idx = max(wpts_count_before_stopline_to_nose, 0)

            #rospy.logdebug('\nsdevikar: stop index turns out to be:%d ', stop_idx)
            #rospy.logdebug('\nsdevikar: calculating distance between wp1:%d and wp2:%d ', i, stop_idx)


            #distance between current waypoint and intended stop line waypoint
            dist = self.distance(waypoints, i, stop_idx)

            # calculate velocity for this waypoint by factoring in the distance
            # to stop waypoint and the desired deceleration
            vel = math.sqrt(2 * MAX_DECEL * dist)

            if vel < 1.:
                vel = 0.

            # if the calculated velocity goes above what the car is driving at,
            # no need to update the velocity
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

            temp.append(p)

        return temp


    def pose_cb(self, msg):
        # TODO: Implement
        # this is called at the rate of ~50hz
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #rospy.logdebug('\nsdevikar: Enter waypoints_cb. Number of waypoints: %d',len(waypoints.waypoints))
        self.base_lane = waypoints
        if not self.waypoints_2d:
            # construct a list of 2d co-ordinates from the waypoints
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        #rospy.logdebug('\nsdevikar: logdebug: Enter traffic_cb')
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
