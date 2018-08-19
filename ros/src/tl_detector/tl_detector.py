#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import yaml
import os
import calendar
import time


STATE_COUNT_THRESHOLD = 2
GENERATE_TRAIN_IMGS = False

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []
        self.is_site = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        # Consider changing subscription to 'image_raw' for processing instead
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.is_site = self.config['is_site']
        self.is_simulator = not self.is_site

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):

        self.lights = msg.lights

    def create_training_data(self, state):
        f_name = "sim_tl_{}_{}.jpg".format(calendar.timegm(time.gmtime()), self.light_label(state))
        dir = './data/train/sim'

        if not os.path.exists(dir):
            os.makedirs(dir)

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image)
        cv_image = cv_image[:, :, ::-1]
        cv2.imwrite('{}/{}'.format(dir, f_name), cv_image)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        rospy.loginfo("=== image_cb() =============================")
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state

            if GENERATE_TRAIN_IMGS:
                # Store images and state for training data for simulator
                self.create_training_data(state)

            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            rospy.loginfo("publish upcoming red light wp index: {}".format(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            rospy.loginfo("publish upcoming red light wp index: {}".format(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            x: x-position to match a waypoint to
            y: y-position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # Assumes waypoint_tree (KDTree) is already created in waypoint_cb()
        closest_wp_indx = self.waypoint_tree.query([x, y], 1)[1]
        #rospy.loginfo('Closest waypoint index to car position (%s, %s): (%s)',
        #                x, y, closest_wp_indx)

        return closest_wp_indx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # THIS BLOCK IS FOR TESTING ONLY - using light state provided
        # from subscription to /vehicle/traffic_lights; this traffic light
        # state is available only in simulator and will NOT be available
        # in real live test track
        #rospy.loginfo('Light state: %s', light.state)
        #return light.state



        ### UNCOMMENT THIS BLOCK FOR REAL LIGHT CLASSIFICATION ###
        ### WHEN CLASSIFIER IS AVAILABLE                       ###
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, 'rgb8')
        #cv_image = cv_image[:, :, ::-1] # switch layers B and R from BGR to RGB
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        classified_state = self.light_classifier.get_classification(cv_image)

        if self.is_simulator:
            rospy.loginfo("Sim ground truth state: {}".format(
                self.light_label(light.state)))

        rospy.loginfo("Classified state:       {}".format(
            self.light_label(classified_state)))

        return classified_state

    def light_label(self, state):
        if state == TrafficLight.RED:
            return "RED"
        elif state == TrafficLight.YELLOW:
            return "YELLOW"
        elif state == TrafficLight.GREEN:
            return "GREEN"
        return "UNKNOWN"

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_upcoming_light = None
        stop_line_wp_indx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_indx = self.get_closest_waypoint(self.pose.pose.position.x,
                                                    self.pose.pose.position.y)

        #rospy.loginfo("== process_traffic_lights() ==================")
        #Find the closest visible traffic light (if one exists)
        indx_dist = len(self.waypoints.waypoints)
        for i, light in enumerate(self.lights):
            # Get stop line waypoint index
            stop_line_pose = stop_line_positions[i]
            wp_indx = self.get_closest_waypoint(stop_line_pose[0],
                                                stop_line_pose[1])
            # Find the closest stop line waypoint index
            d = wp_indx - car_wp_indx

            #rospy.loginfo("light: {}, car_wp_indx: {}, wp_indx: {}, d: {}".format(
            #    i, car_wp_indx, wp_indx, d))
            if d >= 0 and d < indx_dist:
                indx_dist = d
                closest_upcoming_light = light
                stop_line_wp_indx = wp_indx

        if closest_upcoming_light:
            state = self.get_light_state(closest_upcoming_light)
            rospy.loginfo(">> closest stop_line_wp_indx: {}, state: {}".format(
                stop_line_wp_indx, self.light_label(state)))

            return stop_line_wp_indx, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
