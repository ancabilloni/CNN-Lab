#!/usr/bin/env python

import rospy
from tf import transformations
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray

from math import cos, sin
from copy import deepcopy

FORWARD_SCAN_WPS = 200
MPS = 0.44704 # is 1 MPH
MIN_VEL = 0.001
MAX_VEL = 11.111
STOP = 0
GO = 1
DECEL = 2

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


class WaypointUpdater(object):
    """
    CONTROL SUBSYSTEM

    *** STEP 2 ***

    This node will publish waypoints from the car's current position to some `x` distance ahead,
    with the correct target velocities, depending on traffic lights and obstacles.
    """
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)  # Simulator data
        rospy.Subscriber('/traffic_waypoint', Int32)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        # Publishers
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_ego_pose = None  # ego car current position and orientation
        self.base_waypoints = None
        self.traffic_lights = None
        self.frame_id = None
        self.tlwp = 0
        self.action = GO

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.base_waypoints is None or self.current_ego_pose is None or self.frame_id is None:
                continue

            car_index = self.get_closest_waypoint_index(self.current_ego_pose, self.base_waypoints)

            # lookahead_waypoints = self.get_next_waypoints(self.base_waypoints, car_index, FORWARD_SCAN_WPS)
            lookahead_waypoints = self.get_waypoints(car_index)

            # Publish
            lane = self.create_lane(self.frame_id, lookahead_waypoints)
            self.final_waypoints_pub.publish(lane)

    ######## Planning waypoints  ##############

    def get_waypoints(self, current_wp):
        # current_wp = car_index
        if (current_wp + FORWARD_SCAN_WPS) > len(self.base_waypoints):
            _lookahead = len(self.base_waypoints) - current_wp
        else:
            _lookahead = FORWARD_SCAN_WPS
        # Calculate CTE for steering
        cte_steer, tfpx, tfpy = self.cte_calc(self.current_ego_pose, self.base_waypoints, _lookahead)

        # Calculate TF distance
        # RED is not negative, everything else is -tfwp
        if self.tlwp > 0:
            cte_tl = self.distance(self.base_waypoints, current_wp, self.tlwp)
        else:
            # cte_tl = -self.distance(self.base_waypoints, current_wp, -self.tlwp)
            # if (cte_tl < -16) and ( cte_tl > -self.current_linear_velocity*2) : # more than 20 m away from Green light and take less than 2 sec to pass waypoint
                cte_tl = None


        # Here goes finding the decision of what to do with the tf distance #
        if (cte_tl is not None) and (cte_tl < 5.0) and (self.current_linear_velocity < 1.0*MPS):
            self.action = STOP

        elif (cte_tl is not None) and (cte_tl < 50.0):
            self.action = DECEL

        # elif (cte_tl is not None) and (cte_tl < 50.0) and (self.current_linear_velocity > 50.0/6.5):

        
        else: # no within stopping range or current green light and can pass
            self.action = GO

        waypoints = []
        wp_to_tl = 0

        for i in range(_lookahead):
            # wp = Waypoint()
            wp = self.base_waypoints[current_wp+i]
            # wp.pose.pose.position.x = self.base_waypoints[current_wp+i].pose.pose.position.x
            # wp.pose.pose.position.y = self.base_waypoints[current_wp+i].pose.pose.position.y 
            # wp.pose.pose.position.z = self.base_waypoints[current_wp+i].pose.pose.position.z
            # wp.pose.pose.orientation.x = self.base_waypoints[current_wp+i].pose.pose.orientation.x
            
            wp.twist.twist.linear.x = self.set_velocity(self.action, -cte_tl+wp_to_tl)
            wp.twist.twist.linear.y = 0.0
            wp.twist.twist.linear.z = 0.0
            wp.twist.twist.angular.x = 0.0
            wp.twist.twist.angular.y = 0.0
            wp.twist.twist.angular.z = cte_steer(tfpx[i])

            waypoints.append(wp)

            wp_to_tl += self.distance(self.base_waypoints, car_index+i, car_index+i+1)

        return waypoints


    ############## HELP Functions ################
    def set_velocity(self, action, distance):
        if action is DECEL:
            decelcurve = self.velocity_calc(self.max_vel)
            velocity = decelcurve(distance)
        elif action is STOP:
            velocity = MIN_VEL
        else:
            velocity = MAX_VEL
        return velocity

    def velocity_calc(self, max_vel, distance, max_decel = 3.0):
        MIN_VEL = 0.0001
        # Setting points to find deceleration curve
        x = [] # distance axis
        y = [] # velocity axis
        # Max point @10 s away
        x.append(-max_vel*20)
        y.append(max_vel)

        x.append(-max_vel*10)
        y.append(max_vel)

        # @7 s away
        x.append(-max_vel*0.8*7)
        y.append(max_vel*0.8)

        # @6 s away
        x.append(-max_vel*0.6*6)
        y.append(max_vel*0.6)

        # @5 s away
        x.append(min[-16, -max_vel*0.3*5])
        y.append(max_vel*0.3)

        # @10 m away
        x.append(-10)
        y.append(MPS)

        # @5 m away
        x.append(-5)
        y.append(MPS*0.5)

        # @0 m away
        x.append(0)
        y.append(MIN_VEL)

        # @ after traffic light, all points are zeros
        x.append(max_vel*20)
        y.append(MIN_VEL)

        decel_curve = np.polyfit(x, y, 3)
        decelcurve = np.poly1d(decel_curve)
        return decelcurve

    def cte_calc(self, pose, waypoints, POINTS_TO_FIT):
        """
        Calculates the distance from the ego cars current position to the waypoints path.

        See Polynomial fitting - http://blog.mmast.net/least-squares-fitting-numpy-scipy
        """
        #rospy.logwarn("-----")
        x_coords, y_coords = self.transform_waypoints(pose, waypoints, POINTS_TO_FIT)  # Use 10 waypoints
        coefficients = np.polyfit(x_coords, y_coords, 3)  # 3-degree polynomial fit, minimising squared error
        distance = np.poly1d(coefficients)  # distance between car position and transformed waypoint

        return distance, x_coords, y_coords

    def transform_waypoints(self, pose, waypoints, points_to_use=None):
        """
        Do transformation that sets origin of waypoints to the ego car position, oriented along x-axis and
        returns transformed waypoint co-ordinates.

        See Change of basis | Essence of linear algebra, chapter 9 - https://youtu.be/P2LTAUO1TdA
        """
        x_coords = []  # array to hold transformed waypoint x
        y_coords = []  # array to hold transformed waypoint y

        _, _, yaw = self.get_euler(pose)
        origin_x = pose.position.x
        origin_y = pose.position.y

        if points_to_use is None:
            points_to_use = len(waypoints)

        for i in range(points_to_use):
            shift_x = waypoints[i].pose.pose.position.x - origin_x
            shift_y = waypoints[i].pose.pose.position.y - origin_y

            x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)
            y = shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw)
            #rospy.logwarn("i %s - x_coord : %s", i, x)
            #rospy.logwarn("i %s - y_coord : %s", i, y)

            x_coords.append(x)
            y_coords.append(y)

    def get_single_wp(self, waypoints, i):
        wp = waypoints[i]
        return wp

    def get_next_waypoints(self, waypoints, i, n):
        """ Returns a list of waypoints ahead of the ego car """
        m = min(len(waypoints), i + n)
        return deepcopy(waypoints[i:m])

    def get_closest_waypoint_index(self, pose, waypoints):
        """ Returns index of the closest waypoint """
        best_distance = float('inf')
        best_waypoint_index = 0
        my_position = pose.position

        for i, waypoint in enumerate(waypoints):

            a_waypoint_position = waypoint.pose.pose.position
            gap = self.get_distance_between_two_points(my_position, a_waypoint_position)

            if gap < best_distance:
                best_waypoint_index, best_distance = i, gap

        is_behind = self.is_waypoint_behind_ego_car(pose, waypoints[best_waypoint_index])
        if is_behind:
            best_waypoint_index += 1
        return best_waypoint_index

    def get_distance_between_two_points(self, a, b):
        """ Returns distance between two points """
        dx = a.x - b.x
        dy = a.y - b.y
        return dx * dx + dy * dy

    def is_waypoint_behind_ego_car(self, pose, waypoint):
        """ Do transformation that sets origin to the ego car position, oriented along x-axis and
        return True if the waypoint is behind the ego car,  False if in front

        See Change of basis | Essence of linear algebra, chapter 9 - https://youtu.be/P2LTAUO1TdA
        """
        _, _, yaw = self.get_euler(pose)
        origin_x = pose.position.x
        origin_y = pose.position.y

        shift_x = waypoint.pose.pose.position.x - origin_x
        shift_y = waypoint.pose.pose.position.y - origin_y

        x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)

        if x > 0:
            return False
        return True

    def get_euler(self, pose):
        """ Returns roll (x), pitch (y), yaw (z) from a Quaternion.

        See ROS Quaternion Basics for usage - http://wiki.ros.org/Tutorials/Quaternions
        """
        return transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def create_lane(self, frame_id, waypoints):
        new_lane = Lane()
        new_lane.header.frame_id = frame_id
        new_lane.waypoints = waypoints
        new_lane.header.stamp = rospy.Time.now()
        return new_lane

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

######## Obtain information from nodes ########## 
    def pose_cb(self, message):
        self.current_pose = message
        self.current_ego_pose = message.pose  # store location (x, y)
        self.frame_id = message.header.frame_id

    def waypoints_cb(self, message):
        self.base_waypoints = message.waypoints

    def traffic_lights_cb(self, message):
        self.traffic_lights = message.lights

    def traffic_cb(self, message):
        # TODO: Callback for /traffic_waypoint message. Implemented.
        self.tlwp = message.data

    def obstacle_cb(self, message):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def velocity_cb(self, message):
        self.velocity = message.twist
        self.current_linear_velocity = message.twist.linear.x
        self.current_angular_velocity = message.twist.angular.z



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
