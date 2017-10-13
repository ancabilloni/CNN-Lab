#!/usr/bin/env python

import rospy
from tf import transformations
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32

from math import cos, sin, sqrt
from copy import deepcopy

FORWARD_SCAN_WPS = 200
CRITICAL_DIST = 5
FAR_DISTANCE = 30
MIN_VEL = 0.0000001
VEL_CONVERT = 0.277778
a = 4.0
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
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)


        # Publishers
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_ego_pose = None  # ego car current position and orientation
        self.base_waypoints = None
        # self.traffic_lights = None
        self.frame_id = None
        self.stop_wp = -1
        self.current_velocity = 0.0

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.base_waypoints is None or self.current_ego_pose is None or self.frame_id is None:
                continue

            # car_index = self.get_closest_waypoint_index(self.current_ego_pose, self.base_waypoints)

            # lookahead_waypoints = self.get_next_waypoints(self.base_waypoints, car_index, FORWARD_SCAN_WPS)

            ## Speed at traffic light
            # velocity = self.get_waypoint_velocity(self.base_waypoints[car_index])
            # car_pos = self.base_waypoints[car_index].pose.pose.position
            # stop_pos = self.base_waypoints[int(self.stop_wp)].pose.pose.position
            # if self.stop_wp != -1:
            #     distance = self.get_distance_between_two_points(car_pose, stop_pos)
            #     if distance > 0 and distance < 50:
            #     	for i in range(50, FORWARD_SCAN_WPS):
            #     		vel_update = 0
            #     		self.set_waypoint_velocity(lookahead_waypoints, i, vel_update)
            
            wpts = self.base_waypoints
            stop_wp = self.stop_wp
            stop_pos = wpts[stop_wp].pose.pose.position
            car_pose = self.current_ego_pose
            car_index = self.get_closest_waypoint_index(car_pose, wpts)
            print (car_index)
            lookahead_waypoints = self.get_next_waypoints(wpts, car_index, FORWARD_SCAN_WPS)
            distance = self.get_distance_between_two_points(car_pose.position, stop_pos)

            print ("Stop waypoint: ", stop_wp)
            rospy.loginfo(stop_wp)
            print ("car index: ", car_index)
            print ("How many look ahead: ", len(lookahead_waypoints))
            print ("Distance from car to stop line: ", distance)
            if stop_wp != -1:
            	# stop_2_car = stop_wp - car_index
            	# for i in range(len(lookahead_waypoints)):
            	# 	if i < stop_2_car:
	            # 		if distance > CRITICAL_DIST and self.current_velocity < 1.0:
	            # 			vel_update = 0.3
	            # 		elif distance > CRITICAL_DIST and self.current_velocity > 1.0:
	            # 			vel_update = self.current_velocity * 0.5
	            # 		elif distance < CRITICAL_DIST and self.current_velocity < 1.0:
	            # 			vel_update = MIN_VEL
	            # 		elif distance < CRITICAL_DIST and self.current_velocity > 1.0:
	            # 			vel_update = MIN_VEL
	            # 	else:
	            # 		vel_update = MIN_VEL
	            # 	lookahead_waypoints[i].twist.twist.linear.x = vel_update

	            min_distance = self.current_velocity**2/(2*a) + CRITICAL_DIST
	            stop_2_car = self.stop_wp - car_index

	            for i in range(len(lookahead_waypoints)):
	            	if distance > CRITICAL_DIST and self.current_velocity < 1.0:
	            		vel_update = 1.0
	            	elif distance < CRITICAL_DIST:
	            		vel_update = MIN_VEL
	            	elif distance < min_distance:
	            		if i < (stop_2_car-1):
	            			dist_to_tf = self.get_distance_between_two_points(lookahead_waypoints[stop_2_car-1].pose.pose.position,
	            															lookahead_waypoints[i].pose.pose.position)
	            			vel_update = sqrt((dist_to_tf)*2*a)
	            		if i >= (stop_2_car-1):
	            			vel_update = MIN_VEL
	            	lookahead_waypoints[i].twist.twist.linear.x = vel_update


            # Publish
            lane = self.create_lane(self.frame_id, lookahead_waypoints)
            self.final_waypoints_pub.publish(lane)

    # def decelerate_planning(self, waypoints, traffic_wp):
    # 	wpts = waypoints
    # 	tf_wp = wpts[traffic_wp]
    # 	tf_wp.twist.twist.linear.x = MIN_VEL
    # 	car_to_stop = wpts[0:traffic_wp]
    # 	for wp in car_to_stop:
    # 		dist = self.get_distance_between_two_points(wp.pose.pose.position
    # 													tf_wp.pose.pose.position)



    def velocity_cb(self, message):
        self.current_velocity = message.twist.linear.x
            
    def pose_cb(self, message):
        self.current_ego_pose = message.pose  # store location (x, y)
        self.frame_id = message.header.frame_id

    def waypoints_cb(self, message):
        self.base_waypoints = message.waypoints
        self.vel_target = self.get_waypoint_velocity(self.base_waypoints[0])*VEL_CONVERT
        print ("First wp velocity: ", self.vel_target)

    def traffic_lights_cb(self, message):
        self.traffic_lights = message.lights

    def traffic_cb(self, message):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_wp = message.data
        print (self.stop_wp)


    def obstacle_cb(self, message):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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
        return sqrt(dx * dx + dy * dy)

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

    # def distance(self, waypoints, wp1, wp2):
    #     dist = 0
    #     dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    #     for i in range(wp1, wp2+1):
    #         dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
    #         wp1 = i
    #     return dist




if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
