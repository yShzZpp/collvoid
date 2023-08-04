#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf.transformations
import math
import copy

from collvoid_msgs.msg import PoseTwistWithCovariance
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3, PoseStamped, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from threading import Lock

pub = None
pub_me = None

RADIUS = 0.17
TIME_OUT = 0.5


class RobotVisualizer(object):
    detected_robots = {}

    def __init__(self):
        rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cb_position_share)
        self.publish_lock = Lock()
        self.pub = rospy.Publisher("found_robots", MarkerArray, queue_size=1)

    def cb_position_share(self, msg):
        assert isinstance(msg, PoseTwistWithCovariance)
        with self.publish_lock:
            if msg.robot_id not in self.detected_robots:
                self.detected_robots[msg.robot_id] = {}
            self.detected_robots[msg.robot_id]['msg'] = msg
            self.detected_robots[msg.robot_id]['last_seen'] = rospy.Time.now()

    def publish_robots(self):
        marker_array = MarkerArray()
        time = rospy.Time.now()
        for robot in self.detected_robots:
            if (time - self.detected_robots[robot]['last_seen']).to_sec() < TIME_OUT:
                marker_array.markers.extend(create_marker(self.detected_robots[robot]['msg']))

        self.pub.publish(marker_array)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.publish_lock:
                self.publish_robots()
            rate.sleep()

def get_robot_color(robot):
    marker = Marker()
    marker.color.a = 1.0  # 设置透明度为1.0，表示不透明

    if robot == "robot_0":
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif robot == "robot_1":
        marker.color.r = 0.439216
        marker.color.g = 0.501961
        marker.color.b = 0.564706
    elif robot == "robot_2":
        marker.color.r = 0.827451
        marker.color.g = 0.827451
        marker.color.b = 0.827451
    elif robot == "robot_3":
        marker.color.r = 0.282353
        marker.color.g = 0.239216
        marker.color.b = 0.545098
    elif robot == "robot_4":
        marker.color.r = 0.254902
        marker.color.g = 0.411765
        marker.color.b = 0.882353
    elif robot == "robot_5":
        marker.color.r = 0.529412
        marker.color.g = 0.807843
        marker.color.b = 0.980392
    elif robot == "robot_6":
        marker.color.r = 0.686275
        marker.color.g = 0.933333
        marker.color.b = 0.933333
    elif robot == "robot_7":
        marker.color.r = 0.372549
        marker.color.g = 0.619608
        marker.color.b = 0.627451
    else:
        # 默认颜色为黑色
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

    return marker.color


def create_marker(robot_msg):
    assert isinstance(robot_msg, PoseTwistWithCovariance)

    markers = []
    pose_marker = Marker()
    pose_marker.header.frame_id = robot_msg.header.frame_id
    pose_marker.header.stamp = robot_msg.header.stamp
    pose_marker.ns = robot_msg.robot_id
    pose_marker.lifetime = rospy.Duration(1)
    pose_marker.action = Marker.ADD
    pose_marker.type = Marker.SPHERE

    if robot_msg.radius > 0:
        pose_marker.scale.x = 2 * robot_msg.radius
        pose_marker.scale.y = 2 * robot_msg.radius
    else:
        pose_marker.scale.x = 2 * RADIUS
        pose_marker.scale.y = 2 * RADIUS

    pose_marker.scale.z = 0.1
    pose_marker.color = get_robot_color(robot_msg.robot_id)
    pose_marker.id = 0
    pose_marker.pose = robot_msg.pose.pose
    markers.append(pose_marker)

    text_marker = copy.deepcopy(pose_marker)
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.scale = Vector3(0, 0, 0.2)
    text_marker.color = ColorRGBA(1., 1., 1., 1.)
    text_marker.id = 1
    text_marker.text = robot_msg.robot_id
    text_marker.pose.position.z = 0.4
    markers.append(text_marker)

    arrow_marker = copy.deepcopy(pose_marker)
    arrow_marker.type = Marker.ARROW
    arrow_marker.scale = Vector3(0.1, 0.2, 0.1)
    arrow_marker.color = ColorRGBA(1., 0., 0., 1.)
    arrow_marker.id = 2
    arrow_marker.pose.orientation = Quaternion(0, 0, 0, 1)

    quat = robot_msg.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    r, p, theta = tf.transformations.euler_from_quaternion(q)

    arrow_marker.points.append(Point())
    pn = Point()
    pn.x = 2 * RADIUS * math.cos(theta)
    pn.y = 2 * RADIUS * math.sin(theta)
    arrow_marker.points.append(pn)

    markers.append(arrow_marker)
    
    return markers

if __name__ == '__main__':
    rospy.init_node("visualize_turtles")
    viz_position = RobotVisualizer()
    viz_position.spin()
