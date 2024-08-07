#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import actionlib
import math
import random
import yaml

from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Twist, Pose2D
from nav_msgs.msg import Odometry
from socket import gethostname
from collvoid_msgs.msg import PoseTwistWithCovariance
from move_base_msgs.msg import *

import tf.transformations

MAX_ZERO_COUNT = 20
THRESHOLD = 0.15


def dist(a, b):
    return math.sqrt(math.pow(a.position.x - b.position.x, 2) + math.pow(a.position.y - b.position.y, 2))


class ControllerRobots(object):
    delayed_goal = None
    ground_truth = None
    zero_count = 0
    pose_stations = {}
    preset = {}

    def __init__(self):
        self.stopped = True

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.client.wait_for_server()

        self.circling = False
        # self.init_guess_srv = rospy.ServiceProxy("init_guess_pub", InitGuess)
        self.stalled = False
        self.goal_reached = True
        self.hostname = rospy.get_namespace()
        self.noise_std = rospy.get_param("/noise_std", 0.0)
        self.covariance = rospy.get_param("/covariance", 0.005)

        if self.hostname == "/":
            self.hostname = gethostname()
            #  self.goals = rospy.get_param("/%s/goals" % self.hostname, [])
        #  else:
            #  self.goals = rospy.get_param("%sgoals" % self.hostname, [])
        self.hostname = self.hostname.replace('/', '')

        #  if len(self.goals) > 0:
        #      rospy.loginfo("goals: %s" % str(self.goals))
        #      self.cur_goal = -1
        #      self.num_goals = len(self.goals)
        #      self.cur_goal_msg = None

        rospy.loginfo("Name: %s", self.hostname)
        print self.hostname
        self.pub_pose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub_stage_pose = rospy.Publisher("/%s/set_pose" % self.hostname, Pose2D, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.cur_goal_pub = rospy.Publisher("current_goal", PoseStamped, queue_size=1)
        self.sample_goal_pub = rospy.Publisher("/%s/move_base_simple/goal" % self.hostname, PoseStamped, queue_size=1)

        self.pub_commands_robot = rospy.Publisher("/commands_robot", String, queue_size=1)

        self.sub_commands_robot = rospy.Subscriber("/commands_robot", String, self.cb_commands_robot)
        self.sub_position_share = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cb_common_positions, queue_size=1)

        self.sub_goal = rospy.Subscriber("delayed_goal", PoseStamped, self.cb_delayed_goal)
        self.sub_ground_truth = rospy.Subscriber("base_pose_ground_truth", Odometry, self.cb_ground_truth, queue_size=1)

        #  use_sim = rospy.get_param("/use_sim_time", default=False)
        #  if use_sim:
        #      from stage_ros.msg import Stall
        #      self.sub_stall = rospy.Subscriber("stall", Stall, self.cb_stall)

        self.reset_srv = rospy.ServiceProxy('move_base/clear_local_costmap', Empty)

        self.global_reset_srv = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        rospy.Service('is_done', Trigger, self.cb_is_done)

        rospack = rospkg.RosPack()
        path = rospack.get_path('collvoid_stage')
        with open('%s/params/stations.yaml' % path, 'r') as f:
            stations = yaml.safe_load(f)
        points = stations.keys()
        for point in points:
            self.pose_stations[point] = stations[point]

        with open('%s/params/preset.yaml' % path, 'r') as f:
            presets_yaml = yaml.safe_load(f)
        presets = presets_yaml.keys()
        for preset in presets:
            self.preset[preset] = presets_yaml[preset]
            #  robots = presets_yaml[preset].keys()
            #  for robot in robots:
            #      init = presets_yaml[preset][robot].get('init')
            #      move = presets_yaml[preset][robot].get('init')
            #      area, point = init.split('-')[0],init.split('-')[-1]
            #      print area,point
            #      self.preset[preset][robot]["init"][]
            #
    def execute_preset(self, preset, action):
        if preset >= len(self.preset):
            print "no % preset", preset
        elif action == "move":
            robots = self.preset[preset].keys()
            if self.hostname not in robots:
                print "% no in %" % (self.hostname, self.preset)
                return
            move = self.preset[preset][self.hostname].get('move')
            area, point = move.split('-')[0], int(move.split('-')[-1])
            print "move to %s %s" % (area, point)
            pose = self.get_pose_stamped(area, point)
            self.sample_goal_pub.publish(pose)
        elif action == "init":
            robots = self.preset[preset].keys()
            if self.hostname not in robots:
                print "% no in %" % (self.hostname, self.preset)
                return
            init = self.preset[preset][self.hostname].get('init')
            area, point = init.split('-')[0], int(init.split('-')[-1])
            print "init to %s %s" % (area, point)
            self.pub_stage_pose.publish(self.get_pose(area, point))
            self.pub_pose.publish(self.get_pose_with_covariance(area, point, self.covariance))


    def get_pose(self, station, index):
        pose = Pose2D()
        pose.x = self.pose_stations.get(station).get(index).get('x')
        pose.y = self.pose_stations.get(station).get(index).get('y')
        pose.theta = self.pose_stations.get(station).get(index).get('yaw')
        return pose

    def get_pose_stamped(self, station, index):
        print type(station), type(index)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.pose_stations.get(station).get(index).get('x')
        pose.pose.position.y = self.pose_stations.get(station).get(index).get('y')
        pose.pose.orientation.z = math.sin(self.pose_stations.get(station).get(index).get('yaw') / 2.0)
        pose.pose.orientation.w = math.cos(self.pose_stations.get(station).get(index).get('yaw') / 2.0)
        return pose

    def get_pose_with_covariance(self ,station, index ,noise_cov):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = self.pose_stations.get(station).get(index).get('x')
        pose.pose.pose.position.y = self.pose_stations.get(station).get(index).get('y')
        pose.pose.pose.orientation.z = math.sin(self.pose_stations.get(station).get(index).get('yaw') / 2.0)
        pose.pose.pose.orientation.w = math.cos(self.pose_stations.get(station).get(index).get('yaw') / 2.0)
        pose.header.stamp = rospy.Time.now()
        pose.pose.covariance[0] = noise_cov
        pose.pose.covariance[7] = noise_cov
        pose.pose.covariance[35] = noise_cov / 4.0
        return pose


    def cb_is_done(self, req):
        if self.goal_reached or self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return {'success': True}
        return {'success': False}

    def return_cur_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = self.goals[self.cur_goal]["x"]
        goal.target_pose.pose.position.y = self.goals[self.cur_goal]["y"]
        q = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.cur_goal]["ang"], axes='sxyz')
        goal.target_pose.pose.orientation = Quaternion(*q)

        goal.target_pose.header.frame_id = "map"
        return goal

    def cb_stall(self, msg):
        self.stalled = msg.stall

    def cb_common_positions(self, msg):
        if self.stopped or self.cur_goal_msg is None:
            return  # rospy.loginfo("%s"%rospy.get_master())
        if msg.robot_id == self.hostname:
            if dist(msg.pose.pose, self.cur_goal_msg.target_pose.pose) < THRESHOLD:
                if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED and not self.circling:
                    rospy.loginfo("Reached last goal")
                    self.goal_reached = True
                    self.stopped = True

                #if self.cur_goal == self.num_goals-1 and not self.circling:
                #    if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                #        rospy.loginfo("Reached last goal")
                #        rospy.logerr("I AM DONE")
                #        self.stopped = True
                #    return
                #else:
                #  if self.circling:
                #      self.cur_goal += 1
                #      if self.circling and self.cur_goal == self.num_goals:
                #          self.cur_goal = 0
                #      rospy.loginfo("sending new goal %d/%d", self.cur_goal, self.num_goals)
                #      #  self.cur_goal_msg = self.return_cur_goal()
                #      str = "%s Start" % self.hostname
                #      self.pub_commands_robot.publish(String(str))

    def cb_delayed_goal(self, msg):
        self.delayed_goal = MoveBaseGoal()
        self.delayed_goal.target_pose.pose.position = msg.pose.position
        self.delayed_goal.target_pose.pose.orientation = msg.pose.orientation
        self.delayed_goal.target_pose.header = msg.header
        print str(self.delayed_goal)

    def cb_ground_truth(self, msg):
        self.ground_truth = PoseWithCovarianceStamped()
        self.ground_truth.header.frame_id = "map"
        self.ground_truth.pose.pose = msg.pose.pose

        if self.stopped:
            return
        if msg.twist.twist.linear.x == 0 and msg.twist.twist.linear.y == 0 and msg.twist.twist.angular.z == 0:
            self.zero_count += 1
            if self.zero_count > MAX_ZERO_COUNT:
                if self.stalled:
                    twist = Twist()
                    twist.linear.x = -0.05
                    self.pub_cmd_vel.publish(twist)
                else:
                    twist = Twist()
                    twist.linear.x = 0.05
                    self.pub_cmd_vel.publish(twist)
                #self.client.cancel_all_goals()
                #self.client.send_goal(self.cur_goal_msg)
                str = "%s Start" % self.hostname
                self.pub_commands_robot.publish(String(str))
        else:
            self.zero_count = 0

        # self.ground_truth.pose.pose.position.x = -msg.pose.pose.position.y
        # self.ground_truth.pose.pose.position.y = msg.pose.pose.position.x
        # q = msg_to_quaternion(msg.pose.pose.orientation)
        # rpy = list(tf.transformations.euler_from_quaternion(q))
        # yaw = rpy[2] + math.pi / 2.0
        # q = tf.transformations.quaternion_from_euler(0,0,yaw, axes='sxyz')
        # self.ground_truth.pose.pose.orientation.x = q[0]
        # self.ground_truth.pose.pose.orientation.y = q[1]
        # self.ground_truth.pose.pose.orientation.z = q[2]
        # self.ground_truth.pose.pose.orientation.w = q[3]

    def publish_init_guess(self, noise_cov, noise_std):
        if self.ground_truth is not None:
            self.ground_truth.pose.pose.position.x += random.gauss(0, noise_std)
            self.ground_truth.pose.pose.position.y += random.gauss(0, noise_std)

            self.ground_truth.header.stamp = rospy.Time.now()
            self.ground_truth.pose.covariance[0] = noise_cov
            self.ground_truth.pose.covariance[7] = noise_cov
            self.ground_truth.pose.covariance[35] = noise_cov / 4.0
            self.pub_pose.publish(self.ground_truth)

    def deg2rad(deg):
        return deg * math.pi / 180.0

    def publish_pose_one(self, noise_cov, noise_std):
        if self.hostname == "robot_0":
            self.pub_stage_pose.publish(self.get_pose('A', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('A', 0, noise_cov))
        elif self.hostname == "robot_1":
            self.pub_stage_pose.publish(self.get_pose('B', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('B', 0, noise_cov))
        elif self.hostname == "robot_2":
            self.pub_stage_pose.publish(self.get_pose('B', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('B', 1, noise_cov))
        elif self.hostname == "robot_3":
            self.pub_stage_pose.publish(self.get_pose('B', 2))
            self.pub_pose.publish(self.get_pose_with_covariance('B', 2, noise_cov))

    def publish_pose_two(self, noise_cov, noise_std):
        if self.hostname == "robot_0":
            self.pub_stage_pose.publish(self.get_pose('A', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('A', 0, noise_cov))
        elif self.hostname == "robot_1":
            self.pub_stage_pose.publish(self.get_pose('B', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('B', 0, noise_cov))
        elif self.hostname == "robot_2":
            self.pub_stage_pose.publish(self.get_pose('C', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('C', 0, noise_cov))
        elif self.hostname == "robot_3":
            self.pub_stage_pose.publish(self.get_pose('C', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('C', 1, noise_cov))

    def publish_opposite_pose1(self, noise_cov, noise_std):
        if self.hostname == "robot_0":
            self.pub_stage_pose.publish(self.get_pose('A', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('A', 1, noise_cov))
        elif self.hostname == "robot_1":
            self.pub_stage_pose.publish(self.get_pose('B', 2))
            self.pub_pose.publish(self.get_pose_with_covariance('B', 2, noise_cov))
        elif self.hostname == "robot_2":
            self.pub_stage_pose.publish(self.get_pose('C', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('C', 0, noise_cov))
        elif self.hostname == "robot_3":
            self.pub_stage_pose.publish(self.get_pose('D', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('D', 0, noise_cov))
        elif self.hostname == "robot_4":
            self.pub_stage_pose.publish(self.get_pose('A', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('A', 1, noise_cov))
        elif self.hostname == "robot_5":
            self.pub_stage_pose.publish(self.get_pose('B', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('B', 1, noise_cov))
        elif self.hostname == "robot_6":
            self.pub_stage_pose.publish(self.get_pose('C', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('C', 1, noise_cov))
        elif self.hostname == "robot_7":
            self.pub_stage_pose.publish(self.get_pose('D', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('D', 1, noise_cov))

    def publish_opposite_pose2(self, noise_cov, noise_std):
        if self.hostname == "robot_0":
            self.pub_stage_pose.publish(self.get_pose('A', 2))
            self.pub_pose.publish(self.get_pose_with_covariance('A', 2, noise_cov))
        elif self.hostname == "robot_1":
            self.pub_stage_pose.publish(self.get_pose('A', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('A', 1, noise_cov))
        elif self.hostname == "robot_2":
            self.pub_stage_pose.publish(self.get_pose('A', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('A', 0, noise_cov))
        elif self.hostname == "robot_3":
            self.pub_stage_pose.publish(self.get_pose('B', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('B', 0, noise_cov))
        elif self.hostname == "robot_4":
            self.pub_stage_pose.publish(self.get_pose('B', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('B', 1, noise_cov))
        elif self.hostname == "robot_5":
            self.pub_stage_pose.publish(self.get_pose('B', 2))
            self.pub_pose.publish(self.get_pose_with_covariance('B', 2, noise_cov))
        elif self.hostname == "robot_6":
            self.pub_stage_pose.publish(self.get_pose('C', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('C', 0, noise_cov))
        elif self.hostname == "robot_7":
            self.pub_stage_pose.publish(self.get_pose('C', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('C', 1, noise_cov))

    def publish_follow_pose(self, noise_cov, noise_std):
        if self.hostname == "robot_0":
            self.pub_stage_pose.publish(self.get_pose('F', 0))
            self.pub_pose.publish(self.get_pose_with_covariance('F', 0, noise_cov))
        elif self.hostname == "robot_1":
            self.pub_stage_pose.publish(self.get_pose('F', 1))
            self.pub_pose.publish(self.get_pose_with_covariance('F', 1, noise_cov))
        elif self.hostname == "robot_2":
            self.pub_stage_pose.publish(self.get_pose('F', 2))
            self.pub_pose.publish(self.get_pose_with_covariance('F', 2, noise_cov))
        elif self.hostname == "robot_3":
            self.pub_stage_pose.publish(self.get_pose('F', 3))
            self.pub_pose.publish(self.get_pose_with_covariance('F', 3, noise_cov))
        elif self.hostname == "robot_4":
            self.pub_stage_pose.publish(self.get_pose('F', 4))
            self.pub_pose.publish(self.get_pose_with_covariance('F', 4, noise_cov))
        elif self.hostname == "robot_5":
            self.pub_stage_pose.publish(self.get_pose('F', 5))
            self.pub_pose.publish(self.get_pose_with_covariance('F', 5, noise_cov))
        elif self.hostname == "robot_6":
            self.pub_stage_pose.publish(self.get_pose('F', 6))
            self.pub_pose.publish(self.get_pose_with_covariance('F', 6, noise_cov))
        elif self.hostname == "robot_7":
            self.pub_stage_pose.publish(self.get_pose('F', 7))
            self.pub_pose.publish(self.get_pose_with_covariance('F', 7, noise_cov))


    def cb_commands_robot(self, msg):
        #  rospy.loginfo("received command %s", msg.data)
        if "all" not in msg.data and self.hostname not in msg.data:
            return

        if "WP Change" in msg.data:
            self.stopped = not self.stopped

        if "Start" in msg.data:
            try:
                self.global_reset_srv()
                self.reset_srv()
            except:
                pass
            #  if self.cur_goal == -1:
            #      self.cur_goal = 0
            #  self.cur_goal_msg = self.return_cur_goal()
            #  rospy.loginfo("sending new goal %d/%d", self.cur_goal, self.num_goals)
            #  self.stopped = False
            #  self.goal_reached = False
            #  self.client.send_goal(self.cur_goal_msg)
            #  self.cur_goal_pub.publish(self.cur_goal_msg.target_pose)

        if "init Guess" in msg.data:
            self.publish_init_guess(self.covariance, self.noise_std)
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)
            rospy.sleep(0.2)
            self.publish_init_guess(self.covariance, self.noise_std)

        if "init" in msg.data:
            area=""
            point = int(self.hostname.split('_')[-1])
            if "-" in msg.data:
                position = msg.data.split(' ')[-1]
                area, point = position.split('-')[0], int(position.split('-')[1])
            else:
                area = msg.data.split(' ')[-1]
            print area,point
            self.pub_stage_pose.publish(self.get_pose(area, point))
            self.pub_pose.publish(self.get_pose_with_covariance(area, point, self.covariance))

        if "move" in msg.data:
            area=""
            point = int(self.hostname.split('_')[-1])
            if "-" in msg.data:
                position = msg.data.split(' ')[-1]
                area, point = position.split('-')[0], int(position.split('-')[1])
            else:
                area = msg.data.split(' ')[-1]
            print area,point
            pose = self.get_pose_stamped(area, point)
            self.sample_goal_pub.publish(pose)

        if "Preseti" in msg.data:
            preset = msg.data.split(' ')[-1]
            self.execute_preset(int(preset), "init")

        if "Presetm" in msg.data:
            preset = msg.data.split(' ')[-1]
            self.execute_preset(int(preset), "move")




        if "follow pose" in msg.data:
            self.publish_follow_pose(self.covariance, self.noise_std)
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)
            rospy.sleep(0.2)
            self.publish_follow_pose(self.covariance, self.noise_std)

        if "opposite1 pose" in msg.data:
            self.publish_opposite_pose1(self.covariance, self.noise_std)
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)
            rospy.sleep(0.2)
            self.publish_opposite_pose1(self.covariance, self.noise_std)

        if "opposite2 pose" in msg.data:
            self.publish_opposite_pose2(self.covariance, self.noise_std)
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)
            rospy.sleep(0.2)
            self.publish_opposite_pose2(self.covariance, self.noise_std)


        if "pose one" in msg.data:
            self.publish_pose_one(self.covariance, self.noise_std)
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)
            rospy.sleep(0.2)
            self.publish_pose_one(self.covariance, self.noise_std)
            

        if "Restart" in msg.data:
            self.stopped = True
            self.client.cancel_all_goals()
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)

        if "Stop" in msg.data:
            self.stopped = True
            self.client.cancel_all_goals()

        if "circle" in msg.data:
            self.circling = not self.circling
            rospy.loginfo("Set circling to %s", str(self.circling))

        if "to F" in msg.data:
            pose = PoseStamped()
            if self.hostname == "robot_0":
                pose = self.get_pose_stamped('F', 0)
            elif self.hostname == "robot_1":
                pose = self.get_pose_stamped('F', 1)
            elif self.hostname == "robot_2":
                pose = self.get_pose_stamped('F', 2)
            elif self.hostname == "robot_3":
                pose = self.get_pose_stamped('F', 3)
            elif self.hostname == "robot_4":
                pose = self.get_pose_stamped('F', 4)
            elif self.hostname == "robot_5":
                pose = self.get_pose_stamped('F', 5)
            elif self.hostname == "robot_6":
                pose = self.get_pose_stamped('F', 6)
            elif self.hostname == "robot_7":
                pose = self.get_pose_stamped('F', 7)
            elif self.hostname == "robot_8":
                pose = self.get_pose_stamped('F', 8)
            self.sample_goal_pub.publish(pose)

        if "to E" in msg.data:
            pose = PoseStamped()
            if self.hostname == "robot_0":
                pose = self.get_pose_stamped('E', 0)
            elif self.hostname == "robot_1":
                pose = self.get_pose_stamped('E', 1)
            elif self.hostname == "robot_2":
                pose = self.get_pose_stamped('E', 2)
            elif self.hostname == "robot_3":
                pose = self.get_pose_stamped('E', 3)
            elif self.hostname == "robot_4":
                pose = self.get_pose_stamped('E', 4)
            elif self.hostname == "robot_5":
                pose = self.get_pose_stamped('E', 5)
            elif self.hostname == "robot_6":
                pose = self.get_pose_stamped('E', 6)
            elif self.hostname == "robot_7":
                pose = self.get_pose_stamped('E', 7)
            elif self.hostname == "robot_8":
                pose = self.get_pose_stamped('E', 8)
            self.sample_goal_pub.publish(pose)

        if "to H" in msg.data:
            pose = PoseStamped()
            if self.hostname == "robot_0":
                pose = self.get_pose_stamped('H', 0)
            elif self.hostname == "robot_1":
                pose = self.get_pose_stamped('H', 1)
            elif self.hostname == "robot_2":
                pose = self.get_pose_stamped('H', 2)
            elif self.hostname == "robot_3":
                pose = self.get_pose_stamped('H', 3)
            elif self.hostname == "robot_4":
                pose = self.get_pose_stamped('H', 4)
            elif self.hostname == "robot_5":
                pose = self.get_pose_stamped('H', 5)
            elif self.hostname == "robot_6":
                pose = self.get_pose_stamped('H', 6)
            elif self.hostname == "robot_7":
                pose = self.get_pose_stamped('H', 7)
            elif self.hostname == "robot_8":
                pose = self.get_pose_stamped('H', 8)
            self.sample_goal_pub.publish(pose)

        if "to I" in msg.data:
            pose = PoseStamped()
            if self.hostname == "robot_0":
                pose = self.get_pose_stamped('I', 0)
            elif self.hostname == "robot_1":
                pose = self.get_pose_stamped('I', 1)
            elif self.hostname == "robot_2":
                pose = self.get_pose_stamped('I', 2)
            elif self.hostname == "robot_3":
                pose = self.get_pose_stamped('I', 3)
            elif self.hostname == "robot_4":
                pose = self.get_pose_stamped('I', 4)
            elif self.hostname == "robot_5":
                pose = self.get_pose_stamped('I', 5)
            elif self.hostname == "robot_6":
                pose = self.get_pose_stamped('I', 6)
            elif self.hostname == "robot_7":
                pose = self.get_pose_stamped('I', 7)
            elif self.hostname == "robot_8":
                pose = self.get_pose_stamped('I', 8)
            self.sample_goal_pub.publish(pose)

        if "to G" in msg.data:
            pose = PoseStamped()
            if self.hostname == "robot_0":
                pose = self.get_pose_stamped('G', 0)
            elif self.hostname == "robot_1":
                pose = self.get_pose_stamped('G', 1)
            elif self.hostname == "robot_2":
                pose = self.get_pose_stamped('G', 2)
            elif self.hostname == "robot_3":
                pose = self.get_pose_stamped('G', 3)
            elif self.hostname == "robot_4":
                pose = self.get_pose_stamped('G', 4)
            elif self.hostname == "robot_5":
                pose = self.get_pose_stamped('G', 5)
            elif self.hostname == "robot_6":
                pose = self.get_pose_stamped('G', 6)
            elif self.hostname == "robot_7":
                pose = self.get_pose_stamped('G', 7)
            elif self.hostname == "robot_8":
                pose = self.get_pose_stamped('G', 8)
            self.sample_goal_pub.publish(pose)



        if "opposite1 Goal" in msg.data:
            pose = PoseStamped()
            if self.hostname == "robot_0":
                pose = self.get_pose_stamped('B', 1)
            elif self.hostname == "robot_1":
                pose = self.get_pose_stamped('A', 1)
            elif self.hostname == "robot_2":
                pose = self.get_pose_stamped('D', 1)
            elif self.hostname == "robot_3":
                pose = self.get_pose_stamped('C', 1)
            elif self.hostname == "robot_4":
                pose = self.get_pose_stamped('D', 2)
            elif self.hostname == "robot_5":
                pose = self.get_pose_stamped('C', 2)
            elif self.hostname == "robot_6":
                pose = self.get_pose_stamped('C', 2)
            self.sample_goal_pub.publish(pose)

        if "opposite2 Goal" in msg.data:
            pose = PoseStamped()
            if self.hostname == "robot_0":
                pose = self.get_pose_stamped('B', 0)
            elif self.hostname == "robot_1":
                pose = self.get_pose_stamped('B', 1)
            elif self.hostname == "robot_2":
                pose = self.get_pose_stamped('B', 2)
            elif self.hostname == "robot_3":
                pose = self.get_pose_stamped('A', 2)
            elif self.hostname == "robot_4":
                pose = self.get_pose_stamped('A', 1)
            elif self.hostname == "robot_5":
                pose = self.get_pose_stamped('A', 0)
            self.sample_goal_pub.publish(pose)


        if "pose one" in msg.data:
            self.publish_pose_one(self.covariance, self.noise_std)
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)
            rospy.sleep(0.2)
            self.publish_pose_one(self.covariance, self.noise_std)

        if "action one" in msg.data:
            pose = PoseStamped()
            if self.hostname == "robot_0":
                pose = self.get_pose_stamped('B', 2)
            elif self.hostname == "robot_1":
                pose = self.get_pose_stamped('A', 1)
            elif self.hostname == "robot_2":
                pose = self.get_pose_stamped('A', 2)
            else:
                print ""
            self.sample_goal_pub.publish(pose)

        if "pose two" in msg.data:
            self.publish_pose_two(self.covariance, self.noise_std)
            try:
                self.global_reset_srv()
                self.global_reset_srv()
                self.reset_srv()
                self.reset_srv()
            except rospy.ServiceException as e:
                rospy.logwarn(e)
            rospy.sleep(0.2)
            self.publish_pose_two(self.covariance, self.noise_std)

        if "action two" in msg.data:
            pose = PoseStamped()
            if self.hostname == "robot_0":
                pose = self.get_pose_stamped('B', 1)
            elif self.hostname == "robot_1":
                pose = self.get_pose_stamped('A', 1)
            elif self.hostname == "robot_2":
                pose = self.get_pose_stamped('D', 0)
            else:
                print ""
            self.sample_goal_pub.publish(pose)

        if "send delayed Goal" in msg.data:
            if self.delayed_goal is not None:
                self.stopped = False
                self.goal_reached = False
                self.client.send_goal(self.delayed_goal)


if __name__ == '__main__':
    rospy.init_node('candle_controller_robots')
    controller_waypoints = ControllerRobots()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        r.sleep()
    rospy.delete_param("/move_base/")
