#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf.transformations
import math
import copy
import yaml
import rospkg
import re
import os
import tf2_ros
import subprocess
import json

from road_control.msg import *
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point, Point32, Vector3, PoseStamped, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from threading import Lock
from cti_msgs.msg import BuildingRobotState, ZoneArray


pub = None
pubRobots = None
pubStations = None
markerArray = None
mapPath = rospkg.RosPack().get_path("road_control") + "/map/"
lifetime = rospy.Duration(0.2)

RADIUS = 0.17
TIME_OUT = 2.0

def toString(state):
    if state == 2:
        return "IDLE"
    elif state == 25:
        return "CHARGING"
    elif state == 19 or state == 219 or state == 7 or state == 21 or state == 22 or state == 23 or state == 27 or state == 28 or state == 29 or state == 48 or (state >= 300 and state <=400):
        return "UNSTOPABLE"
    elif state == 5 or state == 6 or state == 7 or state == 8 or state == 9 or state == 202 or state == 203 or state == 204 or state == 233 or state == 15:
        return "LIFTING"
    elif state == 3 or state == 4 or state == 24 or state == 26 or state == 30 or state == 220:
        return "DOCKING"
    elif state == 10 or state == 17 or state == 18 or state == 19 or state == 219: 
        return "BUSY"
    elif state < 100:
        return "IDLE"
    else:
        return "FAULT"

class RobotVisualizer(object):
    roadRelasions = {}
    expanded_path = {}

    def __init__(self):
        #  rospy.Subscriber("/road_control/road_relation", roadRelationsMsg, self.roadRelationCallback)
        #  rospy.Subscriber("/road_control/robots_expansion_path", robotsExpansionPathMsg, self.expansionPathCallback)
        self.buildingname = "default"
        self.floor = "-99"
        #  rospy.Subscriber("/robot_state", BuildingRobotState, self.robotStateCallback)
        #  rospy.Subscriber("/otherRobot", String, self.onOtherRobotCallback)
        self.publishLock = Lock()
        self.robots = {}
        self.thisRobot = None
        #  self.pub = rospy.Publisher("/visualization/road_relations_visulizer", MarkerArray, queue_size=1)
        #  self.pubRobots = rospy.Publisher("/visualization/robots", MarkerArray, queue_size=1)
        #  self.pubRobotState = rospy.Publisher("/visualization/robot_state", MarkerArray, queue_size=1)
        self.pubStations = rospy.Publisher("/visualization/stations", MarkerArray, queue_size=1, latch=1)
        self.pubZones = rospy.Publisher("/visualization/zones", MarkerArray, queue_size=1, latch=1)
        self.markerArray = MarkerArray()
        self.zonesMarkerArray = MarkerArray()
        self.saveStation()
        self.useSim = False
        if self.useSim:
            self.saveSimOneway()
        else:
            rospy.Subscriber("/zones", ZoneArray, self.zonesCallback)

    def changeMap(self):
        file = mapPath + self.buildingname + "/" + self.floor + ".yaml"
        if not os.path.exists(file):
            print("file %s not exist" % file)
            self.buildingname = "default" 
            self.floor = "-99"
            return
        print("changing map to %s" % file)
        process = subprocess.Popen(['rosrun', 'map_server', 'map_server', file])
        rospy.sleep(2)
        print("change map success")
        process.terminate()

    def zonesCallback(self, msg):
        markers = []
        for zone in msg.zones:
            markerOnewayPose = Marker()
            markerOnewayPose.header.frame_id = "map"
            markerOnewayPose.header.stamp = rospy.Time.now()
            markerOnewayPose.lifetime = rospy.Duration(0)
            markerOnewayPose.id = zone.id
            markerOnewayPose.action = Marker.ADD
            markerOnewayPose.type = Marker.LINE_STRIP
            markerOnewayPose.color.a = 0.3
            markerOnewayPose.scale.z = 0.1
            markerOnewayPose.scale.y = 0.1
            markerOnewayPose.scale.x = 0.1
            if zone.property == 2:
                markerOnewayPose.color.r = 0.6
                markerOnewayPose.color.g = 0
                markerOnewayPose.color.b = 0.7
            elif zone.property == 4:
                markerOnewayPose.color.r = 0.7
                markerOnewayPose.color.g = 0.1
                markerOnewayPose.color.b = 0.1
            else:
                markerOnewayPose.color.r = 1.0
                markerOnewayPose.color.g = 1.0
                markerOnewayPose.color.b = 0.1
            points = []
            markerOnewayPose.ns = "zones_" + zone.place + "_" + str(zone.id)
            for anchor in zone.anchors:
                points.append(anchor.position)
            points.append(zone.anchors[0].position)
            markerOnewayPose.points = points
            markers.append(copy.deepcopy(markerOnewayPose))
            self.zonesMarkerArray.markers.extend(markers)

    def onOtherRobotCallback(self, msg):
        data = json.loads(msg.data)
        if data["robotid"] not in self.robots:
            self.robots[data["robotid"]] = BuildingRobotState()
        self.robots[data["robotid"]].robotid = str(data["robotid"]) 
        self.robots[data["robotid"]].payloadState = 2 if data["hiveAttched"] else 0
        self.robots[data["robotid"]].robot_type = str(data["state"])
        self.robots[data["robotid"]].pose.position.x = data["pose"][0]
        self.robots[data["robotid"]].pose.position.y = data["pose"][1]


    def robotStateCallback(self, msg):
        if  self.buildingname != msg.buildingname:
            self.buildingname = msg.buildingname
            self.floor = msg.current_floor
            print("buildingname: %s floor: %s" % (self.buildingname, self.floor))
            self.changeMap()
        elif self.floor != msg.current_floor:
            self.floor = msg.current_floor
            print("buildingname: %s floor: %s" % (self.buildingname, self.floor))
            self.changeMap()
        self.robots[msg.robotid] = msg
        self.robots[msg.robotid].robot_type = toString(msg.state)
        self.thisRobot = msg.robotid

        translation = (msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
        rotation = (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
        br = tf.TransformBroadcaster()
        br.sendTransform(translation, rotation, rospy.Time.now(), "base_link", "map")



    def expansionPathCallback(self, msgs):
        assert isinstance(msgs, robotsExpansionPathMsg)
        with self.publishLock:
            for msg in msgs.data:
                assert isinstance(msg, robotExpansionPathMsg)
                if msg.robot not in self.expanded_path:
                    self.expanded_path[msg.robot] = {}
                self.expanded_path[msg.robot]['points'] = msg.points
                self.expanded_path[msg.robot]['path'] = msg.path
                self.expanded_path[msg.robot]['ahead'] = msg.ahead
                self.expanded_path[msg.robot]['density'] = msg.density
                self.expanded_path[msg.robot]['last_seen'] = rospy.Time.now()
                if not self.useSim and msg.robot != self.thisRobot and msg.robot in self.robots:
                    if len(msg.path) > 1:
                        yaw = math.atan2(msg.path[1].y - msg.path[0].y, msg.path[1].x - msg.path[0].x)
                        self.robots[msg.robot].pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))

    def roadRelationCallback(self, msgs):
        assert isinstance(msgs, roadRelationsMsg)
        with self.publishLock:
            for msg in msgs.data:
                assert isinstance(msg, roadRelationMsg)
                if msg.robotOne not in self.roadRelasions:
                    self.roadRelasions[msg.robotOne] = {}
                if msg.robotTwo not in self.roadRelasions[msg.robotOne]:
                    self.roadRelasions[msg.robotOne][msg.robotTwo] = {}
                self.roadRelasions[msg.robotOne][msg.robotTwo]['msg'] = msg
                self.roadRelasions[msg.robotOne][msg.robotTwo]['last_seen'] = rospy.Time.now()

    def publishLaserMergeTF(self):
        tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        transformStamped = geometry_msgs.msg.TransformStamped()
        transformStamped.header.frame_id = 'base_link'
        transformStamped.child_frame_id = 'laser_merge'
        transformStamped.transform.translation.x = 0.000
        transformStamped.transform.translation.y = 0.000
        transformStamped.transform.translation.z = 0.200
        transformStamped.transform.rotation.x = 0.000
        transformStamped.transform.rotation.y = 0.000
        transformStamped.transform.rotation.z = 0.000
        transformStamped.transform.rotation.w = 1.000
        tf_broadcaster.sendTransform(transformStamped)


    def publish_road_relations(self):
        markerArray = MarkerArray()
        time = rospy.Time.now()
        for robot in self.roadRelasions:
            for robot2 in self.roadRelasions[robot]:
                if (time - self.roadRelasions[robot][robot2]['last_seen']).to_sec() - 10 < TIME_OUT:
                    markerArray.markers.extend(createRoadRelastionMarker(self.roadRelasions[robot][robot2]['msg']))
        for robot in self.expanded_path:
            if (time - self.expanded_path[robot]['last_seen']).to_sec() < TIME_OUT:
                markerArray.markers.extend(createPointsMarker(robot, self.expanded_path[robot]['points'], self.expanded_path[robot]['density']))
                markerArray.markers.extend(createPathMarker(robot, self.expanded_path[robot]['path']))
                markerArray.markers.extend(createAheadMarker(robot, self.expanded_path[robot]['ahead']))
        self.pub.publish(markerArray)

    def publishRobots(self):
        time = rospy.Time.now()
        markerArrayRobots= MarkerArray()
        markerArrayState = MarkerArray()
        robotLifeTime = 0.2
        for robotid, robot in self.robots.items():
            marker0 = Marker()
            marker0.header.frame_id = "map"
            marker0.header.stamp = rospy.Time.now()
            marker0.ns = robotid + "_position"
            marker0.id = 0
            marker0.type = Marker.CUBE
            marker0.action = Marker.ADD
            marker0.pose = robot.pose
            marker0.scale.x = 0.7
            marker0.scale.y = 0.5
            marker0.scale.z = 0.4
            marker0.color = getRobotColor(robotid)
            marker0.color.a = 0.5
            marker0.lifetime = rospy.Duration(robotLifeTime)
            markerArrayRobots.markers.append(marker0)
            

            if robot.payloadState == 2:
                marker1 = Marker()
                marker1.header.frame_id = "map"
                marker1.header.stamp = rospy.Time.now()
                marker1.ns = robotid + "_position_hive"
                marker1.id = 1
                marker1.type = Marker.CUBE
                marker1.action = Marker.ADD
                marker1.pose.position.x = robot.pose.position.x
                marker1.pose.position.y = robot.pose.position.y
                marker1.pose.orientation = robot.pose.orientation
                marker1.pose.position.z = 0.5
                marker1.scale.x = 0.75
                marker1.scale.y = 0.55
                marker1.scale.z = 0.4
                marker1.color.r = 1.0
                marker1.color.g = 1.0
                marker1.color.b = 1.0
                marker1.color.a = 0.7
                marker1.lifetime = rospy.Duration(robotLifeTime)
                markerArrayRobots.markers.append(marker1)

            marker2 = Marker()
            marker2.header.frame_id = "map"
            marker2.header.stamp = rospy.Time.now()
            marker2.ns = robotid + "_position_pose"
            marker2.id = 2
            marker2.type = Marker.ARROW
            marker2.action = Marker.ADD
            marker2.pose = robot.pose
            marker2.scale.x = 0.5
            marker2.scale.y = 0.1
            marker2.scale.z = 0.1
            marker2.color = getRobotColor(robotid)
            marker2.color.a = 0.5
            marker2.lifetime = rospy.Duration(robotLifeTime)
            markerArrayRobots.markers.append(marker2)

            marker3 = Marker()
            marker3.header.frame_id = "map"
            marker3.header.stamp = rospy.Time.now()
            marker3.ns = robotid + "_position_id"
            marker3.id = 3
            marker3.type = Marker.TEXT_VIEW_FACING
            marker3.action = Marker.ADD
            marker3.pose = robot.pose
            marker3.scale.z = 0.4
            marker3.color.r = 0
            marker3.color.g = 0
            marker3.color.b = 0
            marker3.text = robotid
            marker3.color.a = 1.0
            marker3.lifetime = rospy.Duration(robotLifeTime)
            markerArrayRobots.markers.append(marker3)


            marker4 = Marker()
            marker4.header.frame_id = "map"
            marker4.header.stamp = rospy.Time.now()
            #使用robot_type作为robot状态的字段
            marker4.ns = str(robotid) + ":" + robot.robot_type
            marker4.id = 0
            marker4.type = Marker.TEXT_VIEW_FACING
            marker4.action = Marker.ADD
            marker4.text = robot.robot_type
            marker4.pose.position.y = robot.pose.position.y - 0.4
            marker4.pose.position.x = robot.pose.position.x
            marker4.pose.position.z = 0.0
            marker4.pose.orientation = robot.pose.orientation
            marker4.scale.z = 0.3
            marker4.color.a = 1.0
            marker4.text = robot.robot_type 
            if robot.robot_type == "IDLE":
                marker4.color.r = 0.0
                marker4.color.g = 0.0
                marker4.color.b = 0.0
            elif robot.robot_type == "CHARGING":
                marker4.color.r = 0.0
                marker4.color.g = 1.0
                marker4.color.b = 0.0
            elif robot.robot_type == "FAULT":
                marker4.color.r = 1.0
                marker4.color.g = 0.0
                marker4.color.b = 0.0
            elif robot.robot_type == "UNSTOPABLE":
                marker4.color.r = 0.5
                marker4.color.g = 0.0
                marker4.color.b = 0.5
            elif robot.robot_type == "LIFTING":
                marker4.color.r = 0.5
                marker4.color.g = 0.5
                marker4.color.b = 0.5
            elif robot.robot_type == "BUSY":
                marker4.color.r = 1.0
                marker4.color.g = 1.0
                marker4.color.b = 0.0
            elif robot.robot_type == "DOCKING":
                marker4.color.r = 0.0
                marker4.color.g = 1.0
                marker4.color.b = 1.0
            else:
                marker4.color.r = 0.0
                marker4.color.g = 0.0
                marker4.color.b = 0.0
            marker4.lifetime = rospy.Duration(robotLifeTime)
            markerArrayState.markers.append(marker4)

        self.pubRobotState.publish(markerArrayState)
        self.pubRobots.publish(markerArrayRobots)
        

    def spin(self):
        rate = rospy.Rate(10)
        times = 0
        while not rospy.is_shutdown():
            #  self.publishLaserMergeTF()
            with self.publishLock:
                #  self.publish_road_relations()
                if times < 30:
                    self.pubZones.publish(self.zonesMarkerArray)
                    self.pubStations.publish(self.markerArray)
                    times += 1
            rate.sleep()

    def saveStation(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('road_control')
        with open('%s/params/stations.yaml' % path, 'r') as f:
            stations = yaml.safe_load(f)
        stationKeys = stations.keys()
        markers = []
        markerStationPose = Marker()
        markerStationPose.header.frame_id = "map"
        markerStationPose.header.stamp = rospy.Time.now()
        markerStationPose.lifetime = rospy.Duration(0)
        markerStationPose.id = 0
        markerStationPose.action = Marker.ADD
        markerStationPose.type = Marker.LINE_STRIP
        markerStationPose.color.r = 0.1
        markerStationPose.color.g = 0.1
        markerStationPose.color.b = 0.1
        markerStationPose.color.a = 0.5
        markerStationPose.scale.x = 0.02
    
        markerStationText= Marker()
        markerStationText.header.frame_id = "map"
        markerStationText.header.stamp = rospy.Time.now()
        markerStationText.lifetime = rospy.Duration(0)
        markerStationText.action = Marker.ADD
        markerStationText.type = Marker.TEXT_VIEW_FACING
        markerStationText.id = 0
        markerStationText.color.r = 0.0
        markerStationText.color.g = 0.0
        markerStationText.color.b = 0.0
        markerStationText.color.a = 1.0
        markerStationText.scale.z = 0.2
    
        for stationKey in stationKeys:
            for stationNum in stations[stationKey]:
                markerStationPose.ns = "stations" + stationKey + str(stationNum)
                points = []
                long = 0.2
                points.append(Point(stations[stationKey][stationNum]['x'] - long, stations[stationKey][stationNum]['y'] - long, 0))
                points.append(Point(stations[stationKey][stationNum]['x'] - long, stations[stationKey][stationNum]['y'] + long, 0))
                points.append(Point(stations[stationKey][stationNum]['x'] + long, stations[stationKey][stationNum]['y'] + long, 0))
                points.append(Point(stations[stationKey][stationNum]['x'] + long, stations[stationKey][stationNum]['y'] - long, 0))
                points.append(Point(stations[stationKey][stationNum]['x'] - long, stations[stationKey][stationNum]['y'] - long, 0))
                markerStationPose.points = points
                markerStationPose.id += 1
                markers.append(copy.deepcopy(markerStationPose))

                markerStationText.ns = "stations_name" + stationKey + str(stationNum)
                markerStationText.text = stationKey + "-" + str(stationNum)
                markerStationText.pose.position.x = stations[stationKey][stationNum]['x']
                markerStationText.pose.position.y = stations[stationKey][stationNum]['y']
                markerStationText.pose.position.z = 0.1
                markerStationText.pose.orientation.w = 1.0
                markerStationText.id += 1

                markers.append(copy.deepcopy(markerStationText))
                self.markerArray.markers.extend(markers)

    def saveSimOneway(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('road_control')
        with open('%s/params/oneways.yaml' % path, 'r') as f:
            oneways = yaml.safe_load(f)
        onewayKeys = oneways.keys()
        markers = []

        # print(oneways)
        # print("--")
        # print(onewayKeys)

        markerOnewayPose = Marker()
        markerOnewayPose.header.frame_id = "map"
        markerOnewayPose.header.stamp = rospy.Time.now()
        markerOnewayPose.lifetime = rospy.Duration(0)
        markerOnewayPose.id = 0
        markerOnewayPose.action = Marker.ADD
        markerOnewayPose.type = Marker.LINE_STRIP
        markerOnewayPose.color.r = 1
        markerOnewayPose.color.g = 1
        markerOnewayPose.color.b = 0
        markerOnewayPose.color.a = 0.3
        markerOnewayPose.scale.z = 0.1
        markerOnewayPose.scale.y = 0.1
        markerOnewayPose.scale.x = 0.1

        for onewayKey in onewayKeys :
            points = []
            markerOnewayPose.ns = "oneways" + onewayKey
            for oneway_num in oneways[onewayKey] : 
                # print(oneways[onewayKey][oneway_num])
                point = Point()
                point.x = oneways[onewayKey][oneway_num]['x']
                point.y = oneways[onewayKey][oneway_num]['y']
                points.append(point)
            p0 = Point()
            p0.x = oneways[onewayKey][0]['x']
            p0.y = oneways[onewayKey][0]['y']
            points.append(p0)
            # print(points)
            markerOnewayPose.points =points
            markerOnewayPose.id += 1
            markers.append(copy.deepcopy(markerOnewayPose))
            self.zonesMarkerArray.markers.extend(markers)

def getRobotColor(robot):
    marker = Marker()
    if robot == "robot_0":
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif robot == "robot_1":
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif robot == "robot_2":
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    elif robot == "robot_3" :
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
    elif robot == "robot_4":
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.5
    elif robot == "robot_5":
        marker.color.r = 0.5
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif robot == "robot_6":
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
    elif robot == "robot_7":
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 1.0
    elif robot == "robot_8":
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
    elif robot.isdigit():
        num = int(robot)
        marker.color.r = num / 1000.0 * 2
        marker.color.g = num % 100 / 100.0 + 0.1
        marker.color.b = num % 10 / 10.0 + 0.1
    else:
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
    return marker.color

def getDistance(p1, p2):
    return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)

def createCircleMarker(circle_msg):
    points = []
    num_points = 20
    radius = 0.5
    for i in range(num_points + 1):
        angle = i * 2 * math.pi / num_points
        p = Point()
        p.x = circle_msg.position.x + radius * math.cos(angle)
        p.y = circle_msg.position.y + radius * math.sin(angle)
        points.append(p)
    return points

def create_rectangle_marker(point1, point2, point3, point4):
    points = []
    points.append(Point(point1.position.x,point1.position.y,0))
    points.append(Point(point2.position.x,point2.position.y,0))
    points.append(Point(point3.position.x,point3.position.y,0))
    points.append(Point(point4.position.x,point4.position.y,0))
    points.append(Point(point1.position.x,point1.position.y,0))
    return points

def createAheadMarker(robot, expandedPointsMsg):
    if len(expandedPointsMsg) == 0:
        return []
    markers = []
    marker_path = Marker()
    marker_path.header.frame_id = "map"
    marker_path.header.stamp = rospy.Time.now()
    marker_path.ns = robot + "_ahead"
    marker_path.lifetime = lifetime
    marker_path.color = getRobotColor(robot)

    marker_path.id = 0
    marker_path.action = Marker.ADD
    marker_path.scale.x = 0.02
    marker_path.color.a = 0.3
    marker_path.type = Marker.LINE_STRIP
    points = []

    for i in range(len(expandedPointsMsg)):
            points.append(Point(expandedPointsMsg[i].x, expandedPointsMsg[i].y, 1.0))
    points.append(Point(expandedPointsMsg[0].x, expandedPointsMsg[0].y, 1.0))

    marker_path.points = points
    markers.append(copy.deepcopy(marker_path))


    return markers


def createPathMarker(robot, expandedPointsMsg):
    if len(expandedPointsMsg) == 0:
        return []
    markers = []
    marker_path = Marker()
    marker_path.header.frame_id = "map"
    marker_path.header.stamp = rospy.Time.now()
    marker_path.ns =robot + "_path"
    marker_path.lifetime = lifetime
    marker_path.color = getRobotColor(robot)
    marker_path.id = 0
    marker_path.action = Marker.ADD
    marker_path.scale.x = 0.02
    marker_path.color.a = 1.0
    marker_path.type = Marker.LINE_STRIP
    points = []

    for i in range(len(expandedPointsMsg)):
        if i % 2 == 0:
            points.append(Point(expandedPointsMsg[i].x, expandedPointsMsg[i].y, 0))

    marker_path.points = points
    markers.append(copy.deepcopy(marker_path))


    return markers

def createPointsMarker(robot, expandedPathMsg, density):
    if len(expandedPathMsg) < 4 :
        return []
    if len(expandedPathMsg) % 2 != 0:
        expandedPathMsg.pop()
    markers = []
    marker_line1 = Marker()
    marker_line1.header.frame_id = "map"
    marker_line1.header.stamp = rospy.Time.now()
    marker_line1.ns = robot + "_expanded_point1"
    marker_line1.lifetime = lifetime
    marker_line1.color = getRobotColor(robot)
    marker_line1.id = 0
    marker_line1.action = Marker.ADD
    marker_line1.scale.x = 0.02
    marker_line1.color.a = 1.0
    marker_line1.type = Marker.LINE_STRIP

    marker_line2 = Marker()
    marker_line2.header.frame_id = "map"
    marker_line2.header.stamp = rospy.Time.now()
    marker_line2.ns = robot + "_expanded_point2"
    marker_line2.lifetime = lifetime
    marker_line2.color = getRobotColor(robot)
    marker_line2.id = 0
    marker_line2.action = Marker.ADD
    marker_line2.scale.x = 0.02
    marker_line2.color.a = 1.0
    marker_line2.type = Marker.LINE_STRIP
    points1 = []
    points2 = []

    markerText = Marker()
    markerText.header.frame_id = "map"
    markerText.header.stamp = rospy.Time.now()
    markerText.ns = robot + "_density" 
    markerText.lifetime = lifetime
    markerText.color.r = 0.0
    markerText.color.g = 0.0
    markerText.color.b = 0.0
    markerText.color.a = 1.0
    markerText.id = 0
    markerText.action = Marker.ADD
    markerText.scale.x = 0.4
    markerText.scale.y = 0.4
    markerText.scale.z = 0.3
    markerText.type = Marker.TEXT_VIEW_FACING
    markerText.pose.position.x = expandedPathMsg[0].x 
    markerText.pose.position.y = expandedPathMsg[0].y
    markerText.pose.position.z = 1.5
    markerText.text = "R" + re.sub('[a-zA-Z_]', '', robot) + ": " + str(density)

    for i in range(0, len(expandedPathMsg), 1):
        if i % 2 == 0:
            points1.append(Point(expandedPathMsg[i].x, expandedPathMsg[i].y, 0))
        else:
            points2.append(Point(expandedPathMsg[i].x, expandedPathMsg[i].y, 0))
        if i == 0:
            points2.append(Point(expandedPathMsg[i].x, expandedPathMsg[i].y, 0))
        if i == range(0,len(expandedPathMsg))[-1]:
            points1.append(Point(expandedPathMsg[i].x, expandedPathMsg[i].y, 0))
    marker_line1.points = points1
    marker_line2.points = points2
    markers.append(copy.deepcopy(marker_line1))
    markers.append(copy.deepcopy(marker_line2))
    markers.append(copy.deepcopy(markerText))

    return markers




def createRoadRelastionMarker(roadMsg):
    markers = []
    assert isinstance(roadMsg, roadRelationMsg)
    markerPose = Marker()
    markerPose.header.frame_id = "map"
    markerPose.header.stamp = rospy.Time.now()
    markerPose.ns = "relations " + roadMsg.robotOne + " " + roadMsg.robotTwo
    markerPose.lifetime = lifetime

    markerPose.color = getRobotColor(roadMsg.robotOne)

    markerPose.id = 0
    markerPose.action = Marker.ADD
    markerPose.scale.x = 0.04
    roadMsg.pose = roadMsg.pose
    markerPose.color.a = 1.0
    markerPose.type = Marker.LINE_STRIP
    points = []
    # none
    if roadMsg.relation == 0:
        markerPose.action = Marker.DELETE
    # follow
    elif roadMsg.relation == 1:
        long = 0.4
        sqrt3 = math.sqrt(3)
        points.append(Point(roadMsg.pose.position.x, roadMsg.pose.position.y + long / sqrt3, 0))
        points.append(Point(roadMsg.pose.position.x + long / 2, roadMsg.pose.position.y - long / (2 * sqrt3), 0))
        points.append(Point(roadMsg.pose.position.x - long / 2, roadMsg.pose.position.y - long / (2 * sqrt3), 0))
        points.append(Point(roadMsg.pose.position.x, roadMsg.pose.position.y + long / sqrt3, 0))

    # opposite
    elif roadMsg.relation == 2:
        points = createCircleMarker(roadMsg.pose)
    # intersection
    elif roadMsg.relation == 3:
        long = 0.3
        points.append(Point(roadMsg.pose.position.x - long, roadMsg.pose.position.y - long, 0))
        points.append(Point(roadMsg.pose.position.x - long, roadMsg.pose.position.y + long, 0))
        points.append(Point(roadMsg.pose.position.x + long, roadMsg.pose.position.y + long, 0))
        points.append(Point(roadMsg.pose.position.x + long, roadMsg.pose.position.y - long, 0))
        points.append(Point(roadMsg.pose.position.x - long, roadMsg.pose.position.y - long, 0))

    markerPose.points = points
    markers.append(copy.deepcopy(markerPose))

    markerText = Marker()
    markerText.header.frame_id = "map"
    markerText.header.stamp = rospy.Time.now()
    markerText.ns = "relations_text " + roadMsg.robotOne + " " + roadMsg.robotTwo
    markerText.lifetime = lifetime
    markerText.color.r = 0.0
    markerText.color.g = 0.0
    markerText.color.b = 0.0
    markerText.color.a = 1.0
    markerText.id = 0
    markerText.action = Marker.ADD
    markerText.scale.x = 0.3
    markerText.scale.y = 0.3
    markerText.scale.z = 0.2
    markerText.type = Marker.TEXT_VIEW_FACING
    markerText.pose.position.x = roadMsg.pose.position.x
    markerText.pose.position.y = roadMsg.pose.position.y
    markerText.pose.position.z = 0.1
    markerText.text = roadMsg.robotTwo
    markers.append(copy.deepcopy(markerText))
    
    return markers
    
if __name__ == '__main__':
    rospy.init_node("road_relations_visualizer")
    viz_position = RobotVisualizer()
    viz_position.spin()
