#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import rospkg
import yaml
import tf.transformations
from geometry_msgs.msg import PoseArray, Pose, Quaternion

try:
    import wx
except ImportError:
    raise ImportError("The wxPython module is required to run this program")

from std_msgs.msg import String
from std_srvs.srv import Empty
from collvoid_msgs.msg import PoseTwistWithCovariance

class Station():
    def __init__(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('collvoid_stage')
        with open('%s/params/stations.yaml' % path, 'r') as f:
            stations = yaml.safe_load(f)
        points = stations.keys()
        self.pose_stations=dict()
        for point in points:
            self.pose_stations[point] = stations[point]
    def getAllAreas(self):
        areas=[]
        for a in self.pose_stations:
            areas.append(a)
        return areas
    def getPoints(self, area):
        points=[]
        for point in self.pose_stations[area]:
            points.append(str(point))
        return points



class Controller(wx.Frame):
    def getPresetComment(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('collvoid_stage')
        with open('%s/params/preset.yaml' % path, 'r') as f:
            presets_yaml = yaml.safe_load(f)
        presets = presets_yaml.keys()
        comment = []
        for preset in presets:
            comment.append(presets_yaml[preset].get('comment'))
        return comment



    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title)
        self.parent = parent
        self.initialized = False
        self.station = Station()

        sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(sizer)

        self.pub = rospy.Publisher('/commands_robot', String, queue_size=10)

        self.robotList = []
        self.robotList.append("all")

        self.reset_srv = rospy.ServiceProxy('/reset_positions', Empty)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "控制"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        #  start = wx.Button(self, wx.ID_ANY, label="Start!")
        #  static_sizer.Add(start, 0)
        #  self.Bind(wx.EVT_BUTTON, self.start, start)
        #
        stop = wx.Button(self, wx.ID_ANY, label="停止")
        static_sizer.Add(stop, 0)
        self.Bind(wx.EVT_BUTTON, self.stop, stop)

        #  reset = wx.Button(self, wx.ID_ANY, label="Reset!")
        #  static_sizer.Add(reset, 0)
        #  self.Bind(wx.EVT_BUTTON, self.reset, reset)

        grid_sizer = wx.GridBagSizer()
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "自定义点位定位"), wx.HORIZONTAL)
        static_sizer.Add(grid_sizer, 0)
        sizer.Add(static_sizer, 0)

        # 机器人
        self.choiceRobotBox = wx.Choice(self, wx.ID_ANY, choices=self.robotList)
        self.choiceRobotBox.SetSelection(0)
        grid_sizer.Add(self.choiceRobotBox, pos=(0, 0), span=(1, 1), flag=wx.EXPAND)

        # 区域
        self.choiceAreaBox = wx.Choice(self, wx.ID_ANY, choices=self.station.getAllAreas())
        self.choiceAreaBox.SetSelection(0)
        grid_sizer.Add(self.choiceAreaBox, pos=(0, 2), span=(1, 1), flag=wx.EXPAND)
        # 绑定区域选择事件
        self.choiceAreaBox.Bind(wx.EVT_CHOICE, self.onAreaChoice)
        # 点位
        self.choicePointBox = wx.Choice(self, wx.ID_ANY, choices=[])
        grid_sizer.Add(self.choicePointBox, pos=(0, 3), span=(1, 1), flag=wx.EXPAND)
        self.SetPosition(wx.Point(200, 200))
        self.SetSize(wx.Size(400, 200))

        initPose = wx.Button(self, wx.ID_ANY, label="定位")
        grid_sizer.Add(initPose, (1, 0))
        self.Bind(wx.EVT_BUTTON, self.initPose, initPose)

        move = wx.Button(self, wx.ID_ANY, label="移动")
        grid_sizer.Add(move, (1, 2))
        self.Bind(wx.EVT_BUTTON, self.move, move)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "预设"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        self.choicePresetBox = wx.Choice(self, wx.ID_ANY, choices=self.getPresetComment())
        self.choicePresetBox.SetSelection(0)
        #  grid_sizer.Add(self.choicePresetBox, pos=(3, 0), span=(1, 1), flag=wx.EXPAND)
        static_sizer.Add(self.choicePresetBox, 0)

        initPreset = wx.Button(self, wx.ID_ANY, label="定位")
        #  grid_sizer.Add(initPreset, (3, 1))
        static_sizer.Add(initPreset, 2)
        self.Bind(wx.EVT_BUTTON, self.initPreset, initPreset )

        movePreset = wx.Button(self, wx.ID_ANY, label="移动")
        #  grid_sizer.Add(movePreset, (3, 2))
        static_sizer.Add(movePreset, 2)
        self.Bind(wx.EVT_BUTTON, self.movePreset, movePreset )

        grid_sizer.AddGrowableCol(0)
        self.SetSizer(sizer)

        self.Layout()
        self.Fit()
        self.Show(True)

        self.obst_pub = rospy.Publisher('/obstacles', PoseArray, queue_size=1, latch=True)
        self.goals_pub = rospy.Publisher('/goals', PoseArray, queue_size=1, latch=True)

        self.num_obstacles = rospy.get_param('/num_obstacles', 0)

        self.obst_msg = PoseArray()
        self.obst_msg.header.frame_id = 'map'
        for i in range(self.num_obstacles):
            p = Pose()
            obst = rospy.get_param('obst_%d'%i, [])
            if len(obst) == 0:
                continue
            p.position.x = obst['x']
            p.position.y = obst['y']
            p.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, obst['ang']))
            self.obst_msg.poses.append(p)
        self.obst_pub.publish(self.obst_msg)

        self.subCommonPositions = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.cbCommonPositions)

        self.initialized = True
        self.services = []
        self.initPointChoices()

    def initPointChoices(self):
        area = self.choiceAreaBox.GetStringSelection()
        points = self.station.getPoints(area)
        points.append("自适应")
        self.choicePointBox.Clear()
        self.choicePointBox.AppendItems(points)
        if points:
            self.choicePointBox.SetSelection(len(points)-1)

    def onAreaChoice(self, event):
        area = self.choiceAreaBox.GetStringSelection()
        points = self.station.getPoints(area)
        points.append("自适应")
        self.choicePointBox.Clear()
        self.choicePointBox.AppendItems(points)
        if points:
            self.choicePointBox.SetSelection(len(points)-1)

    def toggleServices(self, event):
        for s in self.services:
            try:
                s()
            except rospy.ServiceException as e:
                rospy.logwarn("could not call toggle service collviod %s", e)
        return

    def cbCommonPositions(self, msg):
        if not self.initialized:
            return
        if self.robotList.count(msg.robot_id) == 0:
            rospy.loginfo("robot added")
            self.robotList.append(msg.robot_id)
            if msg.controlled:
                s = rospy.ServiceProxy(msg.robot_id + '/toggle_active_collvoid', Empty)
                self.services.append(s)
            self.choiceRobotBox.Append(msg.robot_id)

    def initPose(self, event):
        robot=self.choiceRobotBox.GetStringSelection()
        area=self.choiceAreaBox.GetStringSelection()
        point=self.choicePointBox.GetStringSelection()
        if not point.isdigit():
            string = "%s init %s" % (robot, area)
        else:
            string = "%s init %s-%s" % (robot, area, point)
        self.pub.publish(str(string))

    def move(self, event):
        robot=self.choiceRobotBox.GetStringSelection()
        area=self.choiceAreaBox.GetStringSelection()
        point=self.choicePointBox.GetStringSelection()
        if not point.isdigit():
            string = "%s move %s" % (robot, area)
        else:
            string = "%s move %s-%s" % (robot, area, point)
        self.pub.publish(str(string))


    def initPreset(self, event):
        string = "%s Preseti %d" % (self.choiceRobotBox.GetStringSelection(), self.choicePresetBox.GetSelection())
        self.pub.publish(str(string))

    def movePreset(self, event):
        string = "%s Presetm %d" % (self.choiceRobotBox.GetStringSelection(), self.choicePresetBox.GetSelection())
        self.pub.publish(str(string))

    def stop(self, event):
        string = "%s Stop" % self.choiceRobotBox.GetStringSelection()
        self.pub.publish(str(string))

    def start(self, event):
        string = "%s next Goal" % self.choiceRobotBox.GetStringSelection()
        self.pub.publish(str(string))

    def all_start(self, event):
        string = "all Start"
        self.pub.publish(str(string))

    def all_init_guess(self, event):
        string = "all init Guess"
        self.pub.publish(str(string))

    def reset(self, event):
        self.pub.publish("all Stop")
        rospy.sleep(0.2)
        self.pub.publish("all Restart")
        # rospy.sleep(0.2)
        self.reset_srv()


if __name__ == '__main__':
    rospy.init_node('controller')
    app = wx.App()
    frame = Controller(None, wx.ID_ANY, 'Controller')

    app.MainLoop()
