#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import tf.transformations
from geometry_msgs.msg import PoseArray, Pose, Quaternion

try:
    import wx
except ImportError:
    raise ImportError("The wxPython module is required to run this program")

from std_msgs.msg import String
from std_srvs.srv import Empty
from collvoid_msgs.msg import PoseTwistWithCovariance


class Controller(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title)
        self.parent = parent
        self.initialized = False

        sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(sizer)

        self.pub = rospy.Publisher('/commands_robot', String, queue_size=10)

        self.robotList = []
        self.robotList.append("all")

        self.reset_srv = rospy.ServiceProxy('/reset_positions', Empty)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Controls"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        start = wx.Button(self, wx.ID_ANY, label="Start!")
        static_sizer.Add(start, 0)
        self.Bind(wx.EVT_BUTTON, self.start, start)

        stop = wx.Button(self, wx.ID_ANY, label="Stop!")
        static_sizer.Add(stop, 0)
        self.Bind(wx.EVT_BUTTON, self.stop, stop)

        reset = wx.Button(self, wx.ID_ANY, label="Reset!")
        static_sizer.Add(reset, 0)
        self.Bind(wx.EVT_BUTTON, self.reset, reset)

        grid_sizer = wx.GridBagSizer()
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "choseRobots"), wx.HORIZONTAL)
        static_sizer.Add(grid_sizer, 0)
        sizer.Add(static_sizer, 0)

        self.choiceBox = wx.Choice(self, wx.ID_ANY, choices=self.robotList)
        self.choiceBox.SetSelection(0)

        grid_sizer.Add(self.choiceBox, (0, 0), (1, 2), wx.EXPAND)
        self.SetPosition(wx.Point(200, 200))
        self.SetSize(wx.Size(600, 200))

        sendToE = wx.Button(self, wx.ID_ANY, label="Send to E")
        grid_sizer.Add(sendToE, (3, 0))
        self.Bind(wx.EVT_BUTTON, self.sendToE, sendToE)

        sendToF = wx.Button(self, wx.ID_ANY, label="Send to F")
        grid_sizer.Add(sendToF, (3, 1))
        self.Bind(wx.EVT_BUTTON, self.sendToF, sendToF)

        sendToG = wx.Button(self, wx.ID_ANY, label="Send to G")
        grid_sizer.Add(sendToG, (3, 2))
        self.Bind(wx.EVT_BUTTON, self.sendToG, sendToG)

        sendToH = wx.Button(self, wx.ID_ANY, label="Send to H")
        grid_sizer.Add(sendToH, (4, 1))
        self.Bind(wx.EVT_BUTTON, self.sendToH, sendToH)

        sendToI = wx.Button(self, wx.ID_ANY, label="Send to I")
        grid_sizer.Add(sendToI, (4, 2))
        self.Bind(wx.EVT_BUTTON, self.sendToI, sendToI)



        initFollowPose = wx.Button(self, wx.ID_ANY, label="Init follow")
        grid_sizer.Add(initFollowPose, (4, 0))
        self.Bind(wx.EVT_BUTTON, self.initFollowPose, initFollowPose)

        #  self.delayTime = wx.TextCtrl(self, wx.ID_ANY, value=u"0")
        #  grid_sizer.Add(self.delayTime, (4, 1))

        initOppositePose1 = wx.Button(self, wx.ID_ANY, label="Init opposite1")
        grid_sizer.Add(initOppositePose1, (5, 0))
        self.Bind(wx.EVT_BUTTON, self.initOppositePose1, initOppositePose1)

        sendOppositeGoal1 = wx.Button(self, wx.ID_ANY, label="Send opposite1 Goal")
        grid_sizer.Add(sendOppositeGoal1, (5, 1))
        self.Bind(wx.EVT_BUTTON, self.sendOppositeGoal1, sendOppositeGoal1)

        initOppositePose2 = wx.Button(self, wx.ID_ANY, label="Init opposite2")
        grid_sizer.Add(initOppositePose2, (6, 0))
        self.Bind(wx.EVT_BUTTON, self.initOppositePose2, initOppositePose2)

        sendOppositeGoal2 = wx.Button(self, wx.ID_ANY, label="Send opposite2 Goal")
        grid_sizer.Add(sendOppositeGoal2, (6, 1))
        self.Bind(wx.EVT_BUTTON, self.sendOppositeGoal2, sendOppositeGoal2)


        initPoseOne = wx.Button(self, wx.ID_ANY, label="PoseOne: 0:A 1,2:B")
        grid_sizer.Add(initPoseOne, (7, 0))
        self.Bind(wx.EVT_BUTTON, self.initPoseOne, initPoseOne)

        actionOne = wx.Button(self, wx.ID_ANY, label="ActionOne: 0->B A<-1,2")
        grid_sizer.Add(actionOne, (7, 1))
        self.Bind(wx.EVT_BUTTON, self.actionOne, actionOne)

        initPoseTwo = wx.Button(self, wx.ID_ANY, label="PoseTwo: 0:A 1:B 2:C")
        grid_sizer.Add(initPoseTwo, (8, 0))
        self.Bind(wx.EVT_BUTTON, self.initPoseTwo, initPoseTwo)

        actionTwo = wx.Button(self, wx.ID_ANY, label="ActionTwo: 0->B A<-1 2->D")
        grid_sizer.Add(actionTwo, (8, 1))
        self.Bind(wx.EVT_BUTTON, self.actionTwo, actionTwo)


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
            self.choiceBox.Append(msg.robot_id)

    def initFollowPose(self, event):
        string = "%s follow pose" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def initPoseOne(self, event):
        string = "%s pose one" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))
    
    def actionOne(self, event):
        string = "%s action one" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def initPoseTwo(self, event):
        string = "%s pose two" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))
    
    def actionTwo(self, event):
        string = "%s action two" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))


    def sendToE(self, event):
        string = "%s to E" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def sendToF(self, event):
        string = "%s to F" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def sendToG(self, event):
        string = "%s to G" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def sendToH(self, event):
        string = "%s to H" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def sendToI(self, event):
        string = "%s to I" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def sendToJ(self, event):
        string = "%s to J" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))


    def sendOppositeGoal1(self, event):
        string = "%s opposite1 Goal" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def initOppositePose1(self, event):
        string = "%s opposite1 pose" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def initOppositePose2(self, event):
        string = "%s opposite2 pose" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def sendOppositeGoal2(self, event):
        string = "%s opposite2 Goal" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))


    def stop(self, event):
        string = "%s Stop" % self.choiceBox.GetStringSelection()
        self.pub.publish(str(string))

    def start(self, event):
        string = "%s next Goal" % self.choiceBox.GetStringSelection()
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
