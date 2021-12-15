#!/usr/bin/env python3

import rospy
import rospkg
import os
import time
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import tf

class ControlTrafficLights():
    def __init__(self):
        self.traffic_state = 1 #0=Yellow, 1=Red, 2=Yellow/Red, 3=Green

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('maps')

        model_path_traffic_stand = pkg_path + '/models/traffic_light/traffic_bar_stand/model.sdf'
        traffic_stand_model = open(model_path_traffic_stand, 'r')
        self.traffic_stand_model = traffic_stand_model.read()

        model_path_green_light = pkg_path + '/models/traffic_light/lights/green_light/model.sdf'
        green_light_model = open(model_path_green_light, 'r')
        self.green_light_model = green_light_model.read()

        model_path_yellow_light = pkg_path + '/models/traffic_light/lights/yellow_light/model.sdf'
        yellow_light_model = open(model_path_yellow_light, 'r')
        self.yellow_light_model = yellow_light_model.read()

        model_path_red_light = pkg_path + '/models/traffic_light/lights/red_light/model.sdf'
        red_light_model = open(model_path_red_light, 'r')
        self.red_light_model = red_light_model.read()

        self.current_time = time.time()
        self.loadTrafficLightModel()
        self.resetLights()

        self.rate = rospy.Rate(10)

    def loadTrafficLightModel(self):
        self.pose = Pose()
        self.pose.position.x = 0.2
        self.pose.position.y = -0.2
        self.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, -1.565)
        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)    

        spawn_model_proxy('traffic_stand', self.traffic_stand_model, '', self.pose, 'world')
        spawn_model_proxy('red_light', self.red_light_model, '', self.pose, 'world')
        spawn_model_proxy('yellow_light', self.yellow_light_model, '', self.pose, 'world')
        spawn_model_proxy('green_light', self.green_light_model, '', self.pose, 'world')

        time.sleep(0.5)

    def changeModelPosition(self, x, y, z, yaw, model_name):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
           set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        except rospy.ServiceException :
            print("Service call failed")

        state_msg = ModelState()

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]

        state_msg.model_name = model_name

        set_state(state_msg)

        time.sleep(0.05)

    def updateTrafficLightState(self):
        if self.traffic_state == 0:      #turn from yellow to red light
            if abs(self.current_time - time.time()) > 2:
                rospy.logerr("turn from yellow to red light")

                self.changeModelPosition(0.2, -0.2, -1, -1.565, 'yellow_light')
                self.changeModelPosition(0.2, -0.2, 0, -1.565, 'red_light')

                self.traffic_state = 1

                self.current_time = time.time()

        elif self.traffic_state == 1:    #turn from red to red/yellow light
            if abs(self.current_time - time.time()) > 10:
                rospy.logerr("turn from red to red/yellow light")

                #self.state_publisher.publish("Red/Yellow")

                self.changeModelPosition(0.2, -0.2, 0, -1.565, 'yellow_light')

                self.traffic_state = 2
                self.current_time = time.time()

        elif self.traffic_state == 2:    #turn from red/yellow to green light
            if abs(self.current_time - time.time()) > 2:
                rospy.logerr("turn from red/yellow to green light")

                self.changeModelPosition(0.2, -0.2, -1, -1.565, 'yellow_light')
                self.changeModelPosition(0.2, -0.2, -1, -1.565, 'red_light')

                #self.state_publisher.publish("Green")

                self.changeModelPosition(0.2, -0.2, 0, -1.565, 'green_light')

                self.traffic_state = 3
                self.current_time = time.time()

        elif self.traffic_state == 3:    #turn from green to yellow light
            if abs(self.current_time - time.time()) > 10:
                rospy.logerr("turn from green to yellow light")

                self.changeModelPosition(0.2, -0.2, -1, -1.565, 'green_light')

                #self.state_publisher.publish("Yellow")

                self.changeModelPosition(0.2, -0.2, 0, -1.565, 'yellow_light')

                self.traffic_state = 0
                self.current_time = time.time()

        self.rate.sleep()

    def resetLights(self):
        if self.traffic_state == 0:      #yellow light
            self.changeModelPosition(0.2, -0.2, -1, -1.565, 'green_light')
            self.changeModelPosition(0.2, -0.2, 0, -1.565, 'yellow_light')
            self.changeModelPosition(0.2, -0.2, -1, -1.565, 'red_light')

        elif self.traffic_state == 1:    #red light
            self.changeModelPosition(0.2, -0.2, -1, -1.565, 'green_light')
            self.changeModelPosition(0.2, -0.2, -1, -1.565, 'yellow_light')
            self.changeModelPosition(0.2, -0.2, 0, -1.565, 'red_light')

        elif self.traffic_state == 2:    #red/yellow light
            self.changeModelPosition(0.2, -0.2, -1, -1.565, 'green_light')
            self.changeModelPosition(0.2, -0.2, 0, -1.565, 'yellow_light')
            self.changeModelPosition(0.2, -0.2, 0, -1.565, 'red_light')

        elif self.traffic_state == 3:    #green light
            self.changeModelPosition(0.2, -0.2, 0, -1.565, 'green_light')
            self.changeModelPosition(0.2, -0.2, -1, -1.565, 'yellow_light')
            self.changeModelPosition(0.2, -0.2, -1, -1.565, 'red_light')