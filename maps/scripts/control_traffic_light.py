#!/usr/bin/env python3

import rospy
import rospkg
import os
import time
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel, DeleteModel

from geometry_msgs.msg import Pose

class ControlTrafficLights():
    def __init__(self):
        self.traffic_state = 1 #1=Red to Red/Yellow light 

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

        self.rate = rospy.Rate(10)
        self.state_publisher = rospy.Publisher('traffic_light_state', String, queue_size=1)
        self.getTrafficLightState()

    def loadTrafficLightModel(self):
        self.pose = Pose()

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        delete_model_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

        spawn_model_proxy('traffic_stand', self.traffic_stand_model, '', self.pose, 'world')
        spawn_model_proxy('red_light', self.red_light_model, '', self.pose, 'world')

    def getTrafficLightState(self):
        while not rospy.is_shutdown():
            rospy.logerr("allllooo")

            if self.traffic_state == 0:      #turn from yellow to red light
                if abs(self.current_time - time.time()) > 2:
                    rospy.logerr("turn from yellow to red light")

                    delete_model_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                    delete_model_proxy('yellow_light')

                    self.state_publisher.publish("Red")

                    rospy.wait_for_service('gazebo/spawn_sdf_model')
                    spawn_model_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                    spawn_model_proxy('red_light', self.red_light_model, '', self.pose, 'world')

                    self.traffic_state = 1
                    self.current_time = time.time()

            elif self.traffic_state == 1:    #turn from red to red/yellow light
                if abs(self.current_time - time.time()) > 10:
                    rospy.logerr("turn from red to red/yellow light")

                    self.state_publisher.publish("Red/Yellow")

                    rospy.wait_for_service('gazebo/spawn_sdf_model')
                    spawn_model_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                    spawn_model_proxy('yellow_light', self.yellow_light_model, '', self.pose, 'world')
                    
                    self.traffic_state = 2
                    self.current_time = time.time()

            elif self.traffic_state == 2:    #turn from red/yellow to green light
                if abs(self.current_time - time.time()) > 2:
                    rospy.logerr("turn from red/yellow to green light")

                    delete_model_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                    delete_model_proxy('red_light')
                    delete_model_proxy('yellow_light')

                    self.state_publisher.publish("Green")

                    rospy.wait_for_service('gazebo/spawn_sdf_model')
                    spawn_model_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                    spawn_model_proxy('green_light', self.green_light_model, '', self.pose, 'world')

                    self.traffic_state = 3
                    self.current_time = time.time()

            elif self.traffic_state == 3:    #turn from green to yellow light
                if abs(self.current_time - time.time()) > 10:
                    rospy.logerr("turn from green to yellow light")

                    delete_model_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                    delete_model_proxy('green_light')

                    self.state_publisher.publish("Yellow")

                    rospy.wait_for_service('gazebo/spawn_sdf_model')
                    spawn_model_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                    spawn_model_proxy('yellow_light', self.yellow_light_model, '', self.pose, 'world')

                    self.traffic_state = 0
                    self.current_time = time.time()

            self.rate.sleep()


def main():
    rospy.init_node('traffic_lights_controller')
    try:
        traffic_lights_cont = ControlTrafficLights()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()