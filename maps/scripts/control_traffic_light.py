#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class ControlTrafficLights():
    def __init__(self):
        self.traffic_state = 0
        self.loadTrafficLightModel()

        self.rate = rospy.Rate(10)
        self.state_publisher = rospy.Publisher('state', String, queue_size=3)
        self.getTrafficLightState()

    def loadTrafficLightModel(self):
        rospy.wait_for_service('gazebo/spawn_sdf_model')

    def controlLights(self):
        rospy.logerr("Hallloooooo")

    def getTrafficLightState(self):
        while not rospy.is_shutdown():
            rospy.logerr("Hallloooooo")
            self.state_publisher.publish("Green")
            self.rate.sleep()

def main():
    rospy.init_node('traffic_lights_controller')
    try:
        traffic_lights_cont = ControlTrafficLights()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()