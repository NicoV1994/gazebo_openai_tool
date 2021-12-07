import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import turtlebot3camera_env
from gym.envs.registration import register
from geometry_msgs.msg import Vector3
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os


class TurtleBot3WorldEnv(turtlebot3camera_env.TurtleBot3Env):
    def __init__(self):
        """
        This Task Env is designed for having the TurtleBot3 in the turtlebot3 world course1.
        It will learn how to move around without crashing.
        """
        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/turtlebot3/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        robot_launch_file = 'spawn_turtlebot3_lane_recognition.launch'#rospy.get_param("/turtlebot3/robot_launch_file", None)


        ROSLauncher(rospackage_name="maps",
                    launch_file_name="lane_recognition_course1.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/turtlebot3/config",
                               yaml_file_name="turtlebot3_lane_regonition_course1.yaml")


        # Here we will add any init functions prior to starting the MyRobotEnv
        super(TurtleBot3WorldEnv, self).__init__(ros_ws_abspath,robot_launch_file)

        # Only variable needed to be set here
        number_actions = rospy.get_param('/turtlebot3/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)

        # Actions and Observations
        self.linear_forward_speed = rospy.get_param('/turtlebot3/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/turtlebot3/linear_turn_speed')
        self.angular_speed = rospy.get_param('/turtlebot3/angular_speed')
        self.init_linear_forward_speed = rospy.get_param('/turtlebot3/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param('/turtlebot3/init_linear_turn_speed')

        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        camera_image = self.get_camera_image()

        # We only use two integers
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))

        # Rewards
        self.forwards_reward = rospy.get_param("/turtlebot3/forwards_reward")
        self.turn_reward = rospy.get_param("/turtlebot3/turn_reward")
        self.end_episode_points = rospy.get_param("/turtlebot3/end_episode_points")

        self.cumulated_steps = 0.0


    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base( self.init_linear_forward_speed,
                        self.init_linear_turn_speed,
                        epsilon=0.05,
                        update_rate=10)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False


    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the turtlebot2
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        if action == 0: #FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARDS"
        elif action == 1: #LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        elif action == 2: #RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            self.last_action = "TURN_RIGHT"

        # We tell TurtleBot3 the linear and angular speed to set to execute
        self.move_base(linear_speed, angular_speed, epsilon=0.05, update_rate=10)

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        TurtleBot3Env
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # We get the camera image
        camera_image = self.get_camera_image()

        discretized_observations = self.discretize_camera_observation(camera_image)

        rospy.logdebug("Observations==>"+str(discretized_observations))
        rospy.logdebug("END Get Observation ==>")

        return discretized_observations


    def _is_done(self, observations):

        # Now we check if it has crashed based on the contact state
        contacts_state_data = self.get_contacts_state()
        
        if (contacts_state_data.states != []):
            rospy.logerr("TurtleBot3 Failed")
            self._episode_done = True
        else:
            rospy.logerr("TurtleBot3 DIDNT Fail")


        return self._episode_done

    def _compute_reward(self, observations, done):

        if not done:
            if self.last_action == "FORWARDS":
                reward = self.forwards_reward
            else:
                reward = self.turn_reward
        else:
            reward = -1*self.end_episode_points


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward


    # Internal TaskEnv Methods

    def discretize_camera_observation(self,data):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """
        self._episode_done = False

        discretized_image = data.data

        return discretized_image