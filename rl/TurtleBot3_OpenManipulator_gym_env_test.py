"""The goal is to test the turtlebot mobile robot to reach a target in an unknown environment while avoiding any obstacles"""

import gym
import rospy
import roslaunch
import time
import numpy as np
import math
import os
import random
from typing import Optional

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env, real_env
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from gym.utils import seeding
from nav_msgs.msg import Odometry
# from transformations import euler_from_quaternion, quaternion_from_euler
from gym_gazebo.envs.turtlebot.mytf import euler_from_quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gym_gazebo.envs.turtlebot.respawnGoal import respawn

# goal_model_dir = '/home/emmarapt/gym_gazebo_kinetic_python36/gym_gazebo/envs/assets/models/PatientWheelChair/model.sdf'
goal_model_dir = '/home/emmarapt/gym_gazebo_kinetic_python36/gym_gazebo/envs/assets/models/Target/model.sdf'


class GazeboTurtlebot3OpenManipulatorEnv(gym.Env):
    def __init__(self, targetx, targety):
        rospy.init_node('gym', anonymous=True)
        """ Env Variables """
        continuous = False
        observation_size = 360 + 4  # /scan + [heading, current_distance] + obstacles [min, angle]
        action_size = 5  # angle: 0, 45, 90, 135, 180 degrees   –– \ | / –– [- 1.5, - 0.75, 0, 0,75, 1.5]

        min_ang_vel = -1.5
        max_ang_vel = 1.5
        min_range = 0.120  # lds_lfcd_sensor
        max_range = 3.5
        self.maximum_distance_from_goal = 100

        # initialize variables
        self.test_reward = 0
        self.steps_reward = 0

        self.continuous = continuous
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.goal_position = Pose()
        self.goal_position.position.x = targetx
        self.goal_position.position.y = targety
        self.heading = 0
        self.past_distance = 0.
        """ Set up ros publishers & subscribers """
        self.pub_cmd_vel = rospy.Publisher('/tb3_hsc/cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('/tb3_hsc/odom', Odometry, self.getOdometry)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        # self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.goal = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.del_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        self.observation_size = observation_size
        self.action_size = action_size
        self.min_ang_vel = min_ang_vel
        self.max_ang_vel = max_ang_vel
        self.min_range = min_range
        self.max_range = max_range

        if self.continuous:
            self.action_space = spaces.Box(low=self.min_ang_vel, high=self.max_ang_vel, shape=(1,), dtype=np.float32)
        else:
            self.action_space = spaces.Discrete(5)
            self.ang_step = max_ang_vel / ((action_size - 1) / 2)
            self.actions = [((action_size - 1) / 2 - action) * self.ang_step for action in
                            range(action_size)]  # [0.5, 0.25, 0.0, -0.25, -0.5]

        low, high = self.get_observation_space_values()
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

    def get_observation_space_values(self):
        low = np.append(np.full(self.observation_size - 2, self.min_range), np.array([-math.pi, 0], dtype=np.float32))
        high = np.append(np.full(self.observation_size - 2, self.max_range),
                         np.array([math.pi, self.maximum_distance_from_goal], dtype=np.float32))
        return low, high

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def getGoalDistance(self):
        goal_distance = round(math.hypot(self.goal_position.position.x - self.position.position.x,
                                         self.goal_position.position.y - self.position.position.y), 2)
        self.past_distance = goal_distance
        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        pitch, roll, yaw = euler_from_quaternion(orientation_list)  # pitch and roll are not used

        # Check goal angle from current position
        goal_angle = math.atan2(self.goal_position.position.y - self.position.position.y,
                                self.goal_position.position.x - self.position.position.x)

        heading = goal_angle - yaw
        if heading > math.pi:
            heading -= 2 * math.pi

        elif heading < -math.pi:
            heading += 2 * math.pi

        self.heading = heading

    def getState(self, scan):  # , new_ranges):
        scan_ranges = []
        heading = self.heading
        collision = 0.22
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_ranges.append(self.max_range)
            elif np.isnan(scan.ranges[i]):
                scan_ranges.append(self.min_range)
            else:
                scan_ranges.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_ranges), 2)
        obstacle_angle = np.argmin(scan_ranges)

        # check collision
        if collision > min(scan_ranges) > 0:
          done = True

        current_distance = self.getGoalDistance()
        if current_distance < 0.20:
            self.get_goalbox = True
            done = True

        return scan_ranges + [heading, current_distance, obstacle_min_range, obstacle_angle], done

    def setReward(self, state, done, action):

        """Robotis Reward"""
        yaw_reward = []
        obstacle_min_range = state[-2]
        current_distance = state[-3]
        heading = state[-4]

        angle = -math.pi / 4 + heading + (math.pi / 8) + math.pi / 2
        tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
        yaw_reward = tr
        distance_rate = 2 ** (current_distance / self.goal_distance)

        if obstacle_min_range < 0.5:
            ob_reward = -5
        else:
            ob_reward = 0

        reward = ((round(yaw_reward * 5, 2)) * distance_rate) + ob_reward

        # Give a large negative reward for actions that collide the robot
        if done:
            rospy.loginfo("Collision detected!!")
            reward = -500
            self.pub_cmd_vel.publish(Twist())  # publish an empty Twist message to stop robot motion
            self.goal_distance = self.getGoalDistance()

        # Give a large positive reward for reaching the goal
        if self.get_goalbox:
            reward = 1000
            rospy.loginfo("Goal reached!!")
            self.pub_cmd_vel.publish(Twist())  # publish an empty Twist message to stop robot motion
            self.goal_distance = self.getGoalDistance()
            self.get_goalbox = False

        return reward


    def set_ang_vel(self, action):
        if self.continuous:
            self.ang_vel = action
        else:
            self.ang_vel = self.actions[action]

    def log_parameter_values(self, filename="parameter_log_real__.txt"):
        with open(filename, "a") as file:
            file.write(
                f"Time Step: {self.steps_reward}, Test Reward: {self.test_reward}, Steps Reward: {self.steps_reward}\n")

    def step(self, action):

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            # resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        self.set_ang_vel(action)

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15  # constant linear velociry
        vel_cmd.angular.z = self.ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/tb3_hsc/scan', LaserScan, timeout=5)
            except:
                pass

        # pause simulation to get state/reward
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        state, done = self.getState(data)
        reward = self.setReward(state, done, action)

        self.test_reward = self.test_reward + reward
        self.steps_reward = self.steps_reward + 1

        # Log the parameter values to a text file
        self.log_parameter_values()

        return np.asarray(state), reward, done, {}


    def reset(self):
        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            # resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/tb3_hsc/scan', LaserScan, timeout=5)
            except:
                pass


        # Build the target - respawn goal target
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            goal_urdf = open(goal_model_dir, "r").read()
            target = SpawnModel
            target.model_name = 'target'  # the same with sdf name
            target.model_xml = goal_urdf
            #self.goal_position.position.x, self.goal_position.position.y = respawn()
            self.goal(target.model_name, target.model_xml, 'namespace', self.goal_position, 'world')
        except (rospy.ServiceException) as e:
            print("/gazebo/failed to build the target")


        self.goal_distance = self.getGoalDistance()

        state, done = self.getState(data)

        if done:
            rospy.wait_for_service('/gazebo/delete_model')
            # delete current target
            self.del_model('target')

        return np.asarray(state)
    #
    def close(self):
        pass

    def render(self, mode='human'):
        pass