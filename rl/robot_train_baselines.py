""" -------------------------------------------- BASELINES ----------------------------------------------------------"""
# from stable_baselines.common.policies import MlpPolicy
# from stable_baselines.common.vec_env import DummyVecEnv
# from stable_baselines import PPO2

color2num = dict(
    gray=30,
    red=31,
    green=32,
    yellow=33,
    blue=34,
    magenta=35,
    cyan=36,
    white=37,
    crimson=38
)

def colorize(string, color, bold=False, highlight=False):
    """
    Colorize a string.

    This function was originally written by John Schulman.
    """
    attr = []
    num = color2num[color]
    if highlight: num += 10
    attr.append(str(num))
    if bold: attr.append('1')
    return '\x1b[%sm%s\x1b[0m' % (';'.join(attr), string)


import sys
#print(colorize(sys.path, 'red', bold=True))
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import baselines.run as run
sys.path.insert(1, "/opt/ros/kinetic/lib/python2.7/dist-packages")
#print(colorize(sys.path, 'red', bold=True))
from gym import wrappers
import gym
import tensorflow as tf
import torch
import rospy
import numpy as np
#from stable_baselines.common.env_checker import check_env


class robot_train(object):
    def __init__(self, env_fn, name):
        self.env_fn = env_fn
        self.name = name
        print(colorize(" ------------- Training for {} --------------- ".format(self.name) , 'green', bold=True))

    def train(self):

        """ Stable Baselines """
        from stable_baselines3 import PPO
        from stable_baselines3.common.evaluation import evaluate_policy

        # Train the agent
        model = PPO("MlpPolicy", self.env_fn, verbose=1)
        model.learn(total_timesteps=250000, progress_bar=True)
        model.save("/media/emmarapt/DATA/stable_baselines/trained_data/path/to/output_dir/{}/ppo_wafflepi_om_ITTcs_reward_HSC".format(self.name))

        # Load the trained agent
        # model = PPO.load("ppo_cartpole", env=self.env_fn)
        # mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
        # vec_env = model.get_env()
        # obs = vec_env.reset()
        # for i in range(1000):
        #     action, _states = model.predict(obs, deterministic=True)
        #     obs, rewards, dones, info = vec_env.step(action)
        #     vec_env.render()

        # from stable_baselines import DDPG
        # from stable_baselines.ddpg.policies import MlpPolicy
        # from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
        #
        # n_actions = self.env_fn.action_space.shape[-1]
        #
        # action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))
        # model = DDPG(MlpPolicy, self.env_fn, verbose=1, param_noise=None, action_noise=action_noise)
        # model.learn(total_timesteps=400000)
        # model.save('baselines/trained_data/path/to/output_dir/{}'.format(self.name))


        """ Baselines """
        # #env_name = 'GazeboCircuit2Rb1LidarEnv-v0'
        # alg = 'ddpg'
        # num_timesteps = '1e50'
        # save_path = 'your/path'.format(self.name)
        #
        # my_args = [
        #     '--alg=' + alg,
        #     '--env=' + self.env_fn.spec.id,
        #     '--num_timesteps=' + num_timesteps,
        #     '--save_path=' + save_path]
        #
        # #from gym_gazebo.envs import gazebo_env, real_env
        # gazebo_env.GazeboEnv.__init__(self, "competition.launch")

        # name_ref = self.env_fn.spec.id + '_' + num_timesteps + '_' + alg
        #
        # # import os
        # # os.environ["OPENAI_LOG_FORMAT"] = "csv"
        # # os.environ["OPENAI_LOGDIR"] = './logs/' + name_ref
        #
        #
        # #rospy.init_node(self.name.replace('-', '_'))
        # # """ Check that an environment follows Gym API """
        # # check_env(self.env_fn)
        #
        # """ Run - Train """
        # run.main(my_args)

