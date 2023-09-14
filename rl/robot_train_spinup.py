""" -------------------------------------------- SPINNING UP --------------------------------------------------------"""
#from spinup import ppo_tf1 as ppo
#from spinup import ddpg_tf1 as ddpg
from spinup import sac_tf1 as sac
from spinup import trpo_tf1 as trpo
from spinup import vpg_tf1 as vpg
from spinup import td3_tf1 as td3
from spinup import ddpg_pytorch as ddpg
from spinup import ppo_pytorch as ppo

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
sys.path.insert(1, "/opt/ros/kinetic/lib/python2.7/dist-packages")
#print(colorize(sys.path, 'red', bold=True))
from gym import wrappers
import gym
import tensorflow as tf
import torch
import rospy
import numpy as np



class robot_train(object):
    def __init__(self, env_fn, name):
        self.env_fn = env_fn
        self.name = name
        print(colorize(" ------------- Training for {} --------------- ".format(self.name) , 'green', bold=True))


    def train(self):

        ac_kwargs = dict(hidden_sizes=[64, 64])
        logger_kwargs = dict(output_dir='path/to/your/directory'.format(self.name), exp_name=self.name)

        ppo(env_fn=self.env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=3000, epochs=250, logger_kwargs=logger_kwargs)
