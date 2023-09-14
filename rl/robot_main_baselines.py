""" This code includes both baselines and stable-baselines framework for training/evaluating a reinforcement learning agent.
Comment/Uncomment based on your preferences"""

from robot_train_baselines import robot_train
import baselines.run as run

import rospy
import sys
sys.path.insert(1, '/home/emmarapt/gym_gazebo_kinetic_python36')
import gym_gazebo
import gym
import tensorflow

"""
Initialize your gym environment
"""

env = gym.make('GazeboTurtlebot3OpenManipulatorEnv-v0')

name = env.spec.id  # store id

ON_TRAIN = True # True when you want to TRAIN your agent else False
ON_TUNE = False  # True when you want to TUNE your agent else False


def evaluate_baselines(env):
    """Code to run Baselines"""
    # alg = 'ppo2'
    # num_timesteps = '1e4'
    # env_name = 'GazeboCircuit2Rb1LidarEnv-v0'
    # save_path = '/home/emmarapt/gym_gazebo_kinetic_python36/examples/baselines/trained_data/ppo2/path/to/output_dir/{}'.format(env_name)
    #
    # my_args = [
    #     '--alg=' + alg,
    #     '--env=' + env_name,
    #     '--num_timesteps=' + num_timesteps,
    #     '--load_path=' + save_path,
    #     '--play']
    #
    # run.main(my_args)


    """Code to run Stable Baselines"""
    from stable_baselines3 import PPO
    from stable_baselines3.common.evaluation import evaluate_policy

    # Load the trained agent
    model = PPO.load("your/path".format(name), env=env)
    evaluate_policy(model, model.get_env(), n_eval_episodes=1)
    # mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
    # vec_env = model.get_env()
    # obs = vec_env.reset()
    # for i in range(1000):
    #     action, _states = model.predict(obs, deterministic=True)
    #     obs, rewards, dones, info = vec_env.step(action)
    #     vec_env.render()



if ON_TRAIN:
    robot_train(env, name).train()
else:
    evaluate_baselines(env)



