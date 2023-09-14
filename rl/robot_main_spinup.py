""" -------------------------------------------- SPINNING UP --------------------------------------------------------"""
from robot_train_spinup import robot_train
from spinup.utils.test_policy import load_policy_and_env, run_policy
from spinup.utils.run_utils import call_experiment, ExperimentGrid
from spinup import ddpg_tf1


import sys
sys.path.insert(2, '/home/emmarapt/gym_gazebo_kinetic_python36') # use your own path
import gym_gazebo # clone https://github.com/erlerobot/gym-gazebo.git
import gym
import tensorflow

"""
Initialize your gym environment
"""
env = gym.make('GazeboTurtlebot3OpenManipulatorEnv-v0')
name = env.spec.id  # store id

ON_TRAIN = True # True when you want to TRAIN your agent else False
ON_TUNE = False  # True when you want to TUNE your agent else False

def evaluate(env):
    print("\n\n\n")
    print("----------------------------------------------------------------------------")
    print("Visualizing the results for the {}".format(name) + " " + "environment")
    print("----------------------------------------------------------------------------")
    _, get_action = load_policy_and_env(
        'spinningup/trained_data/path/to/output_dir/{}/ppo_wafflepi_om_ITTcs_reward_HSC_nav'.format(name))
    run_policy(env, get_action)

    """
    Plot results: python -m spinup.run plot [path/to/output_directory ...] [--legend [LEGEND ...]]
    [--xaxis XAXIS] [--value [VALUE ...]] [--count] [--smooth S]
    [--select [SEL ...]] [--exclude [EXC ...]]
    """

if __name__ == '__main__':
    
    if ON_TRAIN:
        """
        Tool for running many experiments given hyperparameter ranges.
        """
        if ON_TUNE:
            import argparse

            parser = argparse.ArgumentParser()
            parser.add_argument('--cpu', type=int, default=4)
            parser.add_argument('--num_runs', type=int, default=3)
            args = parser.parse_args()

            Tuning = ExperimentGrid(name='RoboticArm:MyPendulum_env-v0')
            Tuning.add('env_name', 'RoboticArm:MyPendulum_env-v0', '', True)

            Tuning.add('seed', [10 * i for i in range(args.num_runs)])
            Tuning.add('epochs', 30)
            Tuning.add('steps_per_epoch', 4000)
            Tuning.add('ac_kwargs:hidden_sizes', [(32, 32), (64, 64), (128, 128), ], 'hid')
            Tuning.add('ac_kwargs:activation', [tensorflow.nn.tanh, tensorflow.nn.relu], '')

            Tuning.print()

            Tuning.run(ddpg_tf1, num_cpu=args.cpu)

        else:
            """
            Train
            """
            env_for_train = lambda: env  # use lambda for training
            robot_train(env_for_train, name).train()
            #robot_train.train()
    else:
        evaluate(env)
