import subprocess
import gym
import sys
sys.path.insert(1, '/home/emmarapt/gym_gazebo_kinetic_python36') # insert the project location
from spinup.utils.test_policy import load_policy_and_env, run_policy
from examples.chatgpt_aspida.Terminal_Launcher import terminal # use your own path


# The position of the objects e.g. Kitchen, Toilet, Computer room, Human etc. should have the position of the AR markers.
objects_dict = {
    "Kitchen": [0.904000, 0.262698], #[1.000449, 0.390001], #[0.709395, 0.368721], #   # Marker0
    "Human": [0.995255, 0.966961], #[0.899999,0.984682],  # Marker4
    "Robot": [0, 0]
}


class ASPiDAWrapper:
    def __init__(self):
        self.window_id = terminal()

    """RL Policy"""
    def navigation_to_target(self, targetx, targety):
        env = gym.make('GazeboTurtlebot3OpenManipulatorEnv-v0', targetx=targetx, targety=targety)
        name = env.spec.id  # store id

        _, get_action = load_policy_and_env(
            'ppo_wafflepi_om_robotis_reward') # use your own path
        run_policy(env, get_action)  # in run_policy i have modified num_episodes=1

    def focus_on_kitchen(self):
        subprocess.call(['xdotool', 'windowactivate', '--sync', self.window_id[4], 'type',
                         'rostopic pub -1 /tb3_hsc/command std_msgs/String restart_mission:find_object_1\n'])

    def reach(self):
        pass

    def pick(self):
        subprocess.call(['xdotool', 'windowactivate', '--sync', self.window_id[4], 'type',
                         'rostopic pub -1 /tb3_hsc/command std_msgs/String restart_mission:pick_object_1\n'])

    def leave_kitchen(self):
        subprocess.call(['xdotool', 'windowactivate', '--sync', self.window_id[4], 'type',
                         'rostopic pub -1 /tb3_hsc/command std_msgs/String restart_mission:leave_object_1\n'])

    def place(self):
        subprocess.call(['xdotool', 'windowactivate', '--sync', self.window_id[4], 'type',
                         'rostopic pub -1 /tb3_hsc/command std_msgs/String restart_mission:nav_to_target_1\n'])

    def get_position(self, object_name):
        query_string = objects_dict[object_name]
        return query_string[0], query_string[1]


    # ROS-based functions
    # def kill_gzclient(self):
    #     subprocess.call(['xdotool', 'windowactivate', '--sync', window_id[4], 'type',
    #                      ' killall -9 gzclient\n'])
    # def kill_gzserver(self):
    #     subprocess.call(['xdotool', 'windowactivate', '--sync', window_id[4], 'type',
    #                      ' killall -9 gzserver\n'])
