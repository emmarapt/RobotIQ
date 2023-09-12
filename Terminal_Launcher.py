import os
import subprocess
import time
#os.system("gnome-terminal -e 'bash -c \"cd ~/python36_ws; exec bash\"'")

def terminal():
    # Open Terminal 1
    os.system("gnome-terminal")
    time.sleep(0.5)

    #Open Terminal 2
    os.system("gnome-terminal")
    time.sleep(0.5)

    #Open Terminal 3
    os.system("gnome-terminal")
    time.sleep(0.5)

    #Open Terminal 4
    os.system("gnome-terminal")
    time.sleep(0.5)

    # get the terminal window ID
    window_id = subprocess.check_output(['xdotool', 'search', '--onlyvisible', '--class', 'gnome-terminal']).strip().decode('utf-8')
    window_ids = window_id.split('\n')

    """Terminal No. 1"""
    # send the command to activate virtual environment and execute desired command
    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[1], 'type', 'cd ~/python36_ws\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[1], 'type', 'source py36env/bin/activate\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[1], 'type', 'source /opt/ros/kinetic/setup.bash\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[1], 'type', 'source /home/emmarapt/python36_ws/devel/setup.bash\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[1], 'type', 'cd src\n'])

    # Load Turtlebot3 with OpenManipulator-X in gazebo
    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[1], 'type', 'roslaunch turtlebot3_home_service_challenge_simulation competition.launch\n'])
    #
    """Terminal No. 2"""
    # send the command to activate virtual environment and execute desired command
    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[2], 'type', 'cd ~/python36_ws\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[2], 'type', 'source py36env/bin/activate\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[2], 'type', 'source /opt/ros/kinetic/setup.bash\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[2], 'type', 'source /home/emmarapt/python36_ws/devel/setup.bash\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[2], 'type', 'cd src\n'])

    time.sleep(6)
    # # Load Turtlebot3 with OpenManipulator-X in gazebo
    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[2], 'type', 'roslaunch turtlebot3_home_service_challenge_tools turtlebot3_home_service_challenge_demo_simulation.launch\n'])
    #subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[2], 'type',
     #                'roslaunch turtlebot3_home_service_challenge_tools turtlebot3_home_service_challenge_demo_remote.launch address:=192.168.1.5\n'])

    """Terminal No. 3"""
    # send the command to activate virtual environment and execy36env/binute desired command
    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[3], 'type', 'cd ~/python36_ws\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[3], 'type', 'source py36env/bin/activate\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[3], 'type', 'source /opt/ros/kinetic/setup.bash\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[3], 'type', 'source /home/emmarapt/python36_ws/devel/setup.bash\n'])

    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[3], 'type', 'cd src\n'])

    time.sleep(4)
    # Load Turtlebot3 with OpenManipulator-X in gazebo
    subprocess.call(['xdotool', 'windowactivate', '--sync', window_ids[3], 'type', 'roslaunch turtlebot3_home_service_challenge_manager manager.launch\n'])

    time.sleep(4)

    return window_ids