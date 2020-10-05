import subprocess, math, time, sys, os, numpy as np
import matplotlib.pyplot as plt
import pybullet as bullet_simulation
import pybullet_data

# setup paths and load the core
abs_path = os.path.dirname(os.path.realpath(__file__))
root_path = abs_path + '/..'
core_path = root_path + '/core'
sys.path.append(core_path)
from Pybullet_Simulation import Simulation

# specific settings for this task
taskId = 2

try:
    if sys.argv[1] == 'nogui':
        gui = False
    else:
        gui = True
except:
    gui = True

global task2_endEffectorName  # same as the joint name
global task2_targetPosition
global task2_endEffectorOrientation



### You may want to change the code since here
pybulletConfigs = {
    "simulation": bullet_simulation,
    "pybullet_extra_data": pybullet_data,
    "gui": gui,
    "panels": False,
    "realTime": False,
    "controlFrequency": 1000,
    "updateFrequency": 250,
    "gravity": -9.81,
    "gravityCompensation": 1.,
    "floor": True,
    "cameraSettings": (1.07, 90.0, -52.8, (0.07, 0.01, 0.76))
}
robotConfigs = {
    "robotPath": core_path + "/nextagea_description/urdf/NextageaOpen.urdf",
    "robotPIDConfigs": core_path + "/PD_gains.yaml",
    "robotStartPos": [0, 0, 0.85],
    "robotStartOrientation": [0, 0, 0, 1],
    "fixedBase": True,
    "colored": False
}
sim = Simulation(pybulletConfigs, robotConfigs)

task2_endEffectorName = "LARM_JOINT2"
task2_targetPosition = -0.8
task2_endEffectorOrientation = [0, 0, 0]
verbose = False

# TODO: Add your code here to start simulation
raise Exception("Method not invoked")

# Now plot some graphs
task2_figure_name = "task2_kinematics.png"
task2_savefig = False
# ... 





