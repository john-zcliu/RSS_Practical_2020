import subprocess
import math
import time
import sys
import os
import numpy as np
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



### You may change the code since here
pybulletConfigs = {
    "simulation": bullet_simulation,
    "pybullet_extra_data": pybullet_data,
    "gui": True,
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

task2_endEffectorName = "LARM_JOINT5"
task2_targetPosition = np.array([0.5, 0.3, 1.3])
task2_endEffectorOrientation = None
verbose = False

# Example code. Feel free to modify
pltTime, pltTargetError, pltFinalTargetError, pltTimeExtra, pltErrorExtra = \
sim.moveEFToTarget_without_PD(
    task2_endEffectorName, task2_targetPosition, 
    speed=0.0025, orientation=task2_endEffectorOrientation,
    threshold=2e-2, maxIter=1000, debug=False, verbose=True)


# Now plot some graphs
task2_figure_name = "task2_kinematics_without_pd.png"
task2_savefig = False
# TODO: please write some code to plot the graph
# The example code is provided in task1.

