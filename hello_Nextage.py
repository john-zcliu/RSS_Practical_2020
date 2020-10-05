import subprocess
import math
import time
import sys
import os
import numpy as np
import pybullet as bullet_simulation
import pybullet_data

# setup paths and load the core
abs_path = os.path.dirname(os.path.realpath(__file__))
root_path = abs_path
core_path = root_path + '/core'
sys.path.append(core_path)
from Pybullet_Simulation_template import Simulation_template

pybulletConfigs = {
    "simulation"          : bullet_simulation,
    "pybullet_extra_data" : pybullet_data,
    "gui"                 : True,   # Ture | False
    "panels"              : False,  # Ture | False
    "realTime"            : False,  # Ture | False
    "controlFrequency"    : 1000,   # Recommand 1000 Hz
    "updateFrequency"     : 250,    # Recommand 250 Hz
    "gravity"             : -9.81,  # Gravity constant
    "gravityCompensation" : 1.,     # Float, 0.0 to 1.0 inclusive
    "floor"               : True,   # Ture | False
    "cameraSettings"      : 'cameraPreset1' # cameraPreset{1..3}, 
}
robotConfigs = {
    "robotPath": core_path + "/nextagea_description/urdf/NextageaOpen.urdf",
    "robotPIDConfigs": core_path + "/PD_gains_template.yaml",
    "robotStartPos"         : [0, 0, 0.85], # (x, y, z)
    "robotStartOrientation" : [0, 0, 0, 1], # (x, y, z, w)
    "fixedBase"             : False,        # Ture | False
    "colored"               : True          # Ture | False
}

sim = Simulation_template(pybulletConfigs, robotConfigs)

try:
    time.sleep(float(sys.argv[1]))
except:
    time.sleep(10)

