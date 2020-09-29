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
taskId = 1

try:
    if sys.argv[1] == 'nogui':
        gui = False
    else:
        gui = True
except:
    gui = True

global task1_jointName, task1_targetPosition, task1_targetVelocity



### You may want to change the code from here 
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
    "robotPIDConfigs": core_path + "/nextagea_pybullet_control.yaml",
    "robotStartPos": [0, 0, 0.85],
    "robotStartOrientation": [0, 0, 0, 1],
    "fixedBase": False,
    "colored": False
}
sim = Simulation(pybulletConfigs, robotConfigs)

task1_jointName = "LARM_JOINT2"
task1_targetPosition = -0.8
task1_targetVelocity = 0.0
verbose = False
task1_figure_name = "Task1_DP_response.png"
task1_savefig = False
### to here




pltTime, pltTarget, pltTorque, pltPosition, pltVelocity = \
    sim.moveJoint_solution(
        task1_jointName, task1_targetPosition, task1_targetVelocity, verbose)

# modify the code in below if needed
fig = plt.figure(figsize=(6, 8))

plt.subplot(311)
plt.plot(pltTime, pltPosition, color='blue')
plt.plot(pltTime, pltTarget, color='magenta')
plt.ylabel("Theta rads")

plt.subplot(312)
plt.plot(pltTime, pltPosition, color='blue')
plt.plot(pltTime, pltVelocity, color='lightblue')
plt.ylabel("Velocity rads/s")

plt.subplot(313)
plt.plot(pltTime, pltTorque, color='orange')
plt.xlabel("Time s")
plt.ylabel("Torque N")

plt.suptitle("Task1.2 Response of the controller", size=16)
plt.tight_layout()
plt.subplots_adjust(left=0.15)

if task1_savefig: 
    fig.savefig(task1_figure_name)
plt.show()
