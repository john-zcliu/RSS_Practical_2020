from scipy.spatial.transform import Rotation as npRotation
import matplotlib.pyplot as plt
import numpy as np
import math
import re
import time
import yaml

from Pybullet_Simulation_base import Simulation_base

# TODO: Rename class name after copying this file
class Simulation_template(Simulation_base):
    """A Bullet simulation involving Nextage robot"""

    def __init__(self, pybulletConfigs, robotConfigs):
        """Constructor
        Creates a simulation instance with Nextage robot.
        For the keyword arguments, please see in the Pybullet_Simulation_base.py
        """
        super().__init__(pybulletConfigs, robotConfigs)

    ########## Task 1: Dynamics ##########

    # Task 1.1 PD controller
    def calculateTorque(self, x_ref, x_real, dx_ref, dx_real, integral, kp, ki, kd):
        """ This method implements the closed-loop control \\
        Arguments: \\
            x_ref - the target position \\
            x_real - current position \\
            dx_ref - target velocity \\
            dx_real - current velocity \\
            integral - integral term (set to 0 for PD control) \\
            kp - proportional gain \\
            kd - derivetive gain \\
            ki - integral gain \\
        Returns: \\
            u(t) - the manipulation signal
        """
        # TODO: Add your code here
        pass

    # Task 1.2
    def moveJoint(self, joint, targetPosition, targetVelocity, verbose=False):
        """ This method moves a joint with your PD controller. \\
        Arguments: \\
            joint - the name of the joint \\
            targetPos - target joint position \\
            targetVel - target joint velocity 
        """
        def toy_tick(x_ref, x_real, dx_ref, dx_real, integral):
            # loads your PID gains
            jointController = self.jointControllers[joint]
            kp = self.ctrlConfig[jointController]['pid']['p']
            ki = self.ctrlConfig[jointController]['pid']['i']
            kd = self.ctrlConfig[jointController]['pid']['d']

            ### Start your code here: ###
            # Calculate the torque here
            torque = 0.0  # TODO: change me
            ### Your code should end by here ###

            pltTorque.append(torque)

            # send the manipulation signal to the joint
            self.p.setJointMotorControl2(
                bodyIndex=self.robot,
                jointIndex=self.jointIds[joint],
                controlMode=self.p.TORQUE_CONTROL,
                force=torque
            )
            # calculate the physics and update the world
            self.p.stepSimulation()
            time.sleep(self.dt)

        # disable joint velocity controller before apply a torque
        self.disableVelocityController(joint)

        pltTime, pltTarget, pltTorque, pltPosition, pltVelocity = [], [], [], [], []

        x_old = 0.0
        integral = 0.0

        duration = 3.0  # in seconds, feel free to change
        currentTime = 0.0
        sleep_time = (self.controlFrequency /
                      self.updateFrequency - 1) * self.dt
        while currentTime < duration:
            ### Start your code here: ###
            # sense the position/velocity of the joint here
            x_real = 0.0  # TODO: change me
            dx_real = 0.0  # TODO: change me
            integral = 0.0  # TODO: change me
            ### Your code should end by here ###

            if verbose:
                print(f"[Task1.2 calTorque] joint {joint} position: {x_real:.4f} " +
                      f"error {targetPosition-x_real:.4f} target {targetPosition}")

            toy_tick(targetPosition, x_real, targetVelocity, dx_real, integral)
            x_old = x_real

            pltTime.append(currentTime)
            pltTarget.append(targetPosition)
            pltPosition.append(x_real)
            pltVelocity.append(dx_real)

            time.sleep(sleep_time)
            currentTime += sleep_time
        return pltTime, pltTarget, pltTorque, pltPosition, pltVelocity

    ########## Task 2: Kinematics ##########
    # Task 2.1 Forward Kinematics
    # Task 2.1.1 Rotation axis of each joint
    jointRotationAxis = {
        'base_to_dummy': np.zeros(3),  # Virtual joint
        'base_to_waist': np.zeros(3),  # Fixed joint
        'CHEST_JOINT0': None,  # TODO: modify from here
        'HEAD_JOINT0': None,
        'HEAD_JOINT1': None,
        'LARM_JOINT0': None,
        'LARM_JOINT1': None,
        'LARM_JOINT2': None,
        'LARM_JOINT3': None,
        'LARM_JOINT4': None,
        'LARM_JOINT5': None,
        'RARM_JOINT0': None,
        'RARM_JOINT1': None,
        'RARM_JOINT2': None,
        'RARM_JOINT3': None,
        'RARM_JOINT4': None,
        'RARM_JOINT5': None  # modify ends here
    }

    # Task 2.1.1 Translation of each joint from its parent
    frameTranslationFromParent = {
        'base_to_dummy': np.zeros(3),  # Virtual joint
        'base_to_waist': np.zeros(3),  # Fixed joint
        'CHEST_JOINT0': None,  # TODO: modify from here
        'HEAD_JOINT0': None,
        'HEAD_JOINT1': None,
        'LARM_JOINT0': None,
        'LARM_JOINT1': None,
        'LARM_JOINT2': None,
        'LARM_JOINT3': None,
        'LARM_JOINT4': None,
        'LARM_JOINT5': None,
        'RARM_JOINT0': None,
        'RARM_JOINT1': None,
        'RARM_JOINT2': None,
        'RARM_JOINT3': None,
        'RARM_JOINT4': None,
        'RARM_JOINT5': None  # modify ends here
    }

    ###################################
    # Add your helper methods if needed

    # End of you helper methods
    ###################################

    # Task 2.1.1 Translation of each joint from its parent
    def getTransformationMatrices(self):
        """Forward Kinematics
        Calculate the homogeneous transformation matrix of all joints
        based on the joint positions.

        Return:
            A dictionary (key, value) = (jointName, transformation matrix)
        """
        # TODO: Add your code here
        pass

    # Task 2.1.2 Kinematic map
    def getJointLocation(self, jointName):
        """Homogeneous transformation of a joint
        Use the tranformation matrix to find the location of the joint
        in the world frame
        """
        # TODO: Add your code here
        pass

    # Task 2.2 Inverse Kinematics
    # Task 2.2.1 Jacobian matrix
    def jacobianMatrix(self):
        """Calculate the Jacobian Matrix for the Nextage Robot"""
        # TODO: Add your code here
        pass

    # Task 2.2.2 Inverse Kinematics
    def inverseKinematics(self, endEffector, targetPosition, orientation=None):
        """Your IK solver
        Arguments:
            endEffector: the jointName the end-effector
            targetPosition: final destination the the end-effector

        Keywork Arguments:
            orientation: the desired orientation of the end-effector 
                         together with its parent link

        Return:
            None, but executes tick() to run the simulation
        """
        # TODO: Add your code here
        pass

    def inverseKinematics_without_PD(self, endEffector, targetPosition, orientation=None):
        """Your IK solver
        Arguments:
            endEffector: the jointName the end-effector
            targetPosition: final destination the the end-effector

        Keywork Arguments:
            orientation: the desired orientation of the end-effector 
                         together with its parent link

        Return:
            None, but executes tick() to run the simulation
        """
        # TODO: Add your code here
        pass

    # Task 2.3 The Nextage Robot Simulation
    def tick(self):
        """Ticks one step of simulation. Your should modify this file as you progress"""
        for joint in self.joints:
            # skip dummy joints (world to base joint)
            jointController = self.jointControllers[joint]
            if jointController == 'SKIP_THIS_JOINT': continue

            # You should disable joint velocity controller before apply a torque  
            self.disableVelocityController(joint)

            # loads your PID gains 
            kp = self.ctrlConfig[jointController]['pid']['p']
            ki = self.ctrlConfig[jointController]['pid']['i']
            kd = self.ctrlConfig[jointController]['pid']['d']

            torque = 0.0  # TODO: change me (Dynamics)

            self.p.setJointMotorControl2(
                bodyIndex   = self.robot,
                jointIndex  = self.jointIds[joint],
                controlMode = self.p.TORQUE_CONTROL,
                force       = torque
            )

            # Gravity compensation
            # A naive gravitiy compensation is provided for you 
            # If you have embeded a better compensation, feel free to modify
            compensation = self.jointGravCompensation[joint]
            self.p.applyExternalForce(
                objectUniqueId = self.robot,
                linkIndex      = self.jointIds[joint],
                forceObj       = [0, 0, -compensation],
                posObj         = self.getLinkCoM(joint), #self.getLinkState(joint)[2],
                flags          = self.p.WORLD_FRAME
            )
            # Gravity compensation ends here

        self.p.stepSimulation()
        self.drawDebugLines()
        time.sleep(self.dt)


    ########## Task 3: "The final pick" ##########
    # Task 3.1 Pushing a cube
    def moveEndEffectorToPosition(self, endEffector, endEffectorTargetPos, 
            speed=0.05, orientation=[0,-math.pi/2,0], solverMaxIter=300, threshold=5e-3,  
            compensationRatio=None, debugLine=True, verbose=False):    
        """A template function for you, you are free to use anything else"""    
        # TODO: Add your code here
        pass

    # Task 3.2 Docking
    def dockingToPosition(self, leftTargetAngle, rightTargetAngle, angularSpeed=0.005, 
            threshold=1e-1, maxIter=300, verbose=False):
        """A template function for you, you are free to use anything else"""
        # TODO: Append your code here
        pass

