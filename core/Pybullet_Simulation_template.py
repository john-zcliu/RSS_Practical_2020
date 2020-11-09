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
            # Calculate the torque with the above method you've made
            torque = 0.0
            ### To here ###

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

        targetPosition, targetVelocity = float(targetPosition), float(targetVelocity)

        # disable joint velocity controller before apply a torque
        self.disableVelocityController(joint)
        # logging for the graph
        pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity = [], [], [], [], [], []

        ### TODO: implement your code from here
        

        return pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity



    ########## Task 2: Kinematics ##########
    # Task 2.1 Forward Kinematics
    # Task 2.1.1 Rotation axis of each joint
    jointRotationAxis = {
        'base_to_dummy': np.zeros(3),  # Virtual joint
        'base_to_waist': np.zeros(3),  # Fixed joint
        'CHEST_JOINT0' : None,  # TODO: modify from here
        'HEAD_JOINT0'  : None,
        'HEAD_JOINT1'  : None,
        'LARM_JOINT0'  : None,
        'LARM_JOINT1'  : None,
        'LARM_JOINT2'  : None,
        'LARM_JOINT3'  : None,
        'LARM_JOINT4'  : None,
        'LARM_JOINT5'  : None,
        'RARM_JOINT0'  : None,
        'RARM_JOINT1'  : None,
        'RARM_JOINT2'  : None,
        'RARM_JOINT3'  : None,
        'RARM_JOINT4'  : None,
        'RARM_JOINT5'  : None   # To here
    }

    # Task 2.1.1 Translation of each joint from its parent
    frameTranslationFromParent = {
        'base_to_dummy': np.zeros(3),  # Virtual joint
        'base_to_waist': np.zeros(3),  # Fixed joint
        'CHEST_JOINT0' : None,  # TODO: modify from here
        'HEAD_JOINT0'  : None,
        'HEAD_JOINT1'  : None,
        'LARM_JOINT0'  : None,
        'LARM_JOINT1'  : None,
        'LARM_JOINT2'  : None,
        'LARM_JOINT3'  : None,
        'LARM_JOINT4'  : None,
        'LARM_JOINT5'  : None,
        'RARM_JOINT0'  : None,
        'RARM_JOINT1'  : None,
        'RARM_JOINT2'  : None,
        'RARM_JOINT3'  : None,
        'RARM_JOINT4'  : None,
        'RARM_JOINT5'  : None   # To here
    }

    # Optional helper function skeleton - Not assessed
    def getJointRotationalMatrix(self, jointName=None):
        '''(optional) Calculate the rotational matrices for a joint 
        Returns: A rotational matrix for a joint
        '''
        pass

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
    def getJointLocationAndOrientation(self, jointName):
        """Homogeneous transformation of a joint 
        Use the tranformation matrix to find the pose (translation, orientation)
        of the joint in the world frame \\

        Argument: \\
            jointName: The name of the end-effector \\
        
        Returns: \\
            (Translation, orientation) = (vector of 3 float numbers, quaternion)            
        """
        # TODO: Add your code here
        pass

    # Helpful method extending from 2.1.2, you might find this useful for later
    def getJointPosition(self, jointName):
        """Get the position of a joint in the world frame, leave this unchanged please."""
        return self.getJointLocationAndOrientation(jointName)[0]

    # Helpful method extending from 2.1.2, you might find this useful for later
    def getJointOrientation(self, jointName):
        """Get the orientation of a joint in the world frame, leave this unchanged please."""
        return self.getJointLocationAndOrientation(jointName)[1]

    # Task 2.2 Inverse Kinematics
    # Task 2.2.1 Jacobian matrix
    def jacobianMatrix(self, endEffector):
        """Calculate the Jacobian Matrix for the Nextage Robot"""
        # TODO: Add your code here
        pass
    
    # Task 2.2.2 Inverse Kinematics
    def inverseKinematics(self, endEffector, targetPosition, orientation=None):
        """Your IK solver \\
        Arguments: \\
            endEffector: the jointName the end-effector \\
            targetPosition: final destination the the end-effector \\

        Keywork Arguments: \\
            orientation: the desired orientation of the end-effector 
                         together with its parent link \\

        Return: \\
            None, but executes tick() to run the simulation
        """
        # TODO: Add your code here
        pass
    
    def moveEFToTarget_without_PD(self, endEffector, targetPosition, speed=0.0025, orientation=None,
                                  threshold=2e-2, maxIter=1000, debug=False, verbose=False):
        """Your IK solver \\
        Arguments: \\
            endEffector: the jointName the end-effector \\
            targetPosition: final destination the the end-effector \\

        Keywork Arguments: \\
            speed: how fast the end-effector should move (m/s) \\
            orientation: the desired orientation of the end-effector 
                        together with its parent link \\
            threshold: the maximum allowed error to the target position \\
            maxIter: the maximum number of iteration allowed \\  
            debug: optional \\
            verbose: optional \\

        Return: None,
        """
        # No need for compensating gravity
        
        # logging for the graph:
        pltTime, pltTargetError, pltFinalTargetError = [], [], []

        ### TODO: implement your code here
        # Hint: without PD, your trajectory should be perfect, so no need for extra converging steps
        # Remember to directly update joint positions from this method
        

        return pltTime, pltTargetError, pltFinalTargetError

    def moveEFToTarget(self, endEffector, targetPosition, speed=0.0025, orientation=None,
                       threshold=2e-2, maxIter=1000, debug=False, verbose=False):
        """Your IK solver \\
        Arguments: \\
            endEffector: the jointName the end-effector \\
            targetPosition: final destination the the end-effector \\

        Keywork Arguments: \\
            speed: how fast the end-effector should move (m/s) \\
            orientation: the desired orientation of the end-effector 
                        together with its parent link \\
            threshold: the maximum allowed error to the target position \\
            maxIter: the maximum number of iteration allowed \\  
            debug: optional \\
            verbose: optional \\

        Return: \\
            None, but executes tick() to run the simulation
        """
        # provided gravity compensation
        compensationRatio = self.gravityCompensationRatio
        self.initGravCompensation()
        for limbs in self.robotLimbs:
            if endEffector in limbs:
                for link in limbs:
                    compensation = self.pybulletConfigs['gravity'] * self.getLinkMass(link) * compensationRatio
                    self.jointGravCompensation[link] = compensation
                    if verbose:
                        print(f'[Gravity compensation] {link} with {compensation} N')

        # logging for the graph:
        pltTime, pltTargetError, pltFinalTargetError = [], [], []
        pltTimeExtra, pltErrorExtra = [], [] # extra converging steps, if needed

        ### TODO: implement your code here
        # Remember to use self.tick() for PD


        return pltTime, pltTargetError, pltFinalTargetError, pltTimeExtra, pltErrorExtra

    # Task 2.3 The Nextage Robot Simulation

    def tick(self):
        """Ticks one step of simulation. Your should modify this file as you progress"""
        for joint in self.joints:
            # skip dummy joints (world to base joint)
            jointController = self.jointControllers[joint]
            if jointController == 'SKIP_THIS_JOINT':
                continue

            # disable joint velocity controller before apply a torque
            self.disableVelocityController(joint)

            # loads your PID gains
            kp = self.ctrlConfig[jointController]['pid']['p']
            ki = self.ctrlConfig[jointController]['pid']['i']
            kd = self.ctrlConfig[jointController]['pid']['d']

            ### Implement your code from here ###
            # TODO: PD controller stuff
            torque = 0.0 # TODO: fix me
            
            ### To here ###

            self.p.setJointMotorControl2(
                bodyIndex=self.robot,
                jointIndex=self.jointIds[joint],
                controlMode=self.p.TORQUE_CONTROL,
                force=torque
            )

            # Gravity compensation
            # A naive gravitiy compensation is provided for you
            # If you have embeded a better compensation, feel free to modify
            compensation = self.jointGravCompensation[joint]
            self.p.applyExternalForce(
                objectUniqueId=self.robot,
                linkIndex=self.jointIds[joint],
                forceObj=[0, 0, -compensation],
                posObj=self.getLinkCoM(joint),  
                flags=self.p.WORLD_FRAME
            )
            # Gravity compensation ends here

        self.p.stepSimulation()
        self.drawDebugLines()
        time.sleep(self.dt)



    ########## Task 3: "The final pick" ##########
    # Task 3.1 Pushing a cube
    def moveEndEffectorToPosition(self, endEffector, endEffectorTargetPos,
                                  speed=0.05, orientation=[0, -math.pi/2, 0], 
                                  solverMaxIter=1000, threshold=15e-3,
                                  debugLine=True, verbose=False):
        """A template function for you, you are free to change anything including the parameters"""
        # TODO: Add your code here
        pass

    # Task 3.2 Docking
    def dockingToPosition(self, targetPose, speed=0.05, threshold=15e-3, maxIter=1000, 
                          debugLine=True, verbose=False):
        """A template function for you, you are free to change anything including the parameters"""
        # TODO: Append your code here
        pass
