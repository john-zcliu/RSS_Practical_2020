from scipy.spatial.transform import Rotation as npRotation
import matplotlib.pyplot as plt
import numpy as np
import math
import re
import time
import yaml


class Simulation_base:
    """A Bullet simulation involving Nextage robot"""

    ########## Class initialiser ##########
    def __init__(self, pybulletConfigs, robotConfigs):
        """Creates a simulation instance with Nextage robot
        Keyword Arguments:
            pybulletConfigs (dict) = {
                "simulation"            : bullet_simulation -- pybullet package
                "pybullet_extra_data"   : pybullet_data     -- pybullet_data package
                "gui"                   : True  -- enables the gui visualizer, if False it will runs headless
                "panels"                : False -- show/hide the user interaction pyBullet panels
                "realTime"              : False -- use realtime simulation
                "controlFrequency"      : 1000  -- pybullet control updating frequency (in hertz)
                "updateFrequency"       : 250   -- pybullet debug view updating frequency (in hertz)
                "gravity"               : -9.81 -- gravity constant
                "gravityCompensation"   : 0.9   -- a naive way to compensate gravity
                "floor"                 : True  -- show floor or not
                "cameraSettings"        : None  -- initial camera settings
            }
            robotConfigs (dict) = {
                "robotPath"             : str           -- path_to_robot_urdf_file
                "robotPIDConfigs"       : str           -- path_to_robot_PID_configs_file
                "robotStartPos"         : [0, 0, 0.85]  -- starting position of the robot 
                "robotStartOrientation" : [0,0,0,1]     -- starting orientation of the robot
                "fixedBase"             : False         -- makes the base of the robot floating/fixed
                "colored"               : False         -- makes the robot coloured
            }
        """

        self.pybulletConfigs = pybulletConfigs
        self.robotConfigs = robotConfigs
        self.p = self.pybulletConfigs["simulation"]
        self.gui = pybulletConfigs["gui"]
        self.pybullet_data = self.pybulletConfigs["pybullet_extra_data"]
        self.controlFrequency = pybulletConfigs["controlFrequency"]
        self.updateFrequency = pybulletConfigs["updateFrequency"]
        self.dt = 1. / self.controlFrequency

        self.cameraPresets = {
            "cameraPreset1": (1.8, 122.0, -27.6, (-0.03, 0.03, 0.83)),
            "cameraPreset2": (1.2, 172.0, 4.0, (0.26, -0.21, 1.14)),
            "cameraPreset3": (1.4, 49.2, -6.4, (0.23, 0.48, 0.88)),
            "cameraPreset4": (1.2, 126.4, -12.8, (-0.12, -0.01, 0.99)),
            "cameraPreset5": (1.2, 90, -22.8, (-0.12, -0.01, 0.99))
        }

        ### Simulation setup
        # Instanciating bullet
        if self.gui:
            self.physicsClient = self.p.connect(self.p.GUI)
        else:
            self.physicsClient = self.p.connect(self.p.DIRECT)

        # GUI / visual configs
        if not self.pybulletConfigs["panels"]:
            self.p.configureDebugVisualizer(self.p.COV_ENABLE_GUI, 0)
            self.p.configureDebugVisualizer(
                self.p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            self.p.configureDebugVisualizer(
                self.p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            self.p.configureDebugVisualizer(
                self.p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        self.p.configureDebugVisualizer(self.p.COV_ENABLE_MOUSE_PICKING, 1)
        if "cameraPreset" in self.pybulletConfigs["cameraSettings"]:
            # try to resolve camera config as a string
            try:
                self.p.resetDebugVisualizerCamera(
                    *self.cameraPresets[self.pybulletConfigs["cameraSettings"]])
            except:
                self.p.resetDebugVisualizerCamera(
                    *self.cameraPresets["cameraPreset1"])
        else:
            # try to resolve camera config as a camera config
            try:
                self.p.resetDebugVisualizerCamera(
                    *self.pybulletConfigs["cameraSettings"])
            except:
                self.p.resetDebugVisualizerCamera(
                    *self.cameraPresets["cameraPreset1"])

        # Engine parameters
        self.p.setRealTimeSimulation(pybulletConfigs["realTime"])
        self.p.setGravity(0, 0, pybulletConfigs["gravity"])
        self.p.setPhysicsEngineParameter(fixedTimeStep=self.dt)

        # Gravity compensation ratio
        self.gravityCompensationRatio = pybulletConfigs["gravityCompensation"]

        ### Loading objects/robots
        # add directories
        self.p.setAdditionalSearchPath(self.pybullet_data.getDataPath())

        # Loading floor and/or plane ground
        if pybulletConfigs["floor"]:
            self.floor = self.p.loadURDF("plane.urdf")
        else:
            self.floor = None

        # Loading robot
        self.mass = None
        robotFixedBase = 1 if robotConfigs["fixedBase"] else 0
        self.robot = self.p.loadURDF(
            fileName=robotConfigs["robotPath"],
            basePosition=robotConfigs["robotStartPos"],
            baseOrientation=robotConfigs["robotStartOrientation"],
            useFixedBase=robotFixedBase,
            # flags         = (self.p.URDF_USE_SELF_COLLISION
            #                       + self.p.URDF_USE_INERTIA_FROM_FILE),
        )

        # loading robot PID configs
        self.ctrlConfig = yaml.load(
            open(self.robotConfigs["robotPIDConfigs"]), Loader=yaml.FullLoader)

        # Setting frictions parameters to default ones
        self.setFloorFrictions()

        # Initialize joints and frames
        self.frames = {}
        self.noJoints = 0
        self.joints = []
        self.jointIds = {}
        self.jointsInfos = {}
        self.jointControllers = {}
        self.jointControlType = {}
        self.jointTorques = {}
        self.jointMaxTorques = {}
        self.jointTargetPos = {}
        self.jointPositionOld = {}
        self.jointTargetVels = {}
        self.jointIntegrals = {}
        self.jointGravCompensation = {}

        self.robotLimbs = [
            ["LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2",
                "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5"],
            ["RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2",
                "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5"],
            ["HEAD_JOINT0", "HEAD_JOINT1"]
        ]

        self.colorPalettes = {
            "lightOrange": [1.0, 0.82, 0.12, 1.0],
            "darkOrange": [1.0, 0.6, 0.0, 1.0],
            "darkGrey": [0.43, 0.43, 0.43, 1.0],
            "lightGrey": [0.65, 0.65, 0.65, 1.0],
        }

        # Initiallizing joint infos
        n = 0
        for k in range(self.p.getNumJoints(self.robot)):
            jointInfo = self.p.getJointInfo(self.robot, k)
            jointName = jointInfo[1].decode('utf-8')

            if not jointName.endswith('_fixing') and not jointName.endswith('_passive'):
                if '_frame' in jointName:
                    self.frames[jointName] = k
                else:
                    self.joints.append(jointName)
                    self.jointIds[jointName] = n
                    n += 1

                    self.jointsInfos[jointName] = {
                        'type': jointInfo[2]
                    }
                    if jointInfo[8] < jointInfo[9]:
                        self.jointsInfos[jointName]['lowerLimit'] = jointInfo[8]
                        self.jointsInfos[jointName]['upperLimit'] = jointInfo[9]
                    else:
                        # default value
                        self.jointsInfos[jointName]['lowerLimit'] = -2.0
                        # default value
                        self.jointsInfos[jointName]['upperLimit'] = 2.0
                    self.jointsInfos[jointName]['restPos'] = self.getJointPos(
                        jointName)
                    # default value
                    self.jointsInfos[jointName]['jointRange'] = 2.0

                    if re.match('(base|BASE)', jointName):
                        self.jointControllers[jointName] = "SKIP_THIS_JOINT"
                    else:
                        self.jointControllers[jointName] = jointName + \
                            "_position_controller"
                    self.jointControlType[jointName] = "velocity"
                    self.jointTorques[jointName] = 0.0
                    self.jointTargetPos[jointName] = 0.0
                    self.jointPositionOld[jointName] = 0.0
                    self.jointTargetVels[jointName] = 0.0
                    self.jointIntegrals[jointName] = 0.0
                    self.jointGravCompensation[jointName] = 0.0

        self.noJoints = len(self.jointIds)

        # Initializing debug lines
        self.initialiseDebugLines()
        # Robot color scheme, modify as you wish
        self.robotColorPreset = {
            'base_to_waist': "darkGrey",
            'CHEST_JOINT0': "lightGrey",
            'HEAD_JOINT0': "darkOrange",
            'HEAD_JOINT1': "lightOrange",
            'LARM_JOINT0': "darkOrange",
            'LARM_JOINT1': "lightOrange",
            'LARM_JOINT2': "darkOrange",
            'LARM_JOINT3': "lightOrange",
            'LARM_JOINT4': "darkOrange",
            'LARM_JOINT5': "lightOrange",
            'RARM_JOINT0': "darkOrange",
            'RARM_JOINT1': "lightOrange",
            'RARM_JOINT2': "darkOrange",
            'RARM_JOINT3': "lightOrange",
            'RARM_JOINT4': "darkOrange",
            'RARM_JOINT5': "lightOrange"
        }
        # Apply colors if required
        if robotConfigs['colored']:
            for joint in self.robotColorPreset:
                self.p.resetVisualShapeData(
                    self.robot, self.jointIds[joint],
                    rgbaColor=self.colorPalettes[self.robotColorPreset[joint]])

        # Finishing up and show no of joints
        print('[Simulation] Found '+str(len(self.jointIds))+' DOFs')

    ########## Destructor ##########
    def __del__(self):
        self.p.disconnect()
        time.sleep(1)
        print(f'[Simulation] Leaving')

    ########## Setting up tools ##########
    def setFloorFrictions(self, lateral=1, spinning=-1, rolling=-1):
        """Sets the frictions with the plane object

        Keyword Arguments:
            lateral (float) -- lateral friction (default: {1.0})
            spinning (float) -- spinning friction (default: {-1.0})
            rolling (float) -- rolling friction (default: {-1.0})
        """
        if self.floor is not None:
            self.p.changeDynamics(self.floor, -1, lateralFriction=lateral,
                                  spinningFriction=spinning, rollingFriction=rolling)

    def resetJointIntegrals(self):
        """Reset the integral value of every joint"""
        for joint in self.joints:
            self.jointIntegrals[joint] = 0.0

    def debugCameralookAt(self, target):
        """Make the debug camera loot at a point

        Arguments:
            target (tuple) -- target as (x,y,z) tuple
        """
        if self.gui:
            params = self.p.getDebugVisualizerCamera()
            self.p.resetDebugVisualizerCamera(
                params[10], params[8], params[9], target)

    def changeLinkColor(self, jointName, color):
        """Change a link's color and opacity

        Arguments:
            jointName (str) -- the joint name
            color [4 float] -- (r,g,b,a) for RGB and opacity from 0 to 1
        """
        self.p.changeVisualShape(
            self.robot, self.jointIds[jointName], rgbaColor=color)

    def resetAllLinkColor(self, color=[1, 1, 1, 1]):
        """Reset all link's color

        Keyword Arguments:
            color [4 floats] -- [1,1,1,1] white color by default
        """
        for i in range(self.noJoints):
            self.p.changeVisualShape(self.robot, i, rgbaColor=[1, 1, 1, 1])

    def initialiseDebugLines(self):
        """Initialise debug lines"""
        self.lines = []
        self.currentLine = 0
        self.lastLinesDraw = 0
        self.lineColors = [[1, 0, 0], [0, 1, 0], [
            0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1]]

    def initGravCompensation(self):
        """Initialise the gravity compensation"""
        for link in self.joints:
            self.jointGravCompensation[link] = 0.0

    def addDebugPosition(self, position, color=None, duration=30):
        """Adds a debug position to be drawn as a line

        Arguments:
            position (tuple) -- (x,y,z)

        Keyword Arguments:
            color (tuple) -- (r,g,b) (0->1) (default: {None})
            duration (float) -- line duration on screen before disapearing (default: {30})
        """
        if color is None:
            color = self.lineColors[self.currentLine]

        if self.currentLine >= len(self.lines):
            self.lines.append({})

        self.lines[self.currentLine]['update'] = True
        self.lines[self.currentLine]['to'] = position
        self.lines[self.currentLine]['color'] = color
        self.lines[self.currentLine]['duration'] = duration

        self.currentLine = (self.currentLine + 1) % len(self.lineColors)

    def drawDebugLines(self):
        """Draw debug lines"""
        self.currentLine = 0
        if time.time() - self.lastLinesDraw > 0.05:
            for line in self.lines:
                if 'from' in line:
                    if line['update'] == True:
                        self.p.addUserDebugLine(
                            line['from'], line['to'], line['color'], 2, line['duration'])
                        line['update'] = False
                    else:
                        del line['from']
                line['from'] = line['to']

            self.lastLinesDraw = time.time()

    def setBallPos(self, ball, position):
        """Sets the ball position on the field

        Arguments:
            ball (int) -- unique id of the ball
            position (tuple of 3 floats) -- position of the ball
        """
        if ball is not None:
            # Putting the ball on the ground at given position
            x, y, z = position
            self.p.resetBasePositionAndOrientation(
                ball, [x, y, z], self.p.getQuaternionFromEuler([0, 0, 0]))
            # Hover the ball if not using fixed base under gravity
            self.p.changeDynamics(ball, 0, linearDamping=0, angularDamping=0.1)

    def getCameraStatus(self):
        """Returns: tuple -- camera status"""
        return self.p.getDebugVisualizerCamera()

    def getCamPosProcessed(self):
        """Returns: tuple -- (camera position, camera focus position)"""
        # distance, yaw, pitch, tarPos, clientid(optional if you have multiple clients)
        config = self.getCameraStatus()
        config = config[10], config[8], config[9], config[11]
        camPos = list(map(lambda x: float(f'{x:.2f}'), config[:3]))
        camTar = list(map(lambda x: float(f'{x:.2f}'), config[3]))
        return (*camPos, tuple(camTar))

    def closeClient(self):
        """Disconnect the Pybullet Simulator"""
        self.p.disconnect()

    ########## Robot Configs & Status ##########

    def getRobotPose(self, robot=None):
        """Gets the robot (origin) position

        Returns:
            (tuple(3), tuple(3)) -- (x,y,z), (roll, pitch, yaw)
        """
        if robot is None:
            robot = self.robot
        pose = self.p.getBasePositionAndOrientation(robot)
        return (pose[0], self.p.getEulerFromQuaternion(pose[1]))

    def setRobotPose(self, pos, orn, robot=None):
        """Sets the robot (origin) pose

        Arguments:
            pos {tuple} -- (x,y,z) position
            orn {tuple} -- (x,y,z,w) quaternions
        """
        if robot is None:
            robot = self.robot
        self.p.resetBasePositionAndOrientation(robot, pos, orn)

    def autoCollisions(self):
        """Returns the total amount of N in autocollisions (not with ground)

        Returns:
            float -- Newtons of collisions not with ground
        """
        total = 0
        for k in range(1, self.noJoints):
            contacts = self.p.getContactPoints(bodyA=k)
            for contact in contacts:
                if contact[2] != self.floor:
                    total += contact[9]
        return total

    def disableRobotCollisions(self):
        """Disable self collision of the robot"""
        for i in range(-1, self.noJoints):
            for j in range(-1, self.noJoints):
                self.p.setCollisionFilterPair(self.robot, self.robot, i, j, 0)

    def disableRobotBallCollision(self, ball):
        """Disable collision between the robot and a ball
        
        Keyword Arguments:
            ball (int) -- the unique id of the ball
        """
        for i in range(-1, self.noJoints):
            self.p.setCollisionFilterPair(self.robot, ball, i, -1, 0)

    def contactPoints(self):
        """Gets all contact points and forces
        
        Returns:
            list -- list of entries (link_name, position in m, force in N)
        """
        result = []
        contacts = self.p.getContactPoints(bodyA=self.floor, bodyB=self.robot)
        for contact in contacts:
            link_index = contact[4]
            if link_index >= 0:
                link_name = (self.p.getJointInfo(
                    self.robot, link_index)[12]).decode()
            else:
                link_name = 'base'
            result.append((link_name, contact[6], contact[9]))

        return result

    ########## Joint Status and Controls ##########

    def showJoints(self):
        """Display the details of joionts of the robot"""
        print('Joints:')
        for i, j in enumerate(self.joints):
            print(f'joint {i}: {j}')

    def getJointInfo(self, jointName):
        """Get informations about a joint
        
        Return: list -- 
            No    Parameter         Type    Description  
            [0]   jointIndex        int     the same joint index as the input parameter  
            [1]   jointName         string  the name of the joint, as specified in the URDF (or SDF etc) file
            [2]   jointType         int     type of the joint, this also implies the number of position and velocity variables. JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED. See the section on Base, Joint and Links for more details.
            [3]   qIndex            int     the first position index in the positional state variables for this body
            [4]   uIndex            int     the first velocity index in the velocity state variables for this body
            [5]   flags             int     reserved
            [6]   jointDamping      float   the joint damping value, as specified in the URDF file
            [7]   jointFriction     float   the joint friction value, as specified in the URDF file
            [8]   jointLowerLimit   float   Positional lower limit for slider and revolute (hinge) joints.
            [9]   jointUpperLimit   float   Positional upper limit for slider and revolute joints. Values ignored in case upper limit <lower limit.
            [10]  jointMaxForce     float   Maximum force specified in URDF (possibly other file formats) Note that this value is not automatically used. You can use maxForce in 'setJointMotorControl2'.
            [11]  jointMaxVelocity  float   Maximum velocity specified in URDF. Note that the maximum velocity is not used in actual motor control commands at the moment.
            [12]  linkName          string  the name of the link, as specified in the URDF (or SDF etc.) file
            [13]  jointAxis         vec3    joint axis in local frame (ignored for JOINT_FIXED)
            [14]  parentFramePos    vec3    joint position in parent frame
            [15]  parentFrameOrn    vec4    joint orientation in parent frame (quaternion x,y,z,w)
            [16]  parentIndex       int     parent link index, -1 for base
        """
        return self.p.getJointInfo(self.robot, self.jointIds[jointName])

    def getJointQIndex(self, jointName):
        """Return the index by given a joint's name"""
        return self.getJointInfo(jointName)[3]

    def getJointLowerLimit(self, jointName):
        """Return the lower limit by given a joint's name"""
        return self.getJointInfo(jointName)[8]

    def getJointUpperLimit(self, jointName):
        """Return the upper limit by given a joint's name"""
        return self.getJointInfo(jointName)[9]

    def getJointRange(self, jointName):
        """Return the range by given a joint's name"""
        return self.jointsInfos[jointName]['jointRange']  # TODO: find out this parameter

    def getJointRestPos(self, jointName):
        """Return the rest position by given a joint's name"""
        return self.jointsInfos[jointName]['restPos']

    def disableVelocityController(self, jointName):
        """Disable the velocity controller of a joint by given a joint's name
        
        Keyword Arguments:
            jointName (str) -- joint name
            
        Returns:
            True -- if succeed   
            False -- else
        """
        if jointName in self.joints:
            self.p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=self.jointIds[jointName],
                controlMode=self.p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0
            )
            return True
        else:
            return False

    def setJoints(self, jointPoses):
        """Set some joints to a given position (for debugging uses)
        Arguments:
            jointPoses { jointName: targetPos }
            e.g. { 'LARM_JOINT0': 0.0, 'RARM_JOINT5': math.pi }
        """
        for jointName in jointPoses:
            self.p.resetJointState(
                self.robot, self.jointIds[jointName], jointPoses[jointName])

    def resetRobot(self):
        """Restore the robot to the default settings (robot position and joints, other internal variables)"""
        # Reset the base pose
        self.setRobotPose(
            self.robotConfigs["robotStartPos"], self.robotConfigs["robotStartOrientation"])
        # Reset the joints to 0
        for joint in self.jointIds:
            self.p.resetJointState(self.robot, joint, 0)

        self.t = 0
        self.start = time.time()
        self.lines = []
        self.currentLine = 0
        self.lastLinesDraw = 0

    def getJointState(self, jointName):
        """Get real time status about a joint
        Return:
            No  Parameter               Type             Description
            [0] jointPosition           float            The position value of this joint.
            [1] jointVelocity           float            The velocity value of this joint.
            [2] jointReactionForces     list of 6 floats These are the joint reaction forces, if a torque sensor is enabled for this joint it is [Fx, Fy, Fz, Mx, My, Mz]. Without torque sensor, it is [0,0,0,0,0,0].
            [3] appliedJointMotorTorque float            This is the motor torque applied during the last stepSimulation. Note that this only applies in VELOCITY_CONTROL and POSITION_CONTROL. If you use TORQUE_CONTROL then the applied joint motor torque is exactly what you provide, so there is no need to report it separately.
        """
        return self.p.getJointState(self.robot, self.jointIds[jointName])

    def getJointPos(self, jointName, precision=19):
        """Get the revolute position of a joint
        Return:
            float -- the angle of the joint in radians
        """
        pos = self.getJointState(jointName)[0]
        return float(f"{pos:.{precision}f}")

    def getJointPoses(self, jointNames, precision=19):
        """Get the real positions of a list of joints
        Return:
            {jointName: float} -- dictionary of joint name and the joint position
        """
        poses = list(map(lambda x: self.getJointPos(x, precision), jointNames))
        result = {}
        for i in range(self.noJoints):
            result[self.joints[i]] = poses[i]
        return result

    def getJointVel(self, jointName, precision=19):
        """Get the real velocity of a joint
        Return:
            float -- the velocity of the joint
        """
        vel = self.getJointState(jointName)[1]
        return float(f"{vel:.{precision}f}")

    def getJointVelArr(self, jointNames, precision=19):
        """Get the real velocity of a list of joints
        Return:
            {jointName: float} -- dictionary of joint name and the joint velocity
        """
        vels = list(map(lambda x: self.getJointVel(x, precision), jointNames))
        result = {}
        for i in range(self.noJoints):
            result[self.joints[i]] = vels[i]
        return result

    ########## Robot dynamics and kinematics ##########

    def getLinkState(self, jointName):
        """
        Return:
            No  Parameter                       Shape                   Descriptions
            [0] linkWorldPosition               vec3, list of 3 floats  Cartesian position of center of mass
            [1] linkWorldOrientation            vec4, list of 4 floats  Cartesian orientation of center of mass, in quaternion [x,y,z,w]
            [2] localInertialFramePosition      vec3, list of 3 floats  local position offset of inertial frame (center of mass) expressed in the URDF link frame
            [3] localInertialFrameOrientation   vec4, list of 4 floats  local orientation (quaternion [x,y,z,w]) offset of the inertial frame expressed in URDF link frame.
            [4] worldLinkFramePosition          vec3, list of 3 floats  world position of the URDF link frame
            [5] worldLinkFrameOrientation       vec4, list of 4 floats  world orientation of the URDF link frame
            [6] worldLinkLinearVelocity         vec3, list of 3 floats  Cartesian world velocity. Only returned if computeLinkVelocity non-zero.
            [7] worldLinkAngularVelocity        vec3, list of 3 floats  Cartesian world velocity. Only returned if computeLinkVelocity non-zero.
        """
        return self.p.getLinkState(self.robot, self.jointIds[jointName])

    def getLinkDynamicsInfo(self, linkName):
        """
        Return:
            No  Parameter               Shape                   Description
            [0] mass                    double                  mass in kg
            [1] lateral_friction        double                  friction coefficient
            [2] local inertia diagonal  vec3, list of 3 floats  local inertia diagonal. Note that links and base are centered around the center of mass and aligned with the principal axes of inertia.
            [3] local inertial pos      vec3                    position of inertial frame in local coordinates of the joint frame
            [4] local inertial orn      vec4                    orientation of inertial frame in local coordinates of joint frame
            [5] restitution             double                  coefficient of restitution
            [6] rolling friction        double                  rolling friction coefficient orthogonal to contact normal
            [7] spinning friction       double                  spinning friction coefficient around contact normal
            [8] contact damping         double                  -1 if not available. damping of contact constraints.
            [9] contact stiffness       double                  -1 if not available. stiffness of contact constraints.
            [10]body type               int                     1=rigid body, 2 = multi body, 3 = soft body
            [11]collision margin        double                  advanced/internal/unsupported info. collision margin of the collision shape. collision margins depend on the shape type, it is not consistent.
        """
        return self.p.getDynamicsInfo(bodyUniqueId=self.robot, linkIndex=self.jointIds[linkName])

    def getLinkMass(self, linkName):
        """Keyword Arguments:
            linkName (str) -- the name of the joint
        Return: the mass of the given link
        """
        return self.getLinkDynamicsInfo(linkName)[0]

    def getLinkMassRecursive(self, linkName):
        """Calculate and return the mass of the given link and its children
        Keyword Arguments:
            linkName (str) -- the name of the link
        Returns:
            float -- the robot mass (kg)
        """
        mass = 0.0
        for limbs in self.robotLimbs:
            if linkName in limbs:
                index = limbs.index(linkName)
                for index in range(index, len(limbs)):
                    mass += self.getLinkMass(limbs[index])

        return mass

    def getRobotMass(self):
        """Returns the robot mass
        Returns:
            float -- the robot mass (kg)
        """
        if self.mass is None:
            k = -1
            self.mass = 0
            while True:
                if k == -1 or self.p.getLinkState(self.robot, k) is not None:
                    d = self.p.getDynamicsInfo(self.robot, k)
                    self.mass += d[0]
                else:
                    break
                k += 1

        return self.mass

    def getCenterOfMassPosition(self):
        """Returns center of mass of the robot
        Returns:
            pos -- (x, y, z) robot center of mass
        """
        k = -1
        mass = 0
        com = np.array([0., 0., 0.])
        while True:
            if k == -1:
                pos, _ = self.p.getBasePositionAndOrientation(self.robot)
            else:
                res = self.p.getLinkState(self.robot, k)
                if res is None:
                    break
                pos = res[0]

            d = self.p.getDynamicsInfo(self.robot, k)
            m = d[0]
            com += np.array(pos) * m
            mass += m

            k += 1

        return com / mass

    # Vector calculations
    def getVectorLength(self, vector):
        """Return the length of a given vector"""
        return math.sqrt(np.dot(vector, vector))

    def getDistance(self, vectorA, vectorB):
        """Get the distance between two vectors"""
        vectorA = np.array(vectorA)
        vectorB = np.array(vectorB)
        return self.getVectorLength(vectorA - vectorB)

    def normaliseVector(self, vector, magnitude=1):
        """Normalise a vector (to unit vector by default
        Arguments:
            vector (np.array) -- the given vector to be normalised
        Keyword Arguments:
            magnitude (float) -- normalise length
        Returns:
            np.array -- normalised vector
        """
        vector = np.array(vector)
        return (vector / self.getVectorLength(vector)) * magnitude

    def calIterToTarget(self, fromVector, toVector, speed):
        """Calculate the steps needed to reach the target
        Arguments:
            vector (np.array) -- the given vector to be normalised
        Keyword Arguments:
            fromVector (np.array) -- starting position 
            toVector (np.array) -- target position
            speed (float) -- target speed
        Returns:
            int -- ceil of the steps
        """
        return math.ceil(self.getVectorLength(toVector - fromVector) / speed * self.updateFrequency)

    # Matrices calculations
    def getFrame(self, linkName):
        """Gets the frame of a link
        Arguments:
            linkName {str} -- link name
        Returns:
            tuple -- (pos, orn), where pos is (x, y, z) and orn is quaternions (x, y, z, w)
        """
        jointState = self.getLinkState(linkName)
        return (self.getLinkCoM(linkName), np.array(jointState[1]))

    def getLinkCoM(self, linkName):
        """Returns the CoM of a link"""
        return np.array(self.getLinkState(linkName)[0])

    def getLinkFramePos(self, jointName):
        """Get the position of the frame of a link. NB: linkId == JointId
        Return:
            vec3 -- (x, y, z) list of 3 floats
        """
        return np.array(self.getLinkState(jointName)[4])

    def getLinkFrameOrientaton(self, jointName):
        """Get the orientation of the frame of a link.  NB: linkId == JointId
        Return:
            vec4 -- (x, y, z, w) list of 4 floats
        """
        return np.array(self.getLinkState(jointName)[5])

    def frameToWorldMatrix(self, frame):
        """Gets the given frame to world matrix transformation. can be a frame name
        from URDF/SDF or "origin" for the part origin
        Arguments:
            frame {str} -- frame name
        Returns:
            np.matrix -- a 4x4 matrix
        """

        if frame == 'origin':
            frameToWorldPose = self.p.getBasePositionAndOrientation(self.robot)
        else:
            frameToWorldPose = self.p.getLinkState(
                self.robot, self.frames[frame])

        return self.poseToMatrix(frameToWorldPose)

    def matrixTransformation(self, frameA, frameB):
        """Transformation matrix from A to B
        Arguments:
            frameA np.matrix -- A 4x4 matrix
            frameB np.matrix -- A 4x4 matrix
        Returns:
            np.matrix -- A 4x4 matrix
        """
        return np.linalg.inv(frameB) * frameA

    def frameTransformation(self, frameA, frameB):
        """Transformation matrix from A to B
        Arguments:
            frameA {str} -- frame A name
            frameB {str} -- frame B name
        Returns:
            np.matrix -- A 4x4 matrix
        """
        AtoWorld = self.frameToWorldMatrix(frameA)
        BtoWorld = self.frameToWorldMatrix(frameB)

        return self.matrixTransformation(AtoWorld, BtoWorld)

    def getRotationMatrix(self, quaternion):
        """Return the rotation matrix from a given quaternion"""
        quaternion = npRotation.from_quat(quaternion)
        return quaternion.as_matrix()

    def poseToMatrix(self, pose):
        """Converts a pyBullet pose to a transformation matrix"""
        translation = pose[0]
        quaternion = pose[1]

        rotation = self.getRotationMatrix(quaternion)
        m = np.identity(4)
        m[0:3, 0:3] = rotation
        m.T[3, 0:3] = translation
        return np.matrix(m)

    def matrixToPose(self, matrix):
        """Converts a transformation matrix to a pyBullet pose"""
        m = np.array(matrix)
        translation = list(m.T[3, 0:3])
        quaternion = npRotation.from_matrix(m[0:3, 0:3])

        return translation, quaternion.as_quat()

    ########## PD controller and IK solver ##########
    ##### These functionalities should be provided by the users, good luck!
