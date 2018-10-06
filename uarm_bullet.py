import pybullet
import pybullet_data
import time
import numpy as np
import cv2


class Simulator:
    def __init__(self, mode):
        pybullet.connect(mode)
        pybullet.resetSimulation()
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)
        # pybullet.setGravity(0, 0, -0)
        # pybullet.setTimeStep(0.001)
        pybullet.setRealTimeSimulation(1)

        self.plane = pybullet.loadURDF("plane.urdf")

        self.link = {"base": 0, "base_rot": 1, "link_1": 2, "link_2": 3, "link_3": 4, "link_end": 5}
        self.joint = {"base_to_base_rot": 0, "base_rot_to_link_1": 1, "link_1_to_link_2": 2, "link_2_to_link_3": 3, "link_3_to_link_end": 4}

        self.picknow = False

        self.createRobot()
        self.createObject(6)

        j1 = 0.5
        j2 = 0.5
        j3 = -0.5
        j4 = 0
        j5 = 0
        self.MATH_L1 = 0.032 + 0.07345
        self.MATH_L2 = 0.02117
        self.MATH_LOWER_ARM = 0.14825
        self.MATH_UPPER_ARM = 0.16
        self.MATH_FRONT_HEADER = 0.036
        self.MATH_ENDEFFECTOR = 0.055

        # 順運動学確認
        # self.cid = pybullet.createConstraint(self.obj_body[0], -1, -1, -1, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0.1, 0])
        # self.cid = pybullet.createConstraint(self.obj_body[1], -1, -1, -1, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, MATH_L2, MATH_L1])
        # self.cid = pybullet.createConstraint(self.obj_body[2], -1, -1, -1, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, MATH_L2 + np.cos(j2) * MATH_LOWER_ARM, MATH_L1 + np.sin(j2) * MATH_LOWER_ARM])
        # self.cid = pybullet.createConstraint(self.obj_body[3], -1, -1, -1, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, MATH_L2 + np.cos(j2) * MATH_LOWER_ARM + np.cos(j2 + j3) * MATH_UPPER_ARM, MATH_L1 + np.sin(j2) * MATH_LOWER_ARM + np.sin(j2 + j3) * MATH_UPPER_ARM])
        #
        # 逆運動学確認
        # x = np.cos(j1) * (MATH_L2 + np.cos(j2) * MATH_LOWER_ARM + np.cos(j2 + j3) * MATH_UPPER_ARM + MATH_FRONT_HEADER)
        # y = np.sin(j1) * (MATH_L2 + np.cos(j2) * MATH_LOWER_ARM + np.cos(j2 + j3) * MATH_UPPER_ARM + MATH_FRONT_HEADER)
        # z = MATH_L1 + np.sin(j2) * MATH_LOWER_ARM + np.sin(j2 + j3) * MATH_UPPER_ARM - MATH_ENDEFFECTOR
        # j1, j2, j3, j4, j5 = self.xyzToAngle(x, y, z + MATH_ENDEFFECTOR)
        # self.cid = pybullet.createConstraint(self.obj_body[5], -1, -1, -1, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
        #                                      [np.cos(j1) * (MATH_L2 + np.cos(j2) * MATH_LOWER_ARM + np.cos(j2 + j3) * MATH_UPPER_ARM + MATH_FRONT_HEADER),
        #                                       np.sin(j1) * (MATH_L2 + np.cos(j2) * MATH_LOWER_ARM + np.cos(j2 + j3) * MATH_UPPER_ARM + MATH_FRONT_HEADER),
        #                                       MATH_L1 + np.sin(j2) * MATH_LOWER_ARM + np.sin(j2 + j3) * MATH_UPPER_ARM - MATH_ENDEFFECTOR - 0.015])  # 0.015は、キューブのサイズ
        # 位置制御
        pybullet.setJointMotorControlArray(self.robotId, [0, 1, 2, 3, 4], pybullet.POSITION_CONTROL, targetPositions=[j1, j2, j3, j4, j5])

        self.cid = None

        self.jointNum = pybullet.getNumJoints(self.robotId)

        # 安定化させる
        for i in range(10):
            self.simulateOneStep()

    def createRobot(self):
        robotStartPos = [0, 0, 0]
        robotStartOrientation = pybullet.getQuaternionFromEuler([0., 0, 0])
        self.robotId = pybullet.loadURDF("urdf/model.urdf", robotStartPos, robotStartOrientation, useFixedBase=True)
        # self.robotId = pybullet.loadURDF("urdf/model.urdf", robotStartPos, robotStartOrientation)

        # 位置制御を外しておく
        # numJoint = pybullet.getNumJoints(self.robotId)
        # pybullet.setJointMotorControlArray(self.robotId, range(numJoint), pybullet.VELOCITY_CONTROL, targetVelocities=[0] * numJoint, forces=[0] * numJoint, positionGains=[0] * numJoint, velocityGains=[0] * numJoint)

        # pybullet.enableJointForceTorqueSensor(robotId, 0, True)

    def simulateOneStep(self):
        pybullet.stepSimulation()

    def createObject(self, num):
        cube_halfextent = 0.015
        self.obj_body = []
        self.obj_num = num
        place = [[-0.1, -0.06, -0.02, -0.1, -0.06, -0.02], [0.3, 0.3, 0.3, 0.26, 0.26, 0.26]]
        for i in range(num):
            obj_visual = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[cube_halfextent] * 3)
            obj_collision = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[cube_halfextent] * 3)
            self.obj_body.append(pybullet.createMultiBody(baseMass=0.001, baseCollisionShapeIndex=obj_collision, baseVisualShapeIndex=obj_visual, basePosition=[place[0][i], place[1][i], cube_halfextent]))

    def forwardKinematics(self, j1, j2, j3, j4, j5):
        x = np.cos(j1) * (self.MATH_L2 + np.cos(j2) * self.MATH_LOWER_ARM + np.cos(j2 + j3) * self.MATH_UPPER_ARM + self.MATH_FRONT_HEADER)
        y = np.sin(j1) * (self.MATH_L2 + np.cos(j2) * self.MATH_LOWER_ARM + np.cos(j2 + j3) * self.MATH_UPPER_ARM + self.MATH_FRONT_HEADER)
        z = self.MATH_L1 + np.sin(j2) * self.MATH_LOWER_ARM + np.sin(j2 + j3) * self.MATH_UPPER_ARM - self.MATH_ENDEFFECTOR
        return x, y, z

    def getEEPos(self):
        curJointStates = pybullet.getJointStates(self.robotId, [0, 1, 2, 3, 4])
        curJointAng = [curJointStates[0][0], curJointStates[1][0], curJointStates[2][0], curJointStates[3][0], curJointStates[4][0]]
        curPos = self.forwardKinematics(curJointAng[0], curJointAng[1], curJointAng[2], curJointAng[3], curJointAng[4])
        return curPos

    def moveToTargetPos(self, x, y, z):
        theta = np.arctan2(y, x)
        # robotStartOrientation = pybullet.getQuaternionFromEuler([0., 0, -theta])
        # jointPoses_ik = list(pybullet.calculateInverseKinematics(self.robotId, self.link["link_3"], [x, y, z + MATH_ENDEFFECTOR], robotStartOrientation))

        # jointPoses = jointPoses_ik.copy()
        jointPoses = [0] * 5
        j1, j2, j3, j4, j5 = self.xyzToAngle(x, y, z)
        jointPoses[0] = j1
        jointPoses[1] = j2
        jointPoses[2] = j3
        jointPoses[3] = j4
        jointPoses[4] = j5
        pybullet.setJointMotorControlArray(self.robotId, [0, 1, 2, 3, 4], pybullet.POSITION_CONTROL,
                                           targetPositions=jointPoses)

        curPos = self.getEEPos()

        diff = np.sqrt((curPos[0] - x) ** 2 + (curPos[1] - y)**2 + (curPos[2] - z)**2)

        # diff_eular = np.fabs(np.fabs(curEular[2]) - np.pi)
        result = False
        if(diff < 0.0001):
            print("pos tgt:", x, y, z, "cur:", curPos)
            result = True
        else:
            # print("jnt tgt:", jointPoses, "cur:", curJointAng, "diff:", diff)
            print("pos tgt:", x, y, z, "cur:", curPos, "diff:", diff)

        return result

        # print(type(jointPoses[0]), type(theta))
        # print(theta)
        # print(pybullet.getLinkState(self.robotId, 4))
        # pybullet.setJointMotorControl2(bodyUniqueId=self.robotId, jointIndex=0, controlMode=pybullet.TORQUE_CONTROL, force=i, positionGain=0.0, velocityGain=0.0)
    # pybullet.setJointMotorControlArray(robotId, [0], pybullet.TORQUE_CONTROL, forces=[i])
    # pybullet.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0, controlMode=pybullet.VELOCITY_CONTROL, targetVelocity=i, force=1000)
    def pickPlace(self, pick_x, pick_y, pick_z, place_x, place_y, place_z):
        result = False
        if(self.picknow == False):
            self.picknow = True
            self.pick_step1 = True
            self.pick_step2 = True
            self.pick_step3 = True
            self.place_step1 = True
            self.place_step2 = True

        hight = 0.17

        if(self.pick_step1 == True):
            ret = self.moveToTargetPos(pick_x, pick_y, hight)
            if(ret == True):
                self.pick_step1 = False
        elif(self.pick_step2 == True):
            ret = self.moveToTargetPos(pick_x, pick_y, pick_z)
            if(ret == True):
                self.pick_step2 = False
                self.attachObject()
        elif(self.pick_step3 == True):
            ret = self.moveToTargetPos(pick_x, pick_y, hight)
            if(ret == True):
                self.pick_step3 = False

        elif(self.place_step1 == True):
            ret = self.moveToTargetPos(place_x, place_y, hight)
            if(ret == True):
                self.place_step1 = False
        elif(self.place_step2 == True):
            ret = self.moveToTargetPos(place_x, place_y, place_z)
            if(ret == True):
                self.place_step2 = False
                self.picknow = False
                result = True
                self.detachObject()

        return result

    def attachObject(self):
        objsize = 0.015
        for i in range(self.obj_num):
            eePos = self.getEEPos()
            objPos = list(pybullet.getBasePositionAndOrientation(self.obj_body[i])[0])
            objPos[2] += objsize
            diff = np.sqrt((eePos[0] - objPos[0]) ** 2 + (eePos[1] - objPos[1])**2 + (eePos[2] - objPos[2])**2)
            if(diff < 0.005):
                self.cid = pybullet.createConstraint(self.obj_body[i], -1, self.robotId, 4, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, objsize], [0, 0, -0.05])

    def detachObject(self):
        if(self.cid != None):
            pybullet.removeConstraint(self.cid)
            self.cid = None

    def xyzToAngle(self, x, y, z):

        xIn = 0.0
        zIn = 0.0
        rightAll = 0.0
        sqrtZX = 0.0
        phi = 0.0

        j1 = np.arctan2(y, x)
        MATH_UPPER_LOWER = self.MATH_UPPER_ARM / self.MATH_LOWER_ARM

        zIn = (z + self.MATH_ENDEFFECTOR - self.MATH_L1) / self.MATH_LOWER_ARM

        if(j1 != np.pi / 2):  # xIn is the stretch
            xIn = (x / np.cos(j1) - self.MATH_L2 - self.MATH_FRONT_HEADER) / self.MATH_LOWER_ARM
        else:
            xIn = (y - self.MATH_L2 - self.MATH_FRONT_HEADER) / self.MATH_LOWER_ARM

        phi = np.arctan2(zIn, xIn)
        sqrtZX = np.sqrt(zIn * zIn + xIn * xIn)

        j2All = (1 + sqrtZX * sqrtZX - MATH_UPPER_LOWER * MATH_UPPER_LOWER) / (2 * sqrtZX)
        j2 = np.arccos(j2All)

        j3All = (1 + MATH_UPPER_LOWER * MATH_UPPER_LOWER - sqrtZX * sqrtZX) / (2 * MATH_UPPER_LOWER)
        j3 = np.arccos(j3All)

        j2 = j2 + phi
        j3 = j3 - np.pi
        j4 = -(j2 + j3)
        j5 = np.pi - j1

        return j1, j2, j3, j4, j5


simu = Simulator(pybullet.GUI)
tgty = 0.3
tgtno = 0
while(pybullet.isConnected()):
    simu.simulateOneStep()

    place1 = [
        [-0.1, -0.06, -0.02, -0.1, -0.06, -0.02],
        [0.3,  0.3,  0.3, 0.26,  0.26,  0.26]
    ]
    place2 = [
        [0.1, 0.06, 0.02, 0.1, 0.06, 0.02],
        [0.3, 0.3, 0.3, 0.26, 0.26, 0.26]
    ]

    if(tgtno < 6):
        ret = simu.pickPlace(place1[0][tgtno], place1[1][tgtno], 0.03, place2[0][tgtno], place2[1][tgtno], 0.03)
    else:
        ret = simu.pickPlace(place2[0][tgtno - 6], place2[1][tgtno - 6], 0.03, place1[0][tgtno - 6], place1[1][tgtno - 6], 0.03)

    if(ret == True):
        tgtno += 1
        if(tgtno >= 12):
            tgtno = 0
    #
    # img = pybullet.getCameraImage(300, 300)
    # img_array = (np.array(img[2])).reshape([300, 300, 4])
    # cv2.imshow('camera capture', img_array.astype(np.uint8))
    # k = cv2.waitKey(3)
    time.sleep(0.01)
