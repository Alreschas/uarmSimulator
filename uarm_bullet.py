import pybullet
import pybullet_data
import time
import numpy as np
import cv2
import math
import quaternion


class Simulator:
    def __init__(self, mode):
        pybullet.connect(mode)
        pybullet.resetSimulation()
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.81)

        # pybullet.setTimeStep(0.001)
        pybullet.setRealTimeSimulation(1)

        self.plane = pybullet.loadURDF("plane.urdf")

        self.link = {"base": 0, "base_rot": 1, "link_1": 2, "link_2": 3, "link_3": 4, "link_end": 5}
        self.joint = {"base_to_base_rot": 0, "base_rot_to_link_1": 1, "link_1_to_link_2": 2, "link_2_to_link_3": 3, "link_3_to_link_end": 4}

        self.picknow = False

        self.createRobot()
        self.createObject()

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

        # 位置制御を外しておく
        # numJoint = pybullet.getNumJoints(self.robotId)
        # pybullet.setJointMotorControlArray(self.robotId, range(numJoint), pybullet.VELOCITY_CONTROL, targetVelocities=[0] * numJoint, forces=[0] * numJoint, positionGains=[0] * numJoint, velocityGains=[0] * numJoint)

    def simulateOneStep(self):
        pybullet.stepSimulation()

    def createObject(self):
        cube_halfextent = 0.015
        self.obj_body = []
        self.obj_num = 6
        place = [
            [-0.1, -0.1, -0.1, -0.07, -0.07, -0.07],
            [0.2, 0.23, 0.26, 0.2, 0.23, 0.26]
        ]

        for i in range(self.obj_num):
            objOrientation = pybullet.getQuaternionFromEuler([0., 0, 0])
            self.obj_body.append(pybullet.loadURDF("urdf/cube.urdf", basePosition=[place[0][i], place[1][i], cube_halfextent], baseOrientation=objOrientation, globalScaling=0.03))
            # obj_visual = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[cube_halfextent] * 3, rgbaColor=[0, 0, 0, 1])
            # obj_collision = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[cube_halfextent] * 3)
            # self.obj_body.append(pybullet.createMultiBody(baseMass=0.001, baseCollisionShapeIndex=obj_collision, baseVisualShapeIndex=obj_visual, basePosition=[place[0][i], place[1][i], cube_halfextent], baseOrientation=objOrientation))

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

        jointPoses = [0] * 5
        ok, j1, j2, j3, j4, j5 = self.inverseKinematics(x, y, z)
        if(not ok):
            print("cannot exec")
            return True

        jointPoses[0] = j1
        jointPoses[1] = j2
        jointPoses[2] = j3
        jointPoses[3] = j4
        jointPoses[4] = j5
        pybullet.setJointMotorControlArray(self.robotId, [0, 1, 2, 3, 4], pybullet.POSITION_CONTROL,
                                           targetPositions=jointPoses)

        curPos = self.getEEPos()

        diff = np.sqrt((curPos[0] - x) ** 2 + (curPos[1] - y)**2 + (curPos[2] - z)**2)

        result = False
        if(diff < 0.0001):
            result = True

        return result

    def pickPlace(self, pick_x, pick_y, pick_z, place_x, place_y, place_z):

        result = False
        if(self.picknow == False):
            self.picknow = True
            self.pick_step1 = True
            self.pick_step2 = True
            self.pick_step3 = True
            self.place_step1 = True
            self.place_step2 = True

        hight = 0.1

        ok1 = self.inverseKinematics(pick_x, pick_y, hight)[0]
        ok2 = self.inverseKinematics(pick_x, pick_y, pick_z)[0]
        ok3 = self.inverseKinematics(place_x, place_y, hight)[0]
        ok4 = self.inverseKinematics(place_x, place_y, place_z)[0]
        if(not ok1 or not ok2 or not ok3 or not ok4):
            print("cannot exec")
            return True

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
        robOrn = pybullet.getLinkState(self.robotId, 4)[1]
        rob2 = pybullet.getQuaternionFromEuler([0, 0, np.pi])
        q0 = np.quaternion(rob2[3], rob2[0], rob2[1], rob2[2])
        q1 = q0 * np.quaternion(robOrn[3], robOrn[0], robOrn[1], robOrn[2])
        for i in range(self.obj_num):
            eePos = self.getEEPos()
            objPos, objOrn = pybullet.getBasePositionAndOrientation(self.obj_body[i])
            objPos = list(objPos)
            objOrn = list(objOrn)
            objPos[2] += objsize
            diff = np.sqrt((eePos[0] - objPos[0]) ** 2 + (eePos[1] - objPos[1])**2 + (eePos[2] - objPos[2])**2)
            if(diff < 0.01):
                pos = np.matrix([(eePos[0] - objPos[0]), (eePos[1] - objPos[1]), objsize]).transpose()
                q2 = np.quaternion(objOrn[3], objOrn[0], objOrn[1], objOrn[2])
                q3 = quaternion.as_float_array(q1 * q2)
                print(q3)

                rotMat = np.matrix(pybullet.getMatrixFromQuaternion(objOrn)).reshape([3, 3])
                m = np.linalg.inv(rotMat)
                p = m.dot(pos)

                # , parentFrameOrientation=objOrn, childFrameOrientation=robOrn
                # self.cid = pybullet.createConstraint(self.obj_body[i], -1, self.robotId, 4, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, objsize], [0, 0, -0.05], parentFrameOrientation=robOrn, childFrameOrientation=objOrn)
                self.cid = pybullet.createConstraint(self.robotId, 4, self.obj_body[i], -1, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, -0.05], p, parentFrameOrientation=[q3[1], q3[2], q3[3], q3[0]])
                # self.cid = pybullet.createConstraint(self.robotId, 4, self.obj_body[i], -1, pybullet.JOINT_FIXED, [0, 0, 0], [0, 0, -0.05], [0, 0, objsize])

    def detachObject(self):
        if(self.cid != None):
            pybullet.removeConstraint(self.cid)
            self.cid = None

    def inverseKinematics(self, x, y, z):

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

        ok = True
        if(math.isnan(j1) or math.isnan(j2) or math.isnan(j3) or math.isnan(j4) or math.isnan(j5)):
            ok = False

        return ok, j1, j2, j3, j4, j5

    def getCameraImage(self):
        width = 960
        height = 720

        fov = 48.4555  # 垂直画角
        aspect = width / height
        near = 0.02
        far = 1

        campos = [0.0, 0.3, 0.5]

        view_matrix = pybullet.computeViewMatrix(campos, [campos[0], campos[1], 0], [0, 1, 0])
        projection_matrix = pybullet.computeProjectionMatrixFOV(fov, aspect, near, far)
        images = pybullet.getCameraImage(width, height, view_matrix, projection_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
        img = (np.array(images[2])).reshape([height, width, 4]).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)
        return img


def getObjectPos(image, LeftRight):
    cam_width = 960
    cam_height = 720
    areaAll = cam_width * cam_height
    result = []

    # グレースケールに変換
    img_gray = cv2.bitwise_not(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    mask = []
    if(LeftRight == 0):
        mask = np.zeros([cam_height, cam_width]).astype(np.uint8)
        mask[:, :int(cam_width / 2)] = 255
    else:
        mask = np.zeros([cam_height, cam_width]).astype(np.uint8)
        mask[:, int(cam_width / 2):] = 255
    img_gray = cv2.bitwise_not(cv2.bitwise_and(img_gray, img_gray, mask=mask))

    # 二値化
    res, img_dst = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY)

    # 境界探索
    imgEdge, contours, hierarchy = cv2.findContours(img_dst, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if(area > areaAll * 0.9 or area < 1000):
            continue

        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        image = cv2.drawContours(image, [box], 0, (0, 0, 255), 2)

        M = cv2.moments(cnt)
        if(M['m00'] <= 0.0001):
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        x = -cam_width / 2. + cx
        y = cam_height / 2. - cy
        x = x * (0.47 * np.tan(np.pi * (62.9275 / 2.) / 180.)) / (cam_width / 2.)
        y = y * (0.47 * np.tan(np.pi * (48.4555 / 2.) / 180.)) / (cam_height / 2.) + 0.3

        result.append([x, y])

    # # テンプレートマッチング
    # if(len(result) == 0):
    #     template = cv2.imread("template.jpg", 0)
    #     res, tmp_dst = cv2.threshold(template, 100, 255, cv2.THRESH_BINARY)
    #
    #     w, h = template.shape[::-1]
    #     res = cv2.matchTemplate(img_dst, template, cv2.TM_CCOEFF_NORMED)
    #     threshold = 0.5
    #     loc = np.where(res >= threshold)
    #     for pt in zip(*loc[::-1]):
    #         cv2.rectangle(image, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)
    #         x = -cam_width / 2. + (pt[0] + w / 2)
    #         y = cam_height / 2. - (pt[1] + h / 2)
    #         x = x * (0.47 * np.tan(np.pi * (61.9275 / 2.) / 180.)) / (cam_width / 2.)
    #         y = y * (0.47 * np.tan(np.pi * (48.4555 / 2.) / 180.)) / (cam_height / 2.) + 0.3
    #         result.append([x, y])

    size = (int(cam_width / 2), int(cam_height / 2))
    halfImg = cv2.resize(image, size)
    cv2.imshow("test", halfImg)
    return result


simu = Simulator(pybullet.GUI)
tgt = [0, 0]
ret = True
LeftRight = 0
goHome = False
while(pybullet.isConnected()):
    # simu.simulateOneStep()

    if(ret == True):
        goHome = False
        img = simu.getCameraImage()

        result = getObjectPos(img, LeftRight)
        if(len(result) != 0):
            tgt = [result[0][0], result[0][1]]
        else:
            LeftRight = not LeftRight
            goHome = True

        print("nextTgt:", tgt)

    if(goHome == True):
        ret = simu.moveToTargetPos(-(LeftRight - 0.5) * 0.1, 0.2, 0.1)
    else:
        tgt2 = tgt.copy()
        if(tgt[1] > 0.3):
            tgt2[1] = 0.3
        ret = simu.pickPlace(tgt[0], tgt[1], 0.03, -tgt2[0], tgt2[1], 0.06)

    time.sleep(0.01)
