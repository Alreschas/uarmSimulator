import pybullet
import pybullet_data
import time


def initSimulator(mode):
    pybullet.connect(mode)
    pybullet.resetSimulation()
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setTimeStep(0.01)
    plane = pybullet.loadURDF("plane.urdf")
    robotStartPos = [0, 0, 0]
    robotStartOrientation = pybullet.getQuaternionFromEuler([0., 0, 0])
    robotId = pybullet.loadURDF("urdf/model.urdf", robotStartPos, robotStartOrientation, useFixedBase=True)
    # robotId = pybullet.loadURDF("urdf/model.urdf", robotStartPos, robotStartOrientation)
    pybullet.setRealTimeSimulation(0)
    # 必ず、位置制御を外すこと
    pybullet.setJointMotorControlArray(robotId, [0, 1, 2, 3, 4], pybullet.VELOCITY_CONTROL, targetVelocities=[0.0, 0.0, 0.0, 0.0, 0.0], forces=[0.0, 0.0, 0.0, 0.0, 0.0], positionGains=[0.0, 0.0, 0.0, 0.0, 0.0], velocityGains=[0.0, 0.0, 0.0, 0.0, 0.0])

    for i in range(50):
        test_visual = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
        test_collision = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
        test_body = pybullet.createMultiBody(baseMass=1, baseCollisionShapeIndex=test_collision, baseVisualShapeIndex=test_visual, basePosition=[0.3 + i * 0.01, 0, 0.5 + i * 0.05])
    return robotId


robotId = initSimulator(pybullet.GUI)
pybullet.enableJointForceTorqueSensor(robotId, 0, True)
i = 0.0
while(pybullet.isConnected()):
    # pybullet.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0, controlMode=pybullet.TORQUE_CONTROL, force=i, positionGain=0.0, velocityGain=0.0)
    # pybullet.setJointMotorControlArray(robotId, [0], pybullet.TORQUE_CONTROL, forces=[i])
    # pybullet.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0, controlMode=pybullet.VELOCITY_CONTROL, targetVelocity=i, force=1000)

    print(i, pybullet.getJointState(robotId, 0)[3])
    pybullet.stepSimulation()
    time.sleep(0.01)
