import pybullet as p
import os
import math
from datetime import datetime
import pybullet_data

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)

POSITION = [ 0.06055646, -0.0054715, 0.17928975]
ORIENTATION = [-0.16294472, 0.90890329, 0.34755264, -0.16294472]
ARM_CORD = [-0.5, 0, 0]
PATH = pybullet_data.getDataPath()
print(PATH)

def load_gripper_obj(orientation, position):
    mass = 0
    path = "/home/donglin/Desktop/robots/meshes/visual/hand.dae"
    shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName=path,
                                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH |
                                    p.GEOM_CONCAVE_INTERNAL_EDGE
                            )
    p.createMultiBody(mass, -1, shape_id, position, orientation)

def get_joint_infos(robotId):
    for i in range(p.getNumJoints(robotId)):
        print(p.getJointInfo(robotId, i))



p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, -0.3])
kukaId = p.loadURDF("/home/donglin/Desktop/robots/model.urdf", [0, 0, 0])


load_gripper_obj(ORIENTATION, POSITION)
p.resetBasePositionAndOrientation(kukaId, [0.6, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 8
numJoints = 7



p.setGravity(0, 0, 0)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]


useRealTimeSimulation = 0
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15

jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, POSITION, ORIENTATION, residualThreshold=1e-6)

for i in range(numJoints):
    p.setJointMotorControl2(bodyIndex=kukaId,
                            jointIndex=i+1,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=jointPoses[i],
                            targetVelocity=0,
                            force=500,
                            positionGain=0.03,
                            velocityGain=1)

i=0
while 1:
    p.stepSimulation()

p.disconnect()