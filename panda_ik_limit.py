import pybullet as p
import os
import math
from datetime import datetime
import pybullet_data
from scipy.spatial.transform import Rotation as R

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

POSITION = [ -0.6, -0.0054715, 0.17928975]
ORIENTATION = [-0.16294472, 0.90890329, 0.34755264, -0.16294472]
# ROTATION = R.from_euler('z', -0.7853981633974483).as_matrix()
# ORIENTATION = R.from_quat(ORIENTATION).as_matrix()
# ORIENTATION = ROTATION * ORIENTATION
# ORIENTATION = R.from_matrix(ORIENTATION).as_quat()
ARM_CORD = [-0.5, 0, 0]
PATH = pybullet_data.getDataPath()
print(PATH)

def load_gripper_obj(orientation, position):
    mass = 0
    # path = os.path.join(PATH, "franka_panda", "meshes", "visual", "hand.obj")
    path = "/home/donglin/ROS1Projects/src/franka_description/meshes/visual/hand.dae"
    shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName=path,
                                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH |
                                    p.GEOM_CONCAVE_INTERNAL_EDGE
                            )
    p.createMultiBody(mass, -1, shape_id, position, orientation)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, -0.3])
robotId = p.loadURDF("/home/donglin/ROS1Projects/src/franka_description/robots/panda.urdf", [0, 0, 0], useFixedBase=1)
load_gripper_obj(ORIENTATION, POSITION)
# p.resetBasePositionAndOrientation(robotId, [0.7, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 8
numJoints = p.getNumJoints(robotId)

def get_robo_info():
    ll, ul, jr, jd = [], [], [], []
    for i in range(p.getNumJoints(robotId)):
        info = p.getJointInfo(robotId, i)
        ll.append(info[8])
        ul.append(info[9])
        jr.append(info[9] - info[8])
        jd.append(info[6])
    return ll, ul, jr, jd

ll, ul, jr, jd = get_robo_info()
#restposes for null space
rp = [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
#joint damping coefficents

for i in range(p.getNumJoints(robotId)):
  print(p.getJointInfo(robotId, i))

for i in range(7):
  p.resetJointState(robotId, i, rp[i])

p.setGravity(0, 0, 0)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 1

useOrientation = 1
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 0
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15

jointPoses = p.calculateInverseKinematics(robotId, kukaEndEffectorIndex, POSITION, ORIENTATION, ll, ul,
                                                  jr, rp, residualThreshold=1e-7)
print(len(jointPoses))
# jointPoses = p.calculateInverseKinematics(robotId, kukaEndEffectorIndex, POSITION, ORIENTATION, residualThreshold=1e-6)
jointPoses = [1.003042595245601, -0.9824273249253487, 2.1400531439688137, -2.1257619369379857, 0.13723159433276244, 3.0347238800267062, -0.10694162407075014]
for i in range(len(jointPoses)):
    p.setJointMotorControl2(bodyIndex=robotId,
                            jointIndex=i,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=jointPoses[i],
                            targetVelocity=0,
                            force=500,
                            positionGain=0.03,
                            velocityGain=1)

def get_link_pose(link_idx):
  result = p.getLinkState(robotId, link_idx, computeForwardKinematics=1)
  return result[0], result[1]


i=0
while 1:
    p.stepSimulation()
    # print(8, get_link_pose(8))
    # print(7, get_link_pose(7))


p.disconnect()