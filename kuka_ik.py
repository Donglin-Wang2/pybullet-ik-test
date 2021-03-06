import pybullet as p
import os
import math
from datetime import datetime
import pybullet_data

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

POSITION = [ 0.06055646, -0.0054715, 0.17928975]
ORIENTATION = [-0.16294472, 0.90890329, 0.34755264, -0.16294472]
ARM_CORD = [-0.5, 0, 0]
PATH = pybullet_data.getDataPath()

def load_gripper_obj(orientation, position):
    mass = 0
    path = os.path.join(PATH, "kuka_iiwa", "meshes", "link_7.obj")
    shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName=path,
                                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH |
                                    p.GEOM_CONCAVE_INTERNAL_EDGE
                            )
    p.createMultiBody(mass, -1, shape_id, position, orientation)


p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, -0.3])
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=1)
load_gripper_obj(ORIENTATION, POSITION)
p.resetBasePositionAndOrientation(kukaId, [0.6, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
  exit()

#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

for i in range(p.getNumJoints(kukaId)):
  print(p.getJointInfo(kukaId, i)[8])

for i in range(numJoints):
  p.resetJointState(kukaId, i, rp[i])

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

jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, POSITION, ORIENTATION, ll, ul,
                                                  jr, rp, residualThreshold=1e-6)
# jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, POSITION, ORIENTATION, residualThreshold=1e-6)
print("KUKAID", kukaId)
for i in range(numJoints):
    p.setJointMotorControl2(bodyIndex=kukaId,
                            jointIndex=i,
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