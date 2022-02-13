import os

from scipy.spatial.transform import Rotation as R
import pybullet as p

p.connect(p.GUI)
p.resetSimulation()
p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=8,
                             cameraPitch=-13.8, cameraTargetPosition=[-0.36, 1.02, -0.57])

POSITION = [0.2, 0.2, 0.2]
POSITION = [0.0, 0.0, 0.0]
ORIENTATION = [0.5, 0.5, 0.5]
ORIENTATION = [0.0, 0.0, 0.0]
ORIENTATION_QUAT = list(R.from_euler("xyz", ORIENTATION).as_quat())
PATH = ["/", "home", "donglin", "Github", "acronym",
        "data", "franka_gripper_collision_mesh.stl"]

TARGET_POS = [0, 0, 0.1]
STEP_PER_SECOND = 240


def load_visual_obj(path, position=[0, 0, 0], orientation=[0, 0, 0, 1]):
    shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                   fileName=path,
                                   flags=p.GEOM_FORCE_CONCAVE_TRIMESH |
                                   p.GEOM_CONCAVE_INTERNAL_EDGE
                                   )
    return p.createMultiBody(baseVisualShapeIndex=shape_id, basePosition=position, baseOrientation=orientation)


def load_primitive_collision_shape(position=[0, 0, 0], orientation=[0, 0, 0, 1], radius=0.1):
    shape_id = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius
    )
    return p.createMultiBody(baseMass=0.2, baseCollisionShapeIndex=shape_id, basePosition=position, baseOrientation=orientation)


GRIPPER_PATH = "./robots/hand.urdf"
robot_id = p.loadURDF(
    GRIPPER_PATH,
    useFixedBase=True,
    basePosition=POSITION,
    baseOrientation=ORIENTATION_QUAT,
)

cur_time = 0

while 1:
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    if cur_time == 0 * STEP_PER_SECOND:
        p.setJointMotorControlArray(
            robot_id,
            [7, 8],
            controlMode=p.POSITION_CONTROL,
            targetPositions=[0.02, 0.02],
        )
    elif cur_time == 1 * STEP_PER_SECOND:
        # collision_id = load_primitive_collision_shape(TARGET_POS, radius=0.01)
        collision_id = p.loadURDF('/home/donglin/Data/URDFs/circle.urdf', basePosition=TARGET_POS)
        p.changeDynamics(collision_id, 0, lateralFriction=1.0)
    elif cur_time == 2 * STEP_PER_SECOND:
        p.setJointMotorControlArray(
            robot_id,
            [7, 8],
            controlMode=p.POSITION_CONTROL,
            targetPositions=[0.0, 0.0],
            forces=[200, 200]
        )
    elif cur_time == 3 * STEP_PER_SECOND:
        p.setJointMotorControlArray(
            robot_id,
            [1, 2, 3],
            controlMode=p.POSITION_CONTROL,
            targetPositions=[0.3, 0.3, 0.3],
            targetVelocities=[0.001, 0.001, 0.001]
        )
    elif cur_time == 4 * STEP_PER_SECOND:
        p.setJointMotorControlArray(
            robot_id,
            [4, 5, 6],
            controlMode=p.POSITION_CONTROL,
            targetPositions=[0.3, 0.3, 0.3],
        )

    cur_time += 1

    p.stepSimulation()
