import pybullet as p
import pybullet_data as p_data
import os
p.connect(p.GUI)
p.setPhysicsEngineParameter(enableFileCaching=0)
robo_id = p.loadURDF("./dum.urdf")

# robo_id = p.loadURDF(os.path.join(p_data.getDataPath(), "franka_panda", "panda.urdf"), useFixedBase=1)
p.setJointMotorControl2(robo_id, 1, p.POSITION_CONTROL, targetPosition=0.5)
p.setJointMotorControl2(robo_id, 2, p.POSITION_CONTROL, targetPosition=0.5)
p.setJointMotorControl2(robo_id, 3, p.POSITION_CONTROL, targetPosition=0.5)
while 1:
    p.stepSimulation()