import pybullet as p
import pybullet_data
import argparse
import time


parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()
urdf_path = args.urdf

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0,0,0.5]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

robotId = p.loadURDF(urdf_path,startPos, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

joints =[0,1]
#p.setRealTimeSimulation(1)

frictionId = p.addUserDebugParameter("head_jointFriction", 0, 20, 10)
torqueId = p.addUserDebugParameter("head_joint torque", -20, 20, 0)

try:
    while True:
        frictionForce = p.readUserDebugParameter(frictionId)
        jointTorque = p.readUserDebugParameter(torqueId)

        #set the joint friction
        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
        #apply a joint torque
        p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=jointTorque)

        
        p.stepSimulation()
        time.sleep(1./24.)


except KeyboardInterrupt:
      pass
	
p.disconnect()    
