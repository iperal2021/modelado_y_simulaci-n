import pybullet as p
import time
import pybullet_data


physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
# p.setRealTimeSimulation(1)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,3.15/2])


robotId = p.loadURDF("~/ikeeeer12/Work/modelado/p2/robot_v1/urdf/robot_v1.urdf", startPos, startOrientation)

# numJoints = p.getNumJoints(robotId)
# print("NumJoints: " + str(numJoints))

# p.resetDebugVisualizerCamera( cameraDistance=13, cameraYaw=60, cameraPitch=-50, cameraTargetPosition=p.getBasePositionAndOrientation(rampaId)[0])

# for j in range (numJoints):
#     print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

# joints = [2,3,4,5]

# speed = 10
# torque = 150

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
            
except KeyboardInterrupt:
      pass
	
p.disconnect()


# try:
#     init_time = time.time()
#     while True:
#         #p.stepSimulation()
#         #time.sleep(1./240.)
        
#         # p.setJointMotorControlArray(robotId,
#         #                       joints,
#         #                       p.VELOCITY_CONTROL,
#         #                       targetVelocities=[speed,speed,speed,speed],
#         #                       forces=[torque,torque,torque,torque])
        
#         # robot_pos = p.getLinkState(robotId, 0)[0][1]
        
#         # if (robot_pos > 20):
#         #     actual_time = time.time()
#         #     print("Lap Time: ", actual_time - init_time)
#         #     break;
        
#         # print("Robot Y pose: ",robot_pos)

# except KeyboardInterrupt:
#       pass
	
# p.disconnect()    