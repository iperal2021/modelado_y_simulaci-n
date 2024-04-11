import pybullet as p
import time
import pybullet_data
cube_path = "cube.urdf"
robot_path = "robot_with_cage/urdf/robot_with_cage.urdf"
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,2]
startOrientation = p.getQuaternionFromEuler([0,0,0])


robotId = p.loadURDF(robot_path, startPos, startOrientation)

cube1_pos = [0, 1, 5]
cube1_orientation = p.getQuaternionFromEuler([0,0,3.15])
cubeId = p.loadURDF(cube_path, cube1_pos, cube1_orientation)
numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

p.resetDebugVisualizerCamera( cameraDistance=13, cameraYaw=60, cameraPitch=-50, cameraTargetPosition=p.getBasePositionAndOrientation(robotId)[0])

for j in range (numJoints):
     print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

joints = [8,9,10,11]

# speed = 10
# torque = 150



try:
    init_time = time.time()
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
        
     #    p.setJointMotorControlArray(robotId,
     #                          joints,
     #                          p.VELOCITY_CONTROL,
     #                          targetVelocities=[speed,speed,speed,speed],
     #                          forces=[torque,torque,torque,torque])
        
     #    robot_pos = p.getLinkState(robotId, 0)[0][1]
        
     #    if (robot_pos < -20):
     #         actual_time = time.time()
     #         print("Lap Time: ", actual_time - init_time)
     #         break;
        
     #    print("Robot Y pose: ",robot_pos)

except KeyboardInterrupt:
      pass
	
p.disconnect()    