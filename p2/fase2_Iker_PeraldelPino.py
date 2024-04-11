import pybullet as p
import time
import pybullet_data


STATE = 0

robot_path = "robot_new/urdf/robot_new.urdf"
cube_path = "cube.urdf"


physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
# cube_1_ID = p.loadURDF("cube.urdf")


startPos = [0,1,2]
startOrientation = p.getQuaternionFromEuler([0,0,3.14])

cube1_pos = [0, 4, 2]
cube1_orientation = p.getQuaternionFromEuler([0,0,3.15])

robotId = p.loadURDF(robot_path, startPos, startOrientation)
cubeId = p.loadURDF(cube_path, cube1_pos, cube1_orientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

p.resetDebugVisualizerCamera( cameraDistance=13, cameraYaw=60, cameraPitch=-50, cameraTargetPosition=p.getBasePositionAndOrientation(robotId)[0])

for j in range (numJoints):
     print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

joints = [0,1,2,3]
pinza = [5,6]
ruedas = [7,8,9,10]

# ------ JOINTS LISTS ------

# 0 - eje_brazo_1_joint_
# 1 - eje_brazo_2_joint_
# 2 - eje_brazo_3
# 3 - eje_brazo_4_joint_
# 4 - cabeza_pinza_link_joint_
# 5 - garra_left_joint
# 6 - garra_right_joint
# 7 - rueda_a_d_link_joint
# 8 - rueda_a_i_link_joint
# 9 - rueda_d_l_link_joint
# 10 - rueda_d_r_link_joint


# ---------------------------

robotEndEffectorIndex=4

speed = 0
torque = 0

pos_target = [0,3.7,0.4]
jointPoses_1 = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, pos_target)

pos_point_1 = [0,3.9, 3]
jointPoses_2 = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, pos_point_1)

pos_point_2 = [2, 1, 4]
jointPoses_3 = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, pos_point_2)

pos_point_3 = [1,-1, 4]
jointPoses_4 = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, pos_point_3)
try:
    init_time = time.time()
    while True:
          p.stepSimulation()
          time.sleep(1./240.)
          
          if STATE == 0:
               robot_pos = p.getLinkState(robotId, 10)[0][1]
               p.setJointMotorControlArray(robotId,
                                ruedas,
                                p.VELOCITY_CONTROL,
                                targetVelocities=[speed,speed,speed,speed])
               p.setJointMotorControlArray(robotId,
                                ruedas,
                                p.TORQUE_CONTROL,
                                forces=[torque,torque,torque,torque])
               
               print(robot_pos)
               if (robot_pos > 1):
                    STATE = 2
                    speed = 0
                    torque = 0
                    print("llegué")
                    tiempo_espera = time.time()
                    
          elif STATE == 2:
               print("Move arm")
               
               if (time.time() - tiempo_espera > 2):
           
                    print(jointPoses_1)
                    p.addUserDebugText("X", pos_target, [0,4,1], 1)


                    
                    p.setJointMotorControlArray(bodyIndex=robotId,
                                                       jointIndices=joints,
                                                       controlMode=p.POSITION_CONTROL,
                                                       targetPositions=[jointPoses_1[0],jointPoses_1[1],jointPoses_1[2],jointPoses_1[3]],
                                                       targetVelocities=[0,0,0,0],
                                                       forces=[100,100,100,100],
                                                       positionGains=[0.01, 0.01, 0.01, 0.01],
                                                       velocityGains=[1,1,1,1])
                    
                    STATE = 3
                    tiempo_espera = time.time()
          
          elif STATE == 3:
               if (time.time() - tiempo_espera > 2):
                    print("cerrar pinza")
                    p.setJointMotorControlArray(robotId,
                                        jointIndices=pinza,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=[0.25,0.25],
                                        targetVelocities=[0,0],
                                        forces=[500,500],
                                        positionGains=[0.05,0.05],
                                        velocityGains=[1,1])
                    
                    STATE = 4
                    tiempo_espera = time.time()

          elif STATE == 4:
               print("Primer punto")
               if (time.time() - tiempo_espera > 2):
                    # print(jointPoses_2)
                    p.addUserDebugText("1", pos_point_1, [0,3.9,3], 1)

                    p.setJointMotorControlArray(bodyIndex=robotId,
                                                  jointIndices=joints,
                                                  controlMode=p.POSITION_CONTROL,
                                                  targetPositions=[jointPoses_2[0],jointPoses_2[1],jointPoses_2[2],jointPoses_2[3]],
                                                  targetVelocities=[0,0,0,0],
                                                  forces=[500,500,500,500],
                                                  positionGains=[0.01, 0.01, 0.01, 0.01],
                                                  velocityGains=[1,1,1,1])
                    STATE = 5
                    tiempo_espera = time.time()
                    print("Segundo punto")
                    
          elif STATE == 5:
               
               # 
               if (time.time() - tiempo_espera > 2):
                    # print(jointPoses_3)
                    p.addUserDebugText("2", pos_point_2, [2,0,4], 1)
                    
                    p.setJointMotorControlArray(bodyIndex=robotId,
                                                  jointIndices=joints,
                                                  controlMode=p.POSITION_CONTROL,
                                                  targetPositions=[jointPoses_3[0],jointPoses_3[1],jointPoses_3[2],jointPoses_3[3]],
                                                  targetVelocities=[0,0,0,0],
                                                  forces=[500,500,500,500],
                                                  positionGains=[0.01, 0.01, 0.01, 0.01],
                                                  velocityGains=[1,1,1,1])
                    
                    STATE = 6
                    tiempo_espera = time.time()
                    print("Tercer punto")
                    
          elif STATE == 6:
               
               
               if (time.time() - tiempo_espera > 2):
                    # print(jointPoses_3)
                    p.addUserDebugText("3", pos_point_3, [255,0,0], 1)
                    
                    p.setJointMotorControlArray(bodyIndex=robotId,
                                                  jointIndices=joints,
                                                  controlMode=p.POSITION_CONTROL,
                                                  targetPositions=[jointPoses_4[0],jointPoses_4[1],jointPoses_4[2],jointPoses_4[3]],
                                                  targetVelocities=[0,0,0,0],
                                                  forces=[500,500,500,500],
                                                  positionGains=[0.01, 0.01, 0.01, 0.01],
                                                  velocityGains=[1,1,1,1])
                    
                    STATE = 7
               
                    tiempo_espera = time.time()
          #    if (robot_pos < -20):
          #         actual_time = time.time()
          #         print("Lap Time: ", actual_time - init_time)
          #         break;
          
          #    print("Robot Y pose: ",robot_pos)

except KeyboardInterrupt:
      pass
	
p.disconnect()    