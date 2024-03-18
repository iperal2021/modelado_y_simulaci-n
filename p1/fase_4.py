import pybullet as p
import time
import pybullet_data
import csv

prev_error = 0
prev_time = 0
current_time = 0
error = 0

Kp_1 = 3.1
Kd_1 = 0.1

Kp_2 = 2.5
Kd_2 = 1.5

Kp_3 = 5
Kd_3 = 1

min_speed = 11
min_torque = 100

def PID_llano(KP, KD):
    
    base_velocity = p.getBaseVelocity(robotId,0)[0][1]   
    error = 2 - abs(base_velocity)

    proportional_term = error * KP
    derivative_term = KD * ((error - prev_error))
    
    #inclination = p.getBasePositionAndOrientation(robotId)[1][0]
    speed =  min_speed + proportional_term + derivative_term
    torque = min_torque + proportional_term + derivative_term
    #print("Speed: ", speed)
    return speed, torque

def PID_subida(KP, KD):
    
    base_velocity = p.getBaseVelocity(robotId,0)[0][1]   
    error = 2 - abs(base_velocity)

    proportional_term = error * KP
    derivative_term = KD * ((error - prev_error))
    
    #inclination = p.getBasePositionAndOrientation(robotId)[1][0]
    speed =  min_speed + 5 + proportional_term + derivative_term
    torque = min_torque + proportional_term + derivative_term
    #print("Speed: ", speed)
    return speed, torque

def PID_bajada(KP, KD):
    
    base_velocity = p.getBaseVelocity(robotId,0)[0][1]   
    error = 2 - abs(base_velocity)

    proportional_term = error * KP
    derivative_term = KD * ((error - prev_error))
    
    #inclination = p.getBasePositionAndOrientation(robotId)[1][0]
    speed =  min_speed - proportional_term - derivative_term
    torque = min_torque + proportional_term + derivative_term
    #print("Speed: ", speed)
    return speed, torque

# URDF paths
rampa_urdf_path = "urdf/rampa_3.urdf"
finish_path = "urdf/finish_line.urdf"
barrera_urdf_path = "urdf/barrier_3_3.urdf"
husky_path = "husky/husky.urdf"

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(1)

planeId = p.loadURDF("plane.urdf")

# Position and orientation of the objects
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,3.15/2])

rampa_pos = [5, 10, 0.5]
rampa_orientation = p.getQuaternionFromEuler([0,0,3.15])

finish_line_pos = [0, 20, 1]
finish_orientation = p.getQuaternionFromEuler([0,0,3.15])

barrera_pos = [-1.5, 17, 0.1]
barrera_orientation = p.getQuaternionFromEuler([0,0,3.15])

# Generate the objects and robot
robotId = p.loadURDF(husky_path, startPos, startOrientation)
rampaId = p.loadURDF(rampa_urdf_path, rampa_pos, rampa_orientation)
finishId = p.loadURDF(finish_path, finish_line_pos, finish_orientation)
barreraId = p.loadURDF(barrera_urdf_path, barrera_pos, barrera_orientation)

p.resetDebugVisualizerCamera( cameraDistance=10, cameraYaw=60, cameraPitch=-50, cameraTargetPosition=p.getBasePositionAndOrientation(rampaId)[0])

p.changeDynamics(robotId, 2)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

# Joints IDs
joints = [2,3,4,5]

# Friction in the tires
for i in range(len(joints)):
    p.changeDynamics(robotId, joints[i], lateralFriction=0.93, spinningFriction=0.005, rollingFriction=0.003)

p.changeDynamics(barreraId, 0, localInertiaDiagonal=[2.5,0.0,2.5])
# Speed and torque values
# speed = 27
# torque = 100

try:
    init_time = time.time()
    init_pos = p.getLinkState(robotId, 0)[0][1]
    
    csv_data = []
    
    while True:
        #p.stepSimulation()
        #time.sleep(1./240.)
        
        robot_pos = p.getLinkState(robotId, 0)[0][1]
        base_velocity = p.getBaseVelocity(robotId,0)[0][1]
        
        # error = 2.5 - abs(base_velocity)
        current_time = time.time()
        time_elapsed = current_time - prev_time
        
        inclination = p.getBasePositionAndOrientation(robotId)[1][0]
        #print(inclination)
        if (inclination > - 0.1 and inclination < 0.14):
            #print("LLANO")
            speed, torque = PID_llano(Kp_1, Kd_1)
        elif (inclination >= 0.14):
            #print("SUBIDA")
            speed, torque = PID_subida(Kp_2, Kd_2)
        elif (inclination <= - 0.1):
            #print("BAJADA")
            speed, torque = PID_bajada(Kp_3, Kd_3)
            
        p.setJointMotorControlArray(robotId,
                              joints,
                              p.VELOCITY_CONTROL,
                              targetVelocities=[speed,speed,speed,speed],
                              forces=[speed,torque,torque,torque])
        

        if (robot_pos > 20):
            actual_time = time.time()
            print("Lap Time: ", actual_time - init_time)
            
            # Creating the csv file
            with open('Fase4.csv', 'w', newline='') as file:
                writer = csv.writer(file)
                # Escribe la cabecera del CSV
                writer.writerow(["Time", "Robot Y pose", "Robot Y velocity", "Robot Tires vel", "Robot Tires torque"])
                writer.writerows(csv_data)
            break;
        
        # print("Robot Y pose: ",robot_pos)
        # print("Robot Y velocity: ", base_velocity)
        # print("Robot Tires vel: ", speed)
        # print("Robot Tires vel: ", torque)
        
        lap_time = (time.time() - init_time)
        
        if (robot_pos - init_pos) >= 0.01:
            # Escribe los datos en el CSV
            csv_data.append([lap_time, robot_pos, base_velocity, speed, torque])
            init_pos = robot_pos
        
        prev_error = error
        prev_time = current_time
except KeyboardInterrupt:
      pass
	
p.disconnect()    