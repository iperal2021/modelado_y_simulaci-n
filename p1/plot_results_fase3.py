import numpy as np
import matplotlib.pyplot as plt

# Lee los datos del archivo CSV
data_1 = np.genfromtxt('Fase3_1.csv', delimiter=',', skip_header=1)
data_2 = np.genfromtxt('Fase3_2.csv', delimiter=',', skip_header=1)
data_3 = np.genfromtxt('Fase3_3.csv', delimiter=',', skip_header=1)

# Extrae la posición y la velocidad del robot
robot_pos_1 = data_1[:, 1]
base_velocity_1 = data_1[:, 2]

robot_pos_2 = data_2[:, 1]
base_velocity_2 = data_2[:, 2]

robot_pos_3 = data_3[:, 1]
base_velocity_3 = data_3[:, 2]

# Crea el gráfico
plt.figure(figsize=(10, 6))
plt.plot(robot_pos_1, base_velocity_1, label='Fase 3.1')
plt.plot(robot_pos_2, base_velocity_2, label='Fase 3.2')
plt.plot(robot_pos_3, base_velocity_3, label='Fase 3.3')
plt.xlabel('Posición del robot (m)')
plt.ylabel('Velocidad del robot (m/s)')
plt.title('Variación de la velocidad del robot en función de su posición')
plt.legend()
plt.grid(True)
plt.legend() 
# Guarda el gráfico en un archivo
plt.savefig('grafico_robot_fase3_3.pdf')

plt.show()