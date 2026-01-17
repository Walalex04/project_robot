
import os
import sys


ruta_actual = os.path.dirname(__file__)  # /home/proyecto1
ruta_proyecto2 = os.path.join(ruta_actual, '..', 'robot', 'tools')

ruta_proyecto2 = os.path.abspath(ruta_proyecto2)  # /home/proyecto2

sys.path.append(ruta_proyecto2)
print(ruta_proyecto2)

from InverseKinematics import InverseKinematics
import numpy as np


print("THIS SCRIPT IS A TEST FOR INVERSE KINEAMTICS")

body = [0.559, 0.12]
legs = [0.,0.00, 0.426, 0.345] 

ik = InverseKinematics(body, legs)


leg_pos = np.array([[0.559/2,0.559/2 ,-0.559/2,-0.559/2],
             [-0.12/2, 0.12/2, -0.12/2,-0.12/2],
              [0.7,0.7,0.7,0.7]])
angles = ik.inverse_kinematic(leg_pos, 0, 0,0, 0,0,0)

