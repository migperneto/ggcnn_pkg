#!/usr/bin/env python

import numpy as np
from tf.transformations import quaternion_from_euler

# Ângulos de Euler (roll, pitch, yaw) em radianos
roll = np.pi/4
pitch = np.pi/2
yaw = np.pi/6

print(f'roll = {roll}, pitch = {pitch}, yaw = {yaw}')

# Converte para quaternion
quaternion = quaternion_from_euler(roll, pitch, yaw)

print("Quaternion: ", quaternion) #Retorna um array do tipo [x, y, z, w]