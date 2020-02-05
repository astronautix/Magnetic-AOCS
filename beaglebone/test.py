import sys
sys.path.append('..')
import os
from math import *
import time
import numpy as np
from scao.scao import SCAO
from scao.stabAlgs import PIDRW, PIDMT
import rcpy.mpu9250 as mpu9250
import rcpy.motor as motor

imu = mpu9250.IMU(enable_dmp = True, dmp_sample_rate = 100, enable_magnetometer = True)
mot = motor.Motor(1)

# Algortihmes de stabilisation
stab = SCAO(PIDRW(RW_P,RW_dP,RW_D),PIDMT(MT_P,MT_dP,MT_D),SCAOratio,I,J,dt) #stabilisateur

M = 0

while True:

    mot.set(0)
    time.sleep(.05)
    state = imu.read()

    W = np.array([[i*pi/360] for i in state['gyro']])
    Q = Quaternion(*state['quat'])
    B = np.array([[i*10**-6] for i in state['mag']])

    mot.set(max(M,1))

    # Sauvegarder les valeurs actuelles:
    stab.setAttitude(Q)
    stab.setRotation(W)
    stab.setMagneticField(B)

    # Prise de la commande de stabilisation
    dw, M = stab.getCommand(Qt) #dans Rv

    mot.set(max(M,1))

    console.log(str(M[:,0]), str(W[:,0]), str(Q.vec()[:,0]), str(B[:,0]))

    time.sleep(.2)
