import sys
sys.path.append('..')
import os
from math import *
import time
import numpy as np
from scao.quaternion import Quaternion
from scao.scao import SCAO
from scao.stabAlgs import PIDRW, PIDMT
import rcpy.mpu9250 as mpu9250
import rcpy.motor as motor
from flask import Flask
from threading import Thread
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

imu = mpu9250.IMU(enable_dmp = True, dmp_sample_rate = 100, enable_magnetometer = True)
mot = motor.Motor(1)

# Algortihmes de stabilisation

lx,ly,lz = 10,10,10 #longueur du satellit selon les axes x,y,z
m = 1 #masse du satellite
M = np.array([[0.],[0.],[0.]]) # vecteur du moment magnétique des bobines
I = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite
J = 1 # moment d'inertie des RW
SCAOratio = 0
RW_P = 3
RW_dP = 2
RW_D = 3
MT_P = 0
MT_dP = 0
MT_D = 50000

stab = SCAO(PIDRW(RW_P,RW_dP,RW_D),PIDMT(MT_P,MT_dP,MT_D),SCAOratio,I,J) #stabilisateur

Qt = Quaternion(.5,.5,.5,.5)

class Runner(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.M = np.array([[0],[0],[0]])
        self.W = np.array([[0],[0],[0]])
        self.B = np.array([[0],[0],[0]])
        self.Q = Quaternion(1,0,0,0)

    def loop(self):
            mot.set(0)
            time.sleep(.05)
            state = imu.read()

            self.Q = Quaternion(*state['quat'])
            self.W = self.Q.V2R(np.array([[i*pi/360] for i in state['gyro']]))
            self.B = self.Q.V2R(np.array([[i*10**-6] for i in state['mag']]))

            mot.set(max(-1,min(-self.M[1],1)))

            # Sauvegarder les valeurs actuelles:
            stab.setAttitude(self.Q)
            stab.setRotation(self.W)
            stab.setMagneticField(self.B)

            # Prise de la commande de stabilisation
            dw, self.M = stab.getCommand(Qt) #dans Rv

            mot.set(max(-1,min(-self.M[1],1)))

            #print("=======================\n\n\n===================\n", "M :", str(self.M[:,0]), "\n W :" , str(self.W[:,0]), "\n Q :", str(self.Q.vec()[:,0]), "\n B :", str(self.B[:,0]))

            time.sleep(.2)

    def run(self):
        while True:
            self.loop()

runner = Runner()
runner.start()

app=Flask(__name__)

@app.route('/')
def index():
    state = imu.read()

    Q = Quaternion(*state['quat'])
    W = Q.V2R(np.array([[i*pi/360] for i in state['gyro']]))

    return repr(runner.M)+"<br/>"+repr(W)+"<br/>"+repr(runner.B)+"<br/>"+repr(Q.vec())

app.run(host='0.0.0.0')
