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
from multiprocessing import Process

app=Flask(__name__)

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

M = np.array([[0],[0],[0]])

Qt = Quaternion(.5,.5,.5,.5)

Q = Quaternion(1,0,0,0)

W = np.array([[0],[0],[0]])

B = np.array([[0],[0],[0]])


@app.route('/')
def index():
    global M, Q, W, B
    return str(M) + "<br/>" + str(Q) + "<br/>" + str(W) + "<br/>" + str(B)

def launchServer():
    app.run(host='0.0.0.0')

server = Process(target=launchServer)
server.start()

class Runner(Process):
    def __init__(self):
        Process.__init__()
        self.M = 

while True:
    global M, Q, W, B
    mot.set(0)
    time.sleep(.05)
    state = imu.read()

    Q = Quaternion(*state['quat'])
    W = Q.V2R(np.array([[i*pi/360] for i in state['gyro']]))
    B = Q.V2R(np.array([[i*10**-6] for i in state['mag']]))

    mot.set(max(-1,min(-M[1],1)))

    # Sauvegarder les valeurs actuelles:
    stab.setAttitude(Q)
    stab.setRotation(W)
    stab.setMagneticField(B)

    # Prise de la commande de stabilisation
    dw, M = stab.getCommand(Qt) #dans Rv

    mot.set(max(-1,min(-M[1],1)))

    print("=======================\n\n\n===================\n", "M :", str(M[:,0]), "\n W :" , str(W[:,0]), "\n Q :", str(Q.vec()[:,0]), "\n B :", str(B[:,0]))

    time.sleep(.2)
