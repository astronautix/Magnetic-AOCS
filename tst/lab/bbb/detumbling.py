import sys
import os
sys.path.append(os.path.join(*['..']*3))
from math import *
import time
import numpy as np
from src.scao.quaternion import Quaternion
from src.scao.scao import SCAO
from src.scao.stabAlgs import PIDRW, PIDMTI
import rcpy.mpu9250 as mpu9250
import rcpy.motor as motor
from flask import Flask
from threading import Thread
import logging
from src.hardware.hardwares import Hardware
from server import Server

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

imu = mpu9250.IMU(enable_dmp = True, dmp_sample_rate = 100, enable_magnetometer = True)

rect = [-1,-1,1]
mots = [motor.Motor(1), motor.Motor(4), motor.Motor(3)] #Motor mapping : x,y,z

# Algorithmes de stabilisation

lx,ly,lz = 0.1,0.1,0.1 #longueur du satellite selon les axes x,y,z
dt = .1 #pas de temps entre deux loops
m = 1 #masse du satellite
M = np.array([[0.],[0.],[0.]]) # vecteur du moment magnétique des bobines
I = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite
J = 1 # moment d'inertie des RW
SCAOratio = 0
RW_P = 3
RW_dP = 2
RW_D = 3
MT_P = 5000
MT_dP = 0 #n'a pas d'effet
MT_D = 15000
MT_I = 0

#####
# paramètres hardware
n_windings = 500
r_wire = 125e-6
r_coil = 75e-4
U_max = 5
mu_rel = 31
J = 1 # moment d'inertie des RW
# r_coil, r_wire, n_coils, mu_rel, U_max
mgt_parameters = r_coil, r_wire, n_windings, mu_rel, U_max
#####

stab = SCAO(PIDRW(RW_P,RW_dP,RW_D),PIDMTI(MT_P,MT_dP,MT_D,MT_I),SCAOratio,I,J) #stabilisateur
hardW = Hardware(mgt_parameters, 'custom coil')  #hardware (bobines custom)

# set as target attitude the initial attitude
state = imu.read()
Qt = Quaternion(*state['quat'])

class Runner(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.M = np.array([[0],[0],[0]])
        self.U = np.array([[0],[0],[0]]) #tension
        self.W = np.array([[0],[0],[0]])
        self.B = np.array([[0],[0],[0]])
        self.Q = Quaternion(1,0,0,0)
        self.server = Server()

    def run(self):
        self.server.start()
        nbitBeforeMagMeasure = round(3)
        nbit = 0
        while True:
            if nbit%nbitBeforeMagMeasure == 0:
                for mot in mots:
                    mot.set(0)
                time.sleep(dt)
                state = imu.read()
                self.Q = Quaternion(*state['quat'])
                self.B = self.Q.V2R(np.array([[i*10**-6] for i in state['mag']]))
                stab.setMagneticField(self.B)
            else:
                state = imu.read()
                self.Q = Quaternion(*state['quat'])
                self.W = self.Q.V2R(np.array([[i*pi/360] for i in state['gyro']]))

                # Sauvegarder les valeurs actuelles:
                stab.setAttitude(self.Q)
                stab.setRotation(self.W)

                # Prise de la commande de stabilisation
                dw, self.M = stab.getCommand(Qt) #dans Rv
                self.U, self.M = hardW.getRealCommand(dw, self.M)
                for nomot, mot in enumerate(mots):
                    print(-self.U[nomot][0]/U_max)
                    mot.set(rect[nomot]*self.U[nomot][0]/U_max)
                print()
                time.sleep(dt)
            self.server.queue(self.M, self.W, self.B, self.Q*Qt.inv())
            nbit += 1

runner = Runner()
runner.start()
