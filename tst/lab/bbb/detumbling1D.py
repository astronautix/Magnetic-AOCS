import sys
import os
sys.path.append(os.path.join(*['..']*3))
from math import *
import time
import numpy as np
from src.scao.quaternion import Quaternion
from src.scao.scao import SCAO
from src.scao.stabAlgs import PIDRW, PIDMT
import rcpy.mpu9250 as mpu9250
import rcpy.motor as motor
from threading import Thread
import logging
from src.hardware.hardwares import Hardware
from server import Server

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

imu = mpu9250.IMU(enable_dmp = True, dmp_sample_rate = 100, enable_magnetometer = True)
mot = motor.Motor(1)

# Algorithmes de stabilisation

lx,ly,lz = 0.1,0.1,0.1 #longueur du satellite selon les axes x,y,z
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

stab = SCAO(PIDRW(RW_P,RW_dP,RW_D),PIDMT(MT_P,MT_dP,MT_D),SCAOratio,I,J) #stabilisateur
hardW = Hardware(mgt_parameters, 'custom coil')  #hardware (bobines custom)

Qt = Quaternion(.5,.5,.5,.5)

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
        nbitBeforeMagMeasure = round(1/0.1)
        nbit = 0
        while True:
            if nbit%nbitBeforeMagMeasure == 0:
                mot.set(0)
                time.sleep(.1)
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

                mot.set(max(-1,min(-self.U[1][0]/U_max,1)))
                time.sleep(.1)
            self.server.queue(self.M, self.W, self.B, self.Q)
            nbit += 1

runner = Runner()
runner.start()
