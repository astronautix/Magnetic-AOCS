import sys
sys.path.append('..')
from simulator import Simulator
from math import *
import numpy as np
from scao.scao import SCAO
from scao.stabAlgs import PIDRW, PIDMT
from environnement.environment import Environment
from environnement.orbit import Orbit
from hardware.hardware import Hardware
from random import *
import matplotlib.pyplot as plt

def rd():
    return 2*(random()-0.5)

################
## Paramètres ##
################

## Satellite:
#============
# Geometry
lx,ly,lz = 10,10,10
m = 1
dt = 1/60
J = 1 # reaction wheels moment of inertia

# Hardware
n_windings = 400
A_coils = 8.64e-3
M_max = 0.13

## Mission
#=========

#Orbit
omega = 0
i = pi/6
e = 0.01
r_p = 7e6
mu = 100
tau = 0

#Environment
B_model = 'wmm'

# Target Attitude
Q_target = np.array([[0.5],[0.5],[0.5],[0.5]])


####################
## Initialisation ##
####################

t=0
t_max = 30

# Satellite
I = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite
W0 = 0.5*np.array([[rd()],[rd()],[rd()]]) #rotation initiale dans le référentiel R_r
L0 = np.dot(I,W0) # Moment cinétique initial !! Attention à bien vérifier que tout est dans le bon référentiel !!
dw = np.array([[0.],[0.],[0.]]) # vecteur de l'accélération angulaire des RW
M = np.array([[0.],[0.],[0.]]) # vecteur du moment magnétique des bobines
hardW = Hardware(n_windings, A_coils, M_max)

# Environnement
orbite = Orbit(omega, i, e, r_p, mu, tau)
environnement = Environment(B_model)

# Initialisation du champ magnétique:
orbite.setTime(t)
environnement.setPosition(orbite.getPosition())
B = environnement.getEnvironment()  # dans le référentiel du satellite

# Simulation
sim = Simulator(dt,L0) #on créée un objet sim qui fera les simus
stab = SCAO(PIDRW(3,3,2),PIDMT(3,3,2),0,I,J,dt)
nbit = 0

# Valeurs
values = {'W':[], 'B':[], 't':[], 'M':[], 'i': []}

while t<t_max:
    t+=dt
    orbite.setTime(t) #0.05*t
    environnement.setPosition(orbite.getPosition())

    B = environnement.getEnvironment() #dans le référentiel géocentrique
    B = np.dot(orbite.A_xs(), np.dot(orbite.A_sy(), B)) # dans le référentiel du satellite

    #B = [[1],[1],[0]] #impose constant B field for testing

    W = sim.getNextIteration(M,dw,J,B,I) # on récupère le prochain vecteur rotation
    # Sauvegarder les valeurs de simulation actuelles:
    stab.setAttitude(sim.Q)
    stab.setRotation(W)
    stab.setMagneticField(B)


    if nbit >= 50: #ne lance pas immediatement le detumbling
        # Calculer les corrections
        dw, M = stab.getCommand(Q_target) #dans Rv
        M, current = hardW.getRealMoment(dw, M)

        #print("Magnetic field:", str(np.linalg.norm(B)))
        #print("dw:", str(sim.Q.V2R(dw[:,0])), "|| M:", str(sim.Q.V2R(M[:,0])))
        #print("W :", str(W[:,0]), "|| norm :", str(np.linalg.norm(W)), "|| dw :", str(dw[:,0]), "|| B :", str(B[:,0]))

    # sauvegarde des valeurs
    values['W'].append(W)
    values['B'].append(B)
    values['t'].append(t)
    values['M'].append(M)
    values['i'].append(current)
    nbit += 1
