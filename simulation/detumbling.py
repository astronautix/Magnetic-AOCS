import sys
sys.path.append('..')
from simulator import Simulator
import vpython as vp
from math import *
import numpy as np
from scao.scao import SCAO
from scao.stabAlgs import PIDRW, PIDMT
from environnement.environment import Environment
from environnement.orbit import Orbit
from random import *
import matplotlib.pyplot as plt

def rd():
    return 2*(random()-0.5)

lx,ly,lz = 10,10,10
m = 1
dt = 1/50
I = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite
W0 = 0.5*np.array([[rd()],[rd()],[rd()]]) #rotation initiale dans le référentiel R_r
L0 = np.dot(I,W0) # Moment cinétique initial !! Attention à bien vérifier que tout est dans le bon référentiel !!
dw = np.array([[0.],[0.],[0.]]) # vecteur de l'accélération angulaire des RW
M = np.array([[0.],[0.],[0.]]) # vecteur du moment magnétique des bobines
J = 1 # moment d'inertie des Ri


# Environnement :
t=0
orbite = Orbit(0, pi/4, 0.6, 5, 100)
environnement = Environment(100)  #valeur de mu_e plus élevée pour que le champ ne soit pas trop faible

# Initialisation du champ magnétique:
orbite.setTime(t)
environnement.setPosition(orbite.getPosition())
B = environnement.getEnvironment() #dans le référentiel géocentrique
B = np.dot(orbite.A_xs(), np.dot(orbite.A_sy(), B)) # dans le référentiel du satellite

## Initialisation graphique ###

ux = vp.vector(1,0,0)
uy = vp.vector(0,1,0)
uz = vp.vector(0,0,1)

# trièdre (z,x,y)
axe_x_r = vp.arrow(pos=vp.vector(0,0,0), axis=10*ux, shaftwidth=0.5, color=vp.vector(1,0,0))
axe_y_r = vp.arrow(pos=vp.vector(0,0,0), axis=10*uy, shaftwidth=0.5, color=vp.vector(0,1,0))
axe_z_r = vp.arrow(pos=vp.vector(0,0,0), axis=10*uz, shaftwidth=0.5, color=vp.vector(0,0,1))

#création du satellite avec son repère propre
axe_x_s = vp.arrow(pos=vp.vector(10,10,10), axis=10*ux, shaftwidth=0.1, color=vp.vector(1,0,0))
axe_y_s = vp.arrow(pos=vp.vector(10,10,10), axis=10*uy, shaftwidth=0.1, color=vp.vector(0,1,0))
axe_z_s = vp.arrow(pos=vp.vector(10,10,10), axis=10*uz, shaftwidth=0.1, color=vp.vector(0,0,1))
sugarbox = vp.box(pos=vp.vector(10,10,10), size=vp.vector(lx,ly,lz), axis=vp.vector(0,0,0), up = uy)
satellite = vp.compound([axe_x_s,axe_y_s,axe_z_s,sugarbox])

#vecteur champ B
b_vector = vp.arrow(pos=vp.vector(-5,-5,-5), axis=10*vp.vector(B[0][0],B[1][0],B[2][0]), shaftwidth=0.1, color=vp.vector(1,1,1))


sim = Simulator(dt,L0) #on créée un objet sim qui fera les simus
stab = SCAO(PIDRW(3,3,2),PIDMT(3,3,2),0,I,J,dt)
nbit = 0
Wr = []
while True:
    t+=dt
    orbite.setTime(0.05*t)
    environnement.setPosition(orbite.getPosition())
    B = environnement.getEnvironment() #dans le référentiel géocentrique
    B = np.dot(orbite.A_xs(), np.dot(orbite.A_sy(), B)) # dans le référentiel du satellite
    b_vector.axis = 10*vp.vector(B[0][0],B[1][0],B[2][0])

    #B = [[1],[1],[0]] #impose constant B field for testing

    W = sim.getNextIteration(M,dw,J,B,I) # on récupère le prochain vecteur rotation
    Wr.append(np.linalg.norm(W))
    # Sauvegarder les valeurs de simulation actuelles:
    stab.setAttitude(sim.Q)
    stab.setRotation(W)

    stab.setMagneticField(B)

    if nbit >= 50: #ne lance pas immediatement le detumbling
        # Calculer les corrections
        dw, M = stab.getCommand(np.array([[0.5],[0.5],[0.5],[0.5]])) #dans Rv

        #print("Magnetic field:", str(np.linalg.norm(B)))
        #print("dw:", str(sim.Q.V2R(dw[:,0])), "|| M:", str(sim.Q.V2R(M[:,0])))
        print("W :", str(W[:,0]), "|| norm :", str(np.linalg.norm(W)), "|| dw :", str(dw[:,0]))

    # Rotate: rotation de tout l'objet autour de la droite de vecteur directeur <axis> et passant par <origin>)
    satellite.rotate(angle=np.linalg.norm(W)*dt, axis=vp.vector(W[0][0],W[1][0],W[2][0]), origin=vp.vector(10,10,10))

    # Rate : réalise 1/dt fois la boucle par seconde
    vp.rate(1/dt)
    nbit += 1
