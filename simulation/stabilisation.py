import sys
sys.path.append('..')
from simulator import Simulator
import vpython as vp
from math import *
import numpy as np
from scao.scao import SCAO
from random import *

lx,ly,lz = 10,10,10
m = 1
dt = 1/50
I = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite
W0 = np.array([[0],[0],[0]])
L0 = np.dot(I,W0) # Moment cinétique initial !! Attention à bien vérifier que tout est dans le bon référentiel !!
dw = np.array([[0.],[0.],[0.]]) # vecteur de l'accélération angulaire des RI
M = np.array([[0.],[0.],[0.]]) # vecteur du moment magnétique des bobines
J = 1 # moment d'inertie des Ri
B = np.array([[0.],[0.],[1.]]) # Champ magnétique environnant

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
stab = SCAO(I,J,500,500,np.diag((0,0,0)),10*np.diag((1,1,1)),np.diag((0,0,0)),np.diag((0,0,0)),dt)

w_iw = np.zeros((3,1)) #vitesse des roues d'inertie
while True:
    W = sim.getNextIteration(M,dw,J,B,I) # on récupère le prochain vecteur rotation

    # Sauvegarder les valeurs de simulation actuelles:
    stab.setAttitude(sim.Q)
    stab.setRotation(W)
    stab.setMagneticField(np.dot(P_v_r,B))

    # Calculer le moment magnétique à fournir:
    #M = stab.getCommandDetumblingMagnetic()
    #dw = stab.getCommandDetumblingWheel()
    res = stab.getCommandStabWheel(np.array([[0.5],[0.5],[0.5],[0.5]]))
    dw = (res - w_iw)/dt # https://quaternions.online/
    w_iw = res

    # Rotate: rotation de tout l'objet autour de la droite de vecteur directeur <axis> et passant par <origin>)
    satellite.rotate(angle=np.linalg.norm(W)*dt, axis=vp.vector(W[0][0],W[1][0],W[2][0]), origin=vp.vector(10,10,10))

    # Rate : réalise 1/dt fois la boucle par seconde
    vp.rate(1/dt)
