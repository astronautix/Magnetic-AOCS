from simulator import Simulator
import vpython as vp
from math import *
import numpy as np


dt = 1/50
L0 = np.array([[0.],[0.],[0.]]) # Moment cinétique initial
I = np.eye(3) # Tenseur inertie du satellite
dw = np.array([[0.],[0.],[0.]]) # vecteur de l'accélération angulaire des RI
M = np.array([[1.],[0.],[0.]]) # vecteur du moment magnétique des bobines
J = 1 # moment d'inertie des Ri
B = np.array([[0.],[0.],[1.]]) # Champ magnétique environnant


ux = vp.vector(1,0,0)
uy = vp.vector(0,1,0)
uz = vp.vector(0,0,1)

# trièdre (z,x,y)
axe_x_r = vp.arrow(pos=vp.vector(0,0,0), axis=10*ux, shaftwidth=0.1, color=vp.vector(1,0,0))
axe_y_r = vp.arrow(pos=vp.vector(0,0,0), axis=10*uy, shaftwidth=0.1)
axe_z_r = vp.arrow(pos=vp.vector(0,0,0), axis=10*uz, shaftwidth=0.1)

#création du satellite avec son repère propre
axe_x_s = vp.arrow(pos=vp.vector(10,10,10), axis=10*ux, shaftwidth=0.1, color=vp.vector(1,0,0))
axe_y_s = vp.arrow(pos=vp.vector(10,10,10), axis=10*uy, shaftwidth=0.1)
axe_z_s = vp.arrow(pos=vp.vector(10,10,10), axis=10*uz, shaftwidth=0.1)
sugarbox = vp.box(pos=vp.vector(10,10,10), size=vp.vector(5,5,5), axis=vp.vector(0,0,0), up = uy)
satellite = vp.compound([axe_x_s,axe_y_s,axe_z_s,sugarbox])

#vecteur champ B
b_vector = vp.arrow(pos=vp.vector(-5,-5,-5), axis=10*vp.vector(B[0][0],B[1][0],B[2][0]), shaftwidth=0.1, color=vp.vector(0,1,0))

sim = Simulator(dt,L0) #on créée un objet sim qui fera les simus

while True:
    W = sim.getNextIteration(M,dw,J,B,I) # on récupère le prochain vecteur rotation
    # Rotate: rotation de tout l'objet autour de la droite de vecteur directeur <axis> et passant par <origin>)
    satellite.rotate(angle=np.linalg.norm(W)*dt, axis=vp.vector(W[0][0],W[1][0],W[2][0]), origin=vp.vector(10,10,10))
    # Rate : réalise 50 fois la boucle par seconde
    vp.rate(1/dt)
