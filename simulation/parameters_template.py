import numpy as np
from random import *
from math import *
###############################
# Paramètres de la simulation #
###############################
# Temps
dt = 50 #pas de temps de la simulation
fAffichage = 25 #fréquence d'affichage

# Géométrie
lx,ly,lz = 10,10,10 #longueur du satellit selon les axes x,y,z
m = 1 #masse du satellite

# Mouvement
W0 = 0*np.array([[2*(random()-0.5)] for i in range(3)]) #rotation initiale dans le référentiel R_r
Qt = np.array([[0.5],[0.5],[0.5],[0.5]]) #quaternion objectif

# SCAO parameters
RW_P = 3
RW_dP = 2
RW_D = 3
MT_e = 0.0005
MT_P = 50*MT_e**2
MT_D = 100*MT_e

# Hardware
n_windings = 400
A_coils = 8.64e-3
M_max = 0.13 #Moment maximum des MT
J = 1 # moment d'inertie des RW

# Orbite
omega = 0
i = pi/6
e = 0.01
r_p = 7e6
mu = 3.986004418e14
tau = 0

# Environment
B_model = 'wmm'
