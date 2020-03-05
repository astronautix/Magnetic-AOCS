import sys
import os
sys.path.append(os.path.join(*['..']*3))
import time
from scao.quaternion import Quaternion
import numpy as np
from numpy import array
import requests
import vpython as vp

lx,ly,lz = 10,10,10 #longueur du satellit selon les axes x,y,z
dt = 0.01

############################
# Initialisation graphique #
############################
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
b_vector = vp.arrow(pos=vp.vector(-5,-5,-5), axis=10*vp.vector(0,0,0), shaftwidth=0.5, color=vp.vector(1,1,1))


Ws = []
ts = []


while True:
    try:
        response = requests.get('http://192.168.8.1:5000', verify=False, timeout=0.5)
        t, M, W, B, Q = response.text.split("<br/>")
        t = eval(t)
        print(t)
        Q = Quaternion(*eval(Q)[:,0])
        M = eval(M)
        W = eval(W)
        B = eval(B)

        Ws.append(W)
        ts.append(t)

        # Actualisation de l'affichage graphique
        b_vector.axis = 10*vp.vector(*B[:,0]/np.linalg.norm(B))
        satellite.axis = vp.vector(*Q.V2R(array([[1],[0],[0]]))[:,0])
        satellite.up = vp.vector(*Q.V2R(array([[0],[0],[1]]))[:,0])
    except requests.exceptions.ConnectionError:
        print('[' + str(int(time.time())) + "] Did not reached server")
    except requests.exceptions.ReadTimeout:
        print('[' + str(int(time.time())) + "] Did not reached server")
    time.sleep(dt)
