import sys
sys.path.append('..')
import time
from scao.quaternion import Quaternion
import numpy as np
from numpy import array
import requests
import vpython as vp
import matplotlib.pyplot as plt
import matplotlib.animation as animation

lx,ly,lz = 10,10,10 #longueur du satellit selon les axes x,y,z
dt = 0.01

####################################
# Initialisation graphique vpython #
####################################
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
Qs = []
t, ts, t_ = [], [], 0

#######################################
# Initialisation graphique matplotlib #
#######################################

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)


def animate(i, Qs, ts):
    global t_, dt
    t_ += dt
    try:
        # Read attitude
        response = requests.get('http://192.168.8.1:5000', verify=False, timeout=0.5)
        M, W, B, Q = response.text.split("<br/>")
        Q = Quaternion(*eval(Q)[:,0])
        M = eval(M)
        W = eval(W)
        B = eval(B)

        # add Q and t_ to lists
        Ws.append(W)
        Qs.append(Q)
        t.append(time.time())
        ts.append(t_)

        # Draw x and y lists
        ax.clear()
        for i in range(4):
            ax.plot(ts, Qs[:,i])

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('Attitude quaternion over Time')
        plt.ylabel('Attitude')
        plt.ylim((-1,1))
        plt.xlim((0,300))

        # Actualisation de l'affichage graphique
        b_vector.axis = 10*vp.vector(*B[:,0]/np.linalg.norm(B))
        satellite.axis = vp.vector(*Q.V2R(array([[1],[0],[0]]))[:,0])
        satellite.up = vp.vector(*Q.V2R(array([[0],[0],[1]]))[:,0])

    except requests.exceptions.ConnectionError:
        print('[' + str(int(time.time())) + "] Did not reached server")
    except requests.exceptions.ReadTimeout:
        print('[' + str(int(time.time())) + "] Did not reached server")

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(ts, Qs), interval=100)
plt.show()
