import sys
sys.path.append('..\..\..')
import time
from src.scao.quaternion import Quaternion
import numpy as np
from numpy import array
import requests
import vpython as vp
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

lx,ly,lz = 10,10,10 #longueur du satellit selon les axes x,y,z
dt = 0.1

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
Wnorms = []
Qs = []
t, ts, nbr_secondes = [], [], 0
Wmax = 0

#######################################
# Initialisation graphique matplotlib #
#######################################
fig = plt.figure()


def animate(i, Qs, ts):
    global nbr_secondes, dt, Wmax
    # required for dezooming
    nbr_secondes += dt
    try:
        # Read attitude
        response = requests.get('http://192.168.8.1:5000', verify=False, timeout=0.5)
        temps, M, W, B, Q = response.text.split("<br/><br/>")[-1].split("<br/>")
        temps = eval(temps)
        Q = Quaternion(*eval(Q)[:,0])
        M = eval(M)
        W = eval(W)
        B = eval(B)

        # add Q and t_ to lists
        Ws.append(W)
        Wnorms.append(np.linalg.norm(W))
        Qs.append(Q.vec())
        t.append(time.time())
        ts.append(temps)

        #update dynamic Wmax
        Wmax = max(Wmax, Wnorms[-1])


        plt.clf()

        # Draw quaternion
        plt.subplot(2,1,1)
            # vecteur (nuances de bleu)
        plt.plot(ts, np.array(Qs)[:,1][:,0], label = "$Q_{}$".format(1), c = "#87CEFA")
        plt.plot(ts, np.array(Qs)[:,2][:,0], label = "$Q_{}$".format(2), c = "#87CEEB")
        plt.plot(ts, np.array(Qs)[:,3][:,0], label = "$Q_{}$".format(3), c = "#00BFFF")
            # angle
        plt.plot(ts, np.array(Qs)[:,0][:,0], label = "$Q_{}$".format(0), c = 'r')

            # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.title('Attitude quaternion and angular speed over time')
        plt.ylabel('Attitude')
        plt.legend()
        plt.ylim((-1,1))
        plt.xlim(right = max(30, ts[-1])+int(nbr_secondes))

        # Draw W
        plt.subplot(2,1,2)

        plt.plot(ts, np.array(Ws)[:,0][:,0], label = "$W_{}$".format(0), c = "#87CEFA")
        plt.plot(ts, np.array(Ws)[:,1][:,0], label = "$W_{}$".format(1), c = "#87CEEB")
        plt.plot(ts, np.array(Ws)[:,2][:,0], label = "$W_{}$".format(2), c = "#00BFFF")

        plt.plot(ts, np.array(Wnorms), label = "$|W|$", c = 'r')
            # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.ylabel('Speed')
        plt.legend()
        plt.ylim((-Wmax*1.05,Wmax*1.05))
        plt.xlim(right = max(30, ts[-1])+int(nbr_secondes))

        # Actualisation de l'affichage graphique
        b_vector.axis = 10*vp.vector(*B[:,0]/np.linalg.norm(B))
        satellite.axis = vp.vector(*Q.V2R(array([[1],[0],[0]]))[:,0])
        satellite.up = vp.vector(*Q.V2R(array([[0],[0],[1]]))[:,0])

    except requests.exceptions.ConnectionError:
        print('[' + str(int(time.time())) + "] Did not reach server")
    except requests.exceptions.ReadTimeout:
        print('[' + str(int(time.time())) + "] Did not reach server")

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(ts, Qs), interval=1/dt)
plt.show()
