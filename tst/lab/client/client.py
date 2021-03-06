import sys
import os
sys.path.append(os.path.join(*['..']*3))
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
import datetime

lx,ly,lz = 10,10,10 #longueur du satellit selon les axes x,y,z
dt = 0.1

vpython_display = False

####################################
# Initialisation graphique vpython #
####################################
if vpython_display:
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


Ws, Qs, Cs, Ms, Bs = [], [], [], [], []
Wnorms, Cnorms, Mnorms, Bnorms = [], [], [], []
ts, nbr_secondes = [], 0
Wmax, Cmax, Mmax, Bmax = 0, 0, 0, 0
t0 = 0


#### FONCTION D'AFFICHAGE

def plot3and1(Ys, Ynorms, Ymax, title, variable_name, indexes):
    plt.subplot(indexes)
    plt.plot(ts, np.array(Ys)[:,0][:,0], label = "${}_{}$".format(variable_name,0), c = "#87CEFA")
    plt.plot(ts, np.array(Ys)[:,1][:,0], label = "${}_{}$".format(variable_name,1), c = "#87CEEB")
    plt.plot(ts, np.array(Ys)[:,2][:,0], label = "${}_{}$".format(variable_name,2), c = "#00BFFF")

    plt.plot(ts, np.array(Ynorms), label = "$|{}|$".format(variable_name), c = 'r')
        # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.ylabel(title)
    plt.xlabel('Time (s)')
    plt.legend()
    plt.ylim((-Ymax*1.05,Ymax*1.05))
    plt.xlim(right = max(30, ts[-1])+int(nbr_secondes))
#######################################
# Initialisation graphique matplotlib #
#######################################
fig = plt.figure()

# enregistrer fichier
os.chdir('datas')
name = input('Name for datas ?\n')
filename = str(datetime.datetime.now().strftime('%H_%M_%S_%f'))+"_"+name+"_"+".txt"

def animate(i, Qs, ts):
    global nbr_secondes, dt, Wmax, Cmax, t0, Mmax, Bmax
    # required for dezooming
    nbr_secondes += dt
    try:
        # Read attitude
        response_raw = requests.get('http://192.168.8.1:5000', verify=False, timeout=0.5)
        if response_raw.text != "":
            for line in response_raw.text.split("<br/><br/>"):
                temps, M, W, B, Q = line.split("<br/>")
                temps = eval(temps)

                if len(ts) == 0:
                    t0 = temps

                temps -= t0

                if len(ts) != 0:
                    if temps <= ts[-1]:
                        continue

                Q = Quaternion(*eval(Q)[:,0])
                M = eval(M)
                W = eval(W)
                B = eval(B)
                C = np.cross(M, B, axisa=0, axisb=0,axisc=0)

                with open(filename, 'a') as f:
                    f.write(str(temps) + '\t' + str(list(M[:,0])) + '\t' + str(list(W[:,0])) + '\t' + str(list(B[:,0])) + '\t' + str(list(Q.vec()[:,0])) + "\t" + str(list(C[:,0])) + '\n')

                # add values to lists
                Ws.append(W)
                Qs.append(Q.vec())
                Cs.append(C)
                Ms.append(M)
                Bs.append(B)

                Wnorms.append(np.linalg.norm(W))
                Cnorms.append(np.linalg.norm(C))
                Mnorms.append(np.linalg.norm(M))
                Bnorms.append(np.linalg.norm(B))

                ts.append(temps)




                #update dynamic Wmax, Cmax
                Wmax = max(Wmax, Wnorms[-1])
                Cmax = max(Cmax, Cnorms[-1])
                Mmax = max(Mmax, Mnorms[-1])
                Bmax = max(Bmax, Bnorms[-1])

            #clear plots
            plt.clf()

            ### Draw quaternion
            plt.subplot(2,2,1)
                # vecteur (nuances de bleu)
            plt.plot(ts, np.array(Qs)[:,1][:,0], label = "$Q_{}$".format(1), c = "#87CEFA")
            plt.plot(ts, np.array(Qs)[:,2][:,0], label = "$Q_{}$".format(2), c = "#87CEEB")
            plt.plot(ts, np.array(Qs)[:,3][:,0], label = "$Q_{}$".format(3), c = "#00BFFF")
                # angle
            plt.plot(ts, np.array(Qs)[:,0][:,0], label = "$Q_{}$".format(0), c = 'r')

                # Format plot
            plt.xticks(rotation=45, ha='right')
            plt.title('Attitude quaternion and magnetic torque over time')
            plt.ylabel('Attitude')
            plt.legend()
            plt.ylim((-1.05,1.05))
            plt.xlim(right = max(30, ts[-1])+int(nbr_secondes))

            ### Draw W
            plt.subplot(2,2,2)

            plt.title('Speed and magnetic field over time')
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


            ### Draw C=MxB
            plt.subplot(2,2,3)

            plt.plot(ts, np.array(Cs)[:,0][:,0], label = "$C_{}$".format(0), c = "#87CEFA")
            plt.plot(ts, np.array(Cs)[:,1][:,0], label = "$C_{}$".format(1), c = "#87CEEB")
            plt.plot(ts, np.array(Cs)[:,2][:,0], label = "$C_{}$".format(2), c = "#00BFFF")

            plt.plot(ts, np.array(Cnorms), label = "$|C|$", c = 'r')
                # Format plot
            plt.xticks(rotation=45, ha='right')
            plt.ylabel('Torque')
            plt.xlabel('Time (s)')
            plt.legend()
            plt.ylim((-Cmax*1.05,Cmax*1.05))
            plt.xlim(right = max(30, ts[-1])+int(nbr_secondes))

            #Draw B
            plt.subplot(2,2,4)

            plt.plot(ts, np.array(Bs)[:,0][:,0], label = "$B_{}$".format(0), c = "#87CEFA")
            plt.plot(ts, np.array(Bs)[:,1][:,0], label = "$B_{}$".format(1), c = "#87CEEB")
            plt.plot(ts, np.array(Bs)[:,2][:,0], label = "$B_{}$".format(2), c = "#00BFFF")


            plt.plot(ts, np.array(Bnorms), label = "$|B|$", c = 'r')
                # Format plot
            plt.xticks(rotation=45, ha='right')
            plt.ylabel('Magnetic field')
            plt.xlabel('Time (s)')
            plt.legend()
            plt.ylim((-Bmax*1.05,Bmax*1.05))
            plt.xlim(right = max(30, ts[-1])+int(nbr_secondes))

    except requests.exceptions.ConnectionError:
        print('[' + str(int(time.time())) + "] Did not reach server")
    except requests.exceptions.ReadTimeout:
        print('[' + str(int(time.time())) + "] Did not reach server")

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(ts, Qs), interval=1/dt)
plt.show()
