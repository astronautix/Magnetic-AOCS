import vpython as vp
from random import *
from math import *

ux = vp.vector(1,0,0)
uy = vp.vector(0,1,0)
uz = vp.vector(0,0,1)
# trièdre (z,x,y)

axe_x = vp.arrow(pos=vp.vector(0,0,0), axis=10*ux, shaftwidth=0.1)
axe_y = vp.arrow(pos=vp.vector(0,0,0), axis=10*uy, shaftwidth=0.1)
axe_z = vp.arrow(pos=vp.vector(0,0,0), axis=10*uz, shaftwidth=0.1)


wx, wy, wz = (1,1,1)
dt = 1/50

sugarbox = vp.box(pos=vp.vector(10,10,10), size=vp.vector(5,5,5), axis=vp.vector(0,0,0), up = uy)

while True:
        # sugarbox.rotate(angle=wx*dt, axis=ux)
        sugarbox.rotate(angle=wy*dt, axis=uy)
        # sugarbox.rotate(angle=wz*dt, axis=uz)
        vp.rate(1/dt)

"""
for wx,wy,wz in getAngularVelocity():
    # Rotate: rotation de tout l'objet autour de la droite de vecteur directeur <axis> et passant par <origin>)
    sugarbox.rotate(angle=wx*dt, axis=vp.vector(1,0,0))
    sugarbox.rotate(angle=wx*dt, axis=vp.vector(1,0,0))
    sugarbox.rotate(angle=wx*dt, axis=vp.vector(1,0,0))
    # Rate : réalise 50 fois la boucle par seconde
    vp.rate(1/dt)
"""
