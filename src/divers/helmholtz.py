# -*- coding: utf-8 -*-
"""
Created on Wed Jan 29 15:29:21 2020

@author: Fetnat III
"""

from math import sqrt, pi
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

U = 11.5
r = 1e-3
a = 0.69 #0.69, 0.63, 0.57
sigma = 59.6e6
mu_0 = pi*4e-7
n = 40
#I = U*sigma*(r**2)/(2*a*n)
I = 3
seuil = 99/100

alpha = mu_0/2*n*I*a**2


f = lambda x : alpha*(((x+a/2)**2+a**2)**(-3/2) + ((x-a/2)**2+a**2)**(-3/2))

X = np.linspace(-0.69, 0.69, 100)
Y = f(X)
Y = Y/max(Y)

plt.plot(X,Y, label="$B/B_{max}(x)$")
plt.plot([-0.7, 0.7], [max(Y)*seuil, max(Y)*seuil], label = "$y = 0.99$")
plt.grid()
plt.xlabel("$x $(m)")
plt.ylabel("$B/B_{max}$")
plt.legend()
plt.xlim(-0.7, 0.7)
plt.ylim(min(Y)*(1-5/100),max(Y)+5*max(Y)/100)
plt.show()


#g = lambda a : fsolve( lambda y : alpha*(((y+a/2)**2+a**2)**(-3/2) + ((y-a/2)**2+a**2)**(-3/2)) - alpha*(((a/2)**2+a**2)**(-3/2) + ((a/2)**2+a**2)**(-3/2))*seuil, 0)[0]
#
#R = np.linspace(0.1,1,10)
#g_R = []
#for r in R:
#    g_R.append(g(r))
#plt.plot(R, g_R)
#plt.show()
