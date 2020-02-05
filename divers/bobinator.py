# -*- coding: utf-8 -*-
"""
Created on Wed Feb  5 14:28:35 2020

@author: Fetnat III
"""


from math import *

mu_0 = pi*4e-7
sigma = 59.6e6
V = 0.6
r= 0.0075
n= 500


s = pi*(125e-6)**2
L = 2*pi*r*n
A = pi*r**2
R = L/(sigma*s)
I = V/R
M = n*I*A
H = n**2*mu_0*A/L





