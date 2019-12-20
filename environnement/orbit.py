import sys
sys.path.append('..')
from math import *
import numpy as np
from random import *
from scipy.optimize import fsolve

class Orbit:
    # Description purement géométrique de l'orbite pour le moment.

    # geocentric gravitationnal constant (in m^3.s^-2)
    # mu = 3.986004418e14

    """
    Contient les paramètres orbitaux de la mission.

    Permet de recevoir, à un temps donné, la position actuelle.

    Permet aussi de calculer les matrices de transferts pour différents référentiels.
    """


    def __init__(self, omega, i, e, r_p, mu=3.986004418e14, tau=0):
        """
        Omega: longitude/right ascension of the ascending node
        omega: argument of periapsis, measured from the ascending node
        i    : inclination between orbital and reference frame
        e    : eccentricity
        r_p  : periapsis, en m
        tau  : time when the satellite crosses the periapsis
        t    : time
        u : omega + theta, where theta the true eccentricity is
        mu : gravitationnal constant
        """
        self.mu = mu
        self.Omega = 0 #on considère pour le moment que le noeud ascendant est situé sur le 1er axe du réf. fixe
        self.omega = omega
        self.i = i
        self.e = e
        self.r_p = r_p
        self.t = None
        self.tau = tau
        self.a = self.r_p/(1-self.e)
        self.u = omega

    def setTime(self, t):
        """
        """
        self.t = t

    def radius(self,theta):
        """
        Get the distance to the primary focus.
        """
        return self.a*(1-self.e**2)/(1+self.e*cos(theta))

    def getPosition(self):
        """
        Return r, i and u.
        """
        n = sqrt(self.mu/self.a**3)
        M = n*(self.t-self.tau)
        #solving Kepler's equation to find the eccentric anomaly, with E_0 = M - sign(M)e
        E = fsolve(lambda x : x - self.e*sin(x) - M, M - np.sign(M)*self.e)
        theta = 2*atan(sqrt((1+self.e)/(1-self.e))*tan(E/2))
        self.u = self.omega+theta
        return self.radius(theta), self.i, self.u

    def getPeriod(self):
        return sqrt(self.a**3*4*pi**2/self.mu)

    #============================== transfer matrixes ==================================================
    def A_sx(self): #V_s = A_sx . V_x
        cu = cos(self.u)
        su = sin(self.u)
        return np.array([[cu, su, 0],[-su, cu, 0],[0, 0, 1]])

    def A_ys(self): #V_y = A_ys . V_s
        si = sin(self.i)
        ci = cos(self.i)
        return np.array([[1,0,0],[0,ci,si],[0,-si,ci]])

    def A_xs(self):
        return np.transpose(self.A_sx())

    def A_sy(self):
        return np.transpose(self.A_ys())
