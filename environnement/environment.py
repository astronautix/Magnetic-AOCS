import sys
sys.path.append('..')
from math import *
import numpy as np
from random import *


class Environment:

    """
    Calcul du champ magnétique à la position du véhicule.

    """

    # mu_e = 7.7425 #T.km^(3)

    def __init__(self, mu_e = 7.7425, delta=9.35*pi/180):
        """
        r : radius, expressed in km
        i : inclination of the orbit
        u : argument of periapsis + true anomaly
        delta : magnetic pole drift
        k : unit vector supporting the dipole vector
        """
        self.mu_e = mu_e
        self.r = None
        self.i = None
        self.u = None
        self.delta = delta
        self.k = np.array([[sin(self.delta)],[0],[cos(self.delta)]])


    def setPosition(self, position):
        """
        Takes the tuple (r,i,u) as an argument.
        """
        self.r = position[0]
        self.i = position[1]
        self.u = position[2]

    def dipoleField(self, pos, dist):
        """
        Raw formula of a magnetic field created by a dipole.
        """
        return -self.mu_e/(dist**5)*(dist**2*self.k-3*np.dot(np.transpose(self.k),pos)*pos)

    def getPosVector(self):
        """
        Calculate the position in the inertial reference frame.
        """
        u = self.u
        i = self.i
        return self.r*np.array([[cos(u)],[-sin(u)*cos(i)],[sin(i)*sin(u)]]) #in the inertial frame of reference

    def getEnvironment(self):
        """
        Get the magnetic field value at the current position of the satellite (expressed in the intertial frame of reference).
        """
        return self.dipoleField(self.getPosVector(), self.r)
