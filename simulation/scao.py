import numpy as np

class scao():

    def __init__(self, I, J, P, D):
        """
        listQ: liste des attitudes
        listL: liste des moments cinétiques
        I: matrice d'inertie
        J: moment d'inertie des roues
        P: facteur P du PID
        D: facteur D du PID
        """
        self.listQ = []
        self.listL = []
        self.J = J
        self.P = P
        self.D = D
        return

    def addAttitude(self, Q):
        """ Ajoute le quaternion actuel à la liste des attitudes.
        """
        self.listQ.append(Q)

    def setMagneticMoment(self, W, B):
        return self.K*np.cross(W,B, axisa=0, axisb=0,axisc=0))
