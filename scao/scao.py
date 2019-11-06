import numpy as np
from scao.quaternion import Quaternion
from math import acos, asin

class SCAO:

    def __init__(self, stabAlg, I, J, delta_t):
        """
        listQ: liste des attitudes dans Rr
        listL: liste des moments cinétiques dans Rr
        listB: liste des champs magnétiques calculées dans Rv
        Mrv : matrice de transfert de r -> v
        Mvr : inverse de Mvr
        P_d_mt: facteurs P du PID pour les magnetorquers pour le detumbling (matrice 3x3 diagonale)
        P_d_rw: facteurs P du PID pour les roues à inertie pour le détumbling (matrice 3x3 diagonale)
        P_s: facteur P du PID pour le controle d'attitude
        D_s: facteur D du PID pour le controle d'attitude
        """
        self.Q = None #exprimé dans Rr
        self.W = None #exprimé dans Rr
        self.B = None #exprimé dans Rr
        self.delta_t = delta_t
        self.I = I
        self.stabAlg = stabAlg

    def setAttitude(self, Q):
        """ Ajoute le quaternion actuel à la liste des attitudes."""
        self.Q = Quaternion(*Q[:,0])

    def setRotation(self, W):
        """ Ajoute le vecteur rotation actuel à la liste des rotations."""
        self.W = W

    def setMagneticField(self, B):
        """ Calcule B dans Rr et l'ajoute."""
        self.B = B

    def getCommand(self, Qt):
        Qt = Quaternion(*Qt[:,0])
        torque = self.stabAlg(self.Q,self.W,Qt,self.B,self.I)
        B_dir = np.dot(self.Q.tminv(),self.B)/np.linalg.norm(self.B)
        torque_perp = torque - np.dot(np.transpose(torque), B_dir)[0,0]*B_dir
        #M = -np.cross(torque_perp, B_dir, axisa=0,axisb=0,axisc=0)/np.linalg.norm(self.B) #"inversion" de C=MxB
        M = np.array([[0],[0],[0]]) #pour l'instant, pas de commande selon les roues
        dw = -torque
        return dw, M
