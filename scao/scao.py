import numpy as np
from scao.quaternion import Quaternion
from math import acos, asin

class SCAO:

    def __init__(self, stabRW, stabMT, rwRatio, I, J, delta_t):
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
        self.I = I #exprimé dans Rv
        self.rwRatio = rwRatio  #ratio de reaction wheel
        self.stabRW = stabRW #retourne un vecteur dans Rv
        self.stabMT = stabMT #retourne un vecteur dans Rv

    def setAttitude(self, Q):
        """ Ajoute le quaternion actuel à la liste des attitudes."""
        self.Q = Q

    def setRotation(self, W):
        """ Ajoute le vecteur rotation actuel à la liste des rotations."""
        self.W = W

    def setMagneticField(self, B):
        """ Calcule B dans Rr et l'ajoute."""
        self.B = B

    def getCommand(self, Qt):
        Qt = Quaternion(*Qt[:,0])

        Bv = self.Q.R2V(self.B)
        torqueM = -(1-self.rwRatio)*self.stabMT(self.Q,self.W,Qt,self.B,self.I)
        B_dir = Bv/np.linalg.norm(Bv)
        M = np.cross(torqueM, B_dir, axisa=0,axisb=0,axisc=0)/np.linalg.norm(Bv) #"inversion" de C=MxB

        dw = -self.rwRatio*self.stabRW(self.Q,self.W,Qt,self.B,self.I)

        return dw, torqueM #dans Rv
