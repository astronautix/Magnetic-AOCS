import numpy as np

class SCAO:

    def __init__(self, I, J, P_DetumblingMagnetic, P_DetumblingWheel):
        """
        listQ: liste des attitudes
        listL: liste des moments cinétiques
        I: matrice d'inertie
        J: moment d'inertie des roues
        P: facteur P du PID
        D: facteur D du PID
        """
        self.Q = None
        self.W = None
        self.B = None
        self.P_d_mt = P_DetumblingMagnetic
        self.P_d_iw = P_DetumblingWheel

    def checkSets(self):
        if self.Q is None or self.W is None:
            raise RuntimeError("setAttitude or setRotation have not been called")
        return

    def setAttitude(self, Q): #Ajoute le quaternion actuel à la liste des attitudes.
        self.Q = Q

    def setRotation(self, W):
        #le vecteur rotation est mesuré dans R_v
        self.W = W

    def setMagneticField(self, B):
        #le champ est mesuré dans R_v
        self.B = B

    def getCommandDetumblingMagnetic(self):
        self.checkSets()
        return self.P_d_mt*np.cross(self.W, self.B, axisa=0, axisb=0,axisc=0)

    def getCommandDetumblingWheel(self):
        self.checkSets()
        return self.P_d_iw*(self.W)
