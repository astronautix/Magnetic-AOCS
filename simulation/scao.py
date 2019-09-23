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
        self.Q = None
        self.L = None
        self.B = None
        self.P = P
        self.D = D

    def checkSets(self):
        if self.Q is None or self.L is None:
            raise RuntimeError("setAttitude or setRotation have not been called")
        return

    def setAttitude(self, Q): #Ajoute le quaternion actuel à la liste des attitudes.
        self.Q = Q

    def setRotation(self, W):
        self.W = W

    def setMagneticField(self, B):
        self.B = B

    def getCommandDetumbling(self):
        self.checkSets()
        # TODO
