import numpy as np

class SCAO:

    def __init__(self, I, J, P_DetumblingMagnetic, P_DetumblingWheel, delta_t):
        """
        listQ: liste des attitudes
        listL: liste des moments cinétiques
        I: matrice d'inertie
        J: moment d'inertie des roues
        P_d_mt: facteur P du PID pour les magnetorquers
        P_d_iw: facteur P du PID pour les roues à inertie
        """
        self.Q = []
        self.W = []
        self.B = []
        self.P_d_mt = P_DetumblingMagnetic
        self.P_d_iw = P_DetumblingWheel
        self.delta_t = delta_t

    def checkSets(self):
        if not self.Q or not self.W : #not [] -> True
            raise RuntimeError("setAttitude or setRotation have not been called")
        return

    def setAttitude(self, Q): #Ajoute le quaternion actuel à la liste des attitudes.
        self.Q.append(Q)

    def setRotation(self, W):
        #le vecteur rotation est mesuré dans R_v
        self.W.append(W)

    def setMagneticField(self, B):
        #le champ est mesuré dans R_v
        self.B.append(B)

    def getCommandDetumblingMagnetic(self):
        self.checkSets()
        # En supposant que l'on a accès à W via les instruments de mesure
        # return self.P_d_mt*np.cross(self.W, self.B, axisa=0, axisb=0,axisc=0)

        #Autrement, on utilise le fait que WxB = dB/dt (B-dot algorithm)
        dB = (self.B[-1]-self.B[-2])/self.delta_t
        print("dB = " + str(dB))
        return -self.P_d_mt*dB

    def getCommandDetumblingWheel(self):
        self.checkSets()
        return self.P_d_iw*(self.W)
