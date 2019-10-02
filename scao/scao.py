import numpy as np
from scao.quaternion import Quaternion
from math import acos

class SCAO:

    def __init__(self, I, J, P_DetumblingMagnetic, P_DetumblingWheel, P_StabMagnetic, P_StabWheel, D_StabMagnetic, D_StabWheel, delta_t):
        """
        listQ: liste des attitudes dans Rr
        listL: liste des moments cinétiques dans Rr
        listB: liste des champs magnétiques calculées dans Rv
        Mrv : matrice de transfert de r -> v
        Mvr : inverse de Mvr
        P_d_mt: facteur P du PID pour les magnetorquers
        P_d_rw: facteur P du PID pour les roues à inertie
        """
        self.Q = [] #exprimé dans Rr
        self.W = [] #exprimé dans Rr
        self.B = [] #exprimé dans Rv /!\
        self.Mrv = None
        self.Mvr = None
        self.P_d_mt = P_DetumblingMagnetic
        self.P_d_rw = P_DetumblingWheel
        self.delta_t = delta_t

        self.P_s_rw = P_StabWheel
        self.D_s_rw = D_StabWheel

    def transferMatrix(self,Q): #
        """ Calculate transfer matrix from Rr to Rv, i.e. X_Rr = M * X_Rv """
        q0,q1,q2,q3 = Q[0][0],Q[1][0],Q[2][0],Q[3][0]
        return np.array([[2*(q0**2+q1**2)-1, 2*(q1*q2-q0*q3)  , 2*(q1*q3+q0*q2)  ],
                         [2*(q1*q2+q0*q3)  , 2*(q0**2+q2**2)-1, 2*(q2*q3-q0*q1)  ],
                         [2*(q1*q3-q0*q2)  , 2*(q2*q3+q0*q1)  , 2*(q0**2+q3**2)-1]])

    def setAttitude(self, Q):
        """ Ajoute le quaternion actuel à la liste des attitudes."""
        self.Q.append(Q)
        self.Mrv = self.transferMatrix(Q) #X_Rr = Mrv * X_Rv
        self.Mvr = np.linalg.inv(self.Mrv) #X_Rv = Mvr * X_Rr

    def setRotation(self, W):
        """ Ajoute le vecteur rotation actuel à la liste des rotations."""
        self.W.append(W)

    def setMagneticField(self, B):
        """ Calcule B dans Rv et l'ajoute."""
        self.B.append(np.dot(self.Mvr,B))

    def getCommandDetumblingMagnetic(self):
        """ Détumbling magnétique si accès à la donnée W via les instruments de mesure."""
        return self.P_d_mt*np.cross(np.dot(self.Mvr,self.W[-1]), self.B[-1], axisa=0, axisb=0,axisc=0)

    def getCommandDetumblingMagneticBDot(self):
        """ Détumbing magnétique si on connait uniquement le champ B instantanné.
            On utilise le fait que WxB = dB/dt (B-dot algorithm)"""
        dB = (self.B[-1]-self.B[-2])/self.delta_t
        return -self.P_d_mt*dB

    def getCommandDetumblingWheel(self):
        """ Détumbling mécanique si accès à W."""
        return self.P_d_rw*(np.dot(self.Mvr,self.W))

    def getCommandStabWheel(self,Qt):
        #Qt = targeted attitude /!\ N'est pas un objet quaternion (mais un quaternion formel)
        #proportional term
        Qr = Quaternion(*Qt[:,0])*Quaternion(*self.Q[-1][:,0]).inv() #quaternion relatif qui effectue la rotation depuis Q vers Qt
        Qr = Qr.vec()
        r = np.array([Qr[1:,0]])
        r = np.transpose(r/np.linalg.norm(r)) #r est le vecteur autour duquel il faut tourner : axe du couple à appliquer!
        e = acos(max(-1,min(Qr[0,0],1)))*2 #angle de rotation à effectuer : correspond à l'erreur!
        a = np.dot(self.Mvr,np.dot(self.P_s_rw, -e*r))

        #derivative term
        W = self.W[-1]
        dr = W/np.linalg.norm(W)
        de = np.linalg.norm(W)
        a += np.dot(self.Mvr,np.dot(self.D_s_rw, de*dr))

        return a
