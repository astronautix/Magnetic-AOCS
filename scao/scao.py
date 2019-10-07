import numpy as np
from scao.quaternion import Quaternion
from math import acos

class SCAO:

    def __init__(self, I, J, P, D, dP, delta_t):
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
        self.Q = [] #exprimé dans Rr
        self.W = [] #exprimé dans Rr
        self.B = [] #exprimé dans Rv /!\
        self.Mrv = None
        self.Mvr = None
        self.delta_t = delta_t
        self.P = P
        self.D = D
        self.dP = dP
        self.I = I

    def transferMatrix(self,Q): #
        """ Calculate transfer matrix from Rr to Rv, i.e. X_Rr = M * X_Rv """
        q0,q1,q2,q3 = Q[0][0],Q[1][0],Q[2][0],Q[3][0]
        return np.array([[2*(q0**2+q1**2)-1, 2*(q1*q2-q0*q3)  , 2*(q1*q3+q0*q2)  ],
                         [2*(q1*q2+q0*q3)  , 2*(q0**2+q2**2)-1, 2*(q2*q3-q0*q1)  ],
                         [2*(q1*q3-q0*q2)  , 2*(q2*q3+q0*q1)  , 2*(q0**2+q3**2)-1]])

    def setAttitude(self, Q):
        """ Ajoute le quaternion actuel à la liste des attitudes."""
        self.Q.append(Quaternion(*Q[:,0]))
        self.Mrv = self.transferMatrix(Q) #X_Rr = Mrv * X_Rv
        self.Mvr = np.linalg.inv(self.Mrv) #X_Rv = Mvr * X_Rr

    def setRotation(self, W):
        """ Ajoute le vecteur rotation actuel à la liste des rotations."""
        self.W.append(W)

    def setMagneticField(self, B):
        """ Calcule B dans Rv et l'ajoute."""
        self.B.append(np.dot(self.Mvr,B))

    def getTorque(self,Qt):
        #Qt = targeted attitude /!\ N'est pas un objet Quaternion (mais un quaternion formel)

        #proportional term - the error is expressed in R_r
        Qr = Quaternion(*Qt[:,0])*self.Q[-1].inv() #quaternion relatif qui effectue la rotation depuis Q vers Qt
        dynamicalP = self.P/(1+np.linalg.norm(self.W[-1]))**self.dP #dynamical P-factor
        error = np.dot(self.Mvr,-dynamicalP*Qr.angle()*Qr.axis())

        #derivative term
        error += np.dot(self.Mvr,self.D*self.W[-1])

        #moment à appliquer
        torque = np.dot(self.I,error)

        return torque


    def getCommand(self, Qt):
        torque = self.getTorque(Qt)
        B_dir = self.B[-1]/np.linalg.norm(self.B[-1])
        torque_perp = torque - np.dot(np.transpose(torque), B_dir)[0,0]*B_dir
        M = np.cross(torque_perp, B_dir, axisa=0,axisb=0,axisc=0)/np.linalg.norm(self.B[-1]) #"inversion" de C=MxB
        dw = np.array([[0],[0],[0]]) #pour l'instant, pas de commande selon les roues
        return dw, M
