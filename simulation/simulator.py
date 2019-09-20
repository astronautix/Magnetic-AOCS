import numpy as np

class Simulator:

    def __init__(self,dt,L0,Q0 = np.array([[1.],[0.],[0.],[0.]])):
        self.t = 0 #temps écoulé
        self.dt = dt
        self.Q = Q0 #quaternion de rotation de Rv par rapport à Rr
        self.L = L0 #Moment cinétique du satellite dans Rr

    def transferMatrix(self): #transfer matrix from Rr to Rv i.e. X_Rr = M * X_Rv
        q0,q1,q2,q3 = self.Q[0][0],self.Q[1][0],self.Q[2][0],self.Q[3][0]
        return np.array([[2*(q0**2+q1**2)-1, 2*(q1*q2-q0*q3)  , 2*(q1*q3+q0*q2)  ],
                         [2*(q1*q2+q0*q3)  , 2*(q0**2+q2**2)-1, 2*(q2*q3-q0*q1)  ],
                         [2*(q1*q3-q0*q2)  , 2*(q2*q3+q0*q1)  , 2*(q0**2+q3**2)-1]])

    def dQ(self,W): #renvoie la dérivée du quaternion
        qw,qx,qy,qz = self.Q[0][0],self.Q[1][0],self.Q[2][0],self.Q[3][0]
        expQ = np.array([[-qx, -qy, -qz],
                         [ qw,  qz, -qy],
                         [-qz,  qw,  qx],
                         [ qy, -qx,  qw]])
        return 1/2*np.dot(expQ,W)

    def dL(self,M,dw,J,B): #renvoie la dérivée du moment cinétique avec le th du moment cinétique
        C_Rv = np.cross(M, np.dot(np.linalg.inv(self.transferMatrix()), B), axisa=0, axisb=0,axisc=0) - J*dw #couple dans Rv du au MC et aux RW
        C_Rr = np.dot(self.transferMatrix(), C_Rv)
        return C_Rr

    def getNextIteration(self,M,dw,J,B,I):
        # M : moment magnétique des MC exprimé dans Rv
        # dw : vecteur accélération angulaire des RI exprimé dans Rv
        # J : Moment d'inertie des RI
        # I : Tenseur d'inertie du satellite exprimé dans Rv
        # B : Vecteur champ magnétique environnant, exprimé dans Rr
        self.L += self.dL(M,dw,J,B)*self.dt #calcul du nouveau moment cinétique
        W = np.dot(np.linalg.inv(np.dot(self.transferMatrix(),I)),self.L) #Vecteur rotation du satellite dans Rr
        self.Q += self.dQ(W)*self.dt #calcul de la nouvelle orientation
        self.Q /= np.linalg.norm(self.Q)
        self.t += self.dt
        return W
