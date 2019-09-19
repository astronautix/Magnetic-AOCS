import numpy as np

class Simulator:

    def __init__(self,dt,L0):
        self.t = 0
        self.dt = dt
        self.Q = np.array([[0.],[0.],[0.],[0.]])
        self.L = L0

    def transferMatrix(self): #transfer matrix from Rr to Rv i.e. X_Rr = M * X_Rv
        q0,q1,q2,q3 = self.Q[0][0],self.Q[1][0],self.Q[2][0],self.Q[3][0]
        return np.array([[2*(q0**2+q1**2)-1, 2*(q1*q2-q0*q3)  , 2*(q1*q3+q0*q2)  ],
                         [2*(q1*q2+q0*q3)  , 2*(q0**2+q2**2)-1, 2*(q2*q3-q0*q1)  ],
                         [2*(q1*q3-q0*q2)  , 2*(q2*q3+q0*q1)  , 2*(q0**2+q3**2)-1]])

    def dL(self,M,dw,J,B):
        C_Rv = np.transpose(np.cross(np.transpose(M), np.transpose(np.dot(np.linalg.inv(self.transferMatrix()), B)))) - J*dw #couple dans Rv du au MC et aux RW
        C_Rr = np.dot(self.transferMatrix(), C_Rv)
        return C_Rr

    def getNextIteration(self,M,dw,J,B,I):
        # M : moment magnétique des MC exprimé dans Rv
        # dw : vecteur accélération angulaire des RI exprimé dans Rv
        # J : Moment d'inertie des RI
        # I : Tenseur d'inertie du satellite exprimé dans Rv
        # B : Vecteur champ magnétique environnant, exprimé dans Rr
        self.L += self.dL(M,dw,J,B)*self.dt
        W = np.dot(np.linalg.inv(np.dot(self.transferMatrix(),I)),self.L)
        self.Q += 1/2*np.dot(np.transpose(np.concatenate((np.array([[0]]), W), axis = 0)),self.Q)*self.dt #cf dérivée quaternions
        self.t += self.dt
        return W

sim = Simulator(0.02,np.array([[1.],[1.],[1.]]))
I = np.eye(3)
dw = np.zeros((3,1))
M = np.zeros((3,1))
J = 1
B = np.array([[0.],[0.],[1.]])
print(sim.getNextIteration(M,dw,J,B,I))
M = np.array([[1.],[0.],[0.]])
print(sim.getNextIteration(M,dw,J,B,I))
print(sim.getNextIteration(M,dw,J,B,I))
print(sim.getNextIteration(M,dw,J,B,I))
