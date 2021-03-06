# coding=utf-8
import numpy as np

from scao.quaternion import Quaternion


class Simulator:
    """
    Class simulator : compute the temporal evolution of the satellite

    """

    def __init__(self, dt, L0, Q0=Quaternion(1, 0, 0, 0)):
        self.t = 0  # temps écoulé
        self.dt = dt  # time increment
        self.Q = Q0  # quaternion de rotation de Rv par rapport à Rr
        self.L = L0  # Moment cinétique du satellite dans Rr

    def dQ(self, W):
        """
        renvoie la dérivée du quaternion
        """
        qw, qx, qy, qz = self.Q[0], self.Q[1], self.Q[2], self.Q[3]
        expQ = np.array([[-qx, -qy, -qz],
                         [qw, qz, -qy],
                         [-qz, qw, qx],
                         [qy, -qx, qw]])
        return 1 / 2 * np.dot(expQ, W)

    def dL(self, M, dw, J_wheel, B_r):
        """
         renvoie la dérivée du moment cinétique avec le th du moment cinétique

         M : le moment magnetique
         dw : la derivé du moment des roues
         J_wheel : le moment d'inerte d'une roue
         B_r : le champs magnetic local dans le repert de reference

        """
        C_Rv = np.cross(M, self.Q.R2V(B_r), axisa=0, axisb=0, axisc=0) - J_wheel * dw  # couple dans Rv du au MC et aux RW
        C_Rr = self.Q.V2R(C_Rv)
        return C_Rr

    def getNextIteration(self, M, dw, J, B, I):
        # M : moment magnétique des MC exprimé dans Rv
        # dw : vecteur accélération angulaire des RI exprimé dans Rv
        # J : Moment d'inertie des RI
        # I : Tenseur d'inertie du satellite exprimé dans Rv
        # B : Vecteur champ magnétique environnant, exprimé dans Rr
        self.L += self.dL(M, dw, J, B) * self.dt  # calcul du nouveau moment cinétique
        W = self.Q.V2R(np.dot(np.linalg.inv(I), self.Q.R2V(self.L)))  # Vecteur rotation du satellite dans Rr
        Qnump = self.Q.vec() + self.dQ(W) * self.dt  # calcul de la nouvelle orientation
        Qnump /= np.linalg.norm(Qnump)
        self.Q = Quaternion(*Qnump[:, 0])
        self.t += self.dt
        return W
