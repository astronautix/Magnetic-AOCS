import numpy as np
from scao.quaternion import Quaternion
from math import acos, cos, pi, sin

def PIDRW(P, D, dP): ## BUG: CHECK HERE FOR REF CONSISTENCY
    def res(Q,W,Qt,B,I):
        #Qt = targeted attitude /!\ N'est pas un objet Quaternion (mais un quaternion formel)
        #proportional term - the error is expressed in R_v
        Qr = Qt*Q.inv() #quaternion relatif qui effectue la rotation depuis Q vers Qt
        dynamicalP = P/(1+np.linalg.norm(W))**dP #dynamical P-factor
        error = np.dot(Q.tminv(),dynamicalP*Qr.angle()*Qr.axis())

        #derivative term
        error += np.dot(Q.tminv(),-D*W)

        #moment à appliquer
        torque = np.dot(I,error)

        return torque

    return res

def PIDMT(P, D):
    def res(Q,W,Qt,B,I):
        Qr = Q*Qt.inv()
        u = Q.R2V((P*Qr.axialPart()+D*W))
        Bb = Q.R2V(B)
        M = np.cross(u,Bb,axisa=0,axisb=0,axisc=0)
        return M
    return res
