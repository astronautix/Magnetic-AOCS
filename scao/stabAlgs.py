import numpy as np
from scao.quaternion import Quaternion
from math import acos

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

def PIDMT(P, D, dP):
    def res(Q,W,Qt,B,I):
        #Qt = targeted attitude /!\ N'est pas un objet Quaternion (mais un quaternion formel)
        #proportional term - the error is expressed in R_v
        Tv = Qt.R2V(B)
        Tr = Q.V2R(Tv)

        dir = np.cross(Tr,B,axisa=0,axisb=0,axisc=0)
        angle = np.dot(np.transpose(Tr),B)
        angle = acos(np.linalg.norm(angle)/(np.linalg.norm(Tr)*np.linalg.norm(B)))
        dir = dir/np.linalg.norm(dir)*angle
        error = 0*Q.R2V(P*dir) #pour l'instant on ne contrôle que l'attitude

        #derivative term
        spareW = W - np.dot(np.transpose(W),B/np.linalg.norm(B))*B/np.linalg.norm(B)
        error += Q.R2V(D*spareW)

        #moment à appliquer
        torque = -np.dot(I,error)

        return torque #dans le refrentiel Rv

    return res
