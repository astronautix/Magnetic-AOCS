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

def PIDMT(P, D, dP):
    def res(Q,W,Qt,B,I):
        #Qt = targeted attitude /!\ N'est pas un objet Quaternion (mais un quaternion formel)
        #proportional term - the error is expressed in R_v
        Tv = Qt.R2V(B)
        Tr = Q.V2R(Tv)

        angle = np.dot(np.transpose(Tr),B)
        angle = acos(np.linalg.norm(angle)/(np.linalg.norm(Tr)*np.linalg.norm(B)))
        dir = -np.cross(Tr,B,axisa=0,axisb=0,axisc=0)
        dir = dir/np.linalg.norm(dir)*angle

        error = 0*Q.R2V(P*dir) #pour l'instant on ne contrôle que l'attitude


        #derivative term
        spareW = W - np.dot(np.transpose(W),B/np.linalg.norm(B))*B/np.linalg.norm(B)
        error += Q.R2V(D*spareW)

        #moment à appliquer
        torque = -np.dot(I,error)

        return torque #dans le refrentiel Rv
    return res

def PIDMT2(e, kp, kv):
    def res(Q,W,Qt,B,I):
        Bdir = B/np.linalg.norm(B)
        med = Bdir + np.array([[1],[0],[0]])
        med = sin(pi/2)*med/np.linalg.norm(med)
        Q_tb = Quaternion(cos(pi/2), med[0][0], med[1][0], med[2][0])
        Qr = Qt*Q.inv()
        rotvec = Q_tb.R2V(Qr.axis())
        T_b = -(e**2*kp*rotvec+e*kv*np.dot(I,W))
        T_v = Q.R2V(Q_tb.V2R(T_b))
        return -T_v
    return res

def PIDMT3(e, kp, kv):
    def res(Q,W,Qt,B,I):
        Qr = Q*Qt.inv()
        u = -Q.R2V((e**2*kp*Qr.axis()+e*kv*W))
        Bb = Q.R2V(B)
        M = np.cross(Bb,u,axisa=0,axisb=0,axisc=0)
        return -M
    return res
