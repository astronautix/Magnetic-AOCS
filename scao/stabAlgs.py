import numpy as np
from scao.quaternion import Quaternion

def PID(P, D, dP):
    def res(Q,W,Qt,B,I):
        #Qt = targeted attitude /!\ N'est pas un objet Quaternion (mais un quaternion formel)
        #proportional term - the error is expressed in R_r
        Qr = Qt*Q.inv() #quaternion relatif qui effectue la rotation depuis Q vers Qt
        dynamicalP = P/(1+np.linalg.norm(W))**dP #dynamical P-factor
        error = np.dot(Q.tminv(),dynamicalP*Qr.angle()*Qr.axis())

        #derivative term
        error += np.dot(Q.tminv(),-D*W)

        #moment à appliquer
        torque = np.dot(I,error)

        return torque

    return res

def PIDMag(P, D, dP):
    def res(Q,W,Qt,B,I):
        #Qt = targeted attitude /!\ N'est pas un objet Quaternion (mais un quaternion formel)
        #proportional term - the error is expressed in R_r
        Tv = np.dot(Qt.tminv(),B)
        Tr = np.dot(Q.tm(),Tv)

        dir = np.cross(Tr,B,axisa=0,axisb=0,axisc=0)/np.linalg.norm(B)**2
        dir = dir/np.linalg.norm(dir)*asin(np.linalg.norm(dir))
        error = np.dot(Q.tminv(),P*dir)

        #derivative term
        error += np.dot(Q.tminv(),D*W)

        #moment à appliquer
        torque = -np.dot(I,error)

        return torque #dans le refrentiel Rv

    return res
