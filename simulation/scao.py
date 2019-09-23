import numpy as np

class scao():

    def __init__(self, J, P, D):
        self.listQ = []
        self.listL = []
        self.J = J
        self.P = P
        self.D = D
        return

    def addAttitude(self, Q):
        self.listQ.append(Q)

    def setCommand(self):
        return None
