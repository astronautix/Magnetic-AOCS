import numpy as np


class Magnetorquers:

    def __init__(self, n, A, M_max):
        self.n = n
        self.A = A
        self.M_max = M_max

    def currentConsumption(self, M):
        return M/(self.n*self.A) #most basic relation - can be improved

    def getMoment(self, M_target):
        """
        Return the real momentum and the current required to do it.
        """
        M_t = np.linalg.norm(M_target)
        if M_t > self.M_max:
            M_target = M_target*self.M_max/M_t #"normalization"
        return M_target, int(sum(list(map(lambda x : self.currentConsumption(x), M_target))))
