from math import pi

class CustomCoil:

    sigma = 59.6e6

    def __init__(self, parameters):
        (r_coil, r_wire, n_coils, mu_rel, U_max) = parameters
        self.r_coil = r_coil
        self.r_wire = r_wire
        self.n = n_coils
        self.mu = mu_rel
        self.U_max = U_max
        self.M_max = self.M(self.U_max)

    def M(self, U):
        return U/2*self.sigma*self.r_wire**2*self.mu*pi*self.r_coil

    def getRealCommand(self, M):
        """
        Return U, M and a boolean indicating if the coil is saturated or not.
        """
        if M>=self.M_max:
            return self.U_max, self.M_max, True
        else:
            return self.U_max*M/self.M_max, M, False
