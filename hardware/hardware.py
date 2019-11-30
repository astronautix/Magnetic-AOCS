from hardware.magnetorquer import Magnetorquers

class Hardware:


    def __init__(self, n, A, M_max):
         """
         n : number of coils windings;
         A : cross-sectional area of the coil
         """
         self.Magnetorquers = Magnetorquers(n, A, M_max)


    def getRealMoment(self, dw, M):
        return self.Magnetorquers.getMoment(M)
