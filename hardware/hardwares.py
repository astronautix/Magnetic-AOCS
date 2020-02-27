from hardware.magnetorquer import Magnetorquer
import numpy as np

class Hardware:


    def __init__(self, mgt_parameters, mgt_model):
        self.magnetorquers = [Magnetorquer(mgt_model, mgt_parameters), Magnetorquer(mgt_model, mgt_parameters), Magnetorquer(mgt_model, mgt_parameters)]



    def getRealCommand(self, dw, M): #M is a np.array

        Ux, Mx, s_x = self.magnetorquers[0].getRealCommand(M[0][0])
        Uy, My, s_y = self.magnetorquers[1].getRealCommand(M[1][0])
        Uz, Mz, s_z = self.magnetorquers[2].getRealCommand(M[2][0])
        if s_x or s_y or s_z:
            #checking if one of the coil would be saturated, and if so, recomputing commands
            M_norm = np.linalg.norm(M)
            M_min  = min(list(map((lambda x : x.getMmax), self.magnetorquers)))
            M_new = M_min*M/M_norm
            Ux, Mx, _ = self.magnetorquers[0].getRealCommand(M_new[0][0])
            Uy, My, _ = self.magnetorquers[1].getRealCommand(M_new[1][0])
            Uz, Mz, _ = self.magnetorquers[2].getRealCommand(M_new[2][0])
        return np.array([[Ux],[Uy],[Uz]]), np.array([[Mx],[My],[Mz]])
