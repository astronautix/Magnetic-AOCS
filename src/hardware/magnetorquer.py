from hardware.customcoil import CustomCoil


class Magnetorquer:

    def __init__(self, model, parameters):
        self.parameters = parameters
        self.model = model
        self.functions = {'custom coil' : CustomCoil} #parameters for custom coil:  (r_coil, r_wire, n_coils, mu_rel, U_max)
        try:
            self.magnetorquer = self.functions[self.model](self.parameters)
        except ValueError:
            print("Les paramètres en entrée ne correspondent pas au modèle choisi!")

    def getRealCommand(self, M): #M is a scalar
        return self.magnetorquer.getRealCommand(M)

    def getMmax(self):
        return self.magnetorquer.M_max
