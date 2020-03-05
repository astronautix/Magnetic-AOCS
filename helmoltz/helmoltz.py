import serial
import time
from pint import UnitRegistry
ureg = UnitRegistry()
Q_ = ureg.Quantity

class GPD3303S:
    def __init__(self,dev):
        self._dev = serial.Serial(dev, 9600, timeout=1)

    def _read(self):
        return self._dev.readline().decode().strip()

    def set_I(self, chan, current):
        if isinstance(current,Q_):
            I = current.to_base_units()
            if I.u==ureg.Unit('A'):
                self._dev.write(f"ISET{chan}:{I.m:.3f}\n".encode())
        else:
            self._dev.write(f"ISET{chan}:{current}\n".encode())

    def _get_I(self, chan):
        self._dev.write(f"IOUT{chan}?\n".encode())
        return Q_(self._read())

    def set_V(self, chan, voltage):
        def is_volt(V):
            return V.u== ((1.*ureg.Unit('V')).to_base_units()).u
        if isinstance(voltage,Q_):
            V = voltage.to_base_units()
            if is_volt(V):
                self._dev.write(f"VSET{chan}:{V.m:.3f}\n".encode())
        else:
            self._dev.write(f"VSET{chan}:{voltage}\n".encode())

    def _get_V(self, chan):
        self._dev.write(f"VOUT{chan}?\n".encode())
        return Q_(self._read())

    @property
    def out(self):
        self._dev.write(f"STATUS?\n".encode())
        return int(self._read().split()[6])

    @out.setter
    def out(self,on_off):
        self._dev.write(f"OUT{on_off}\n".encode())

    @property
    def I_ch1(self):
        return self._get_I(1)

    @I_ch1.setter
    def I_ch1(self,current):
        self.set_I(1,current)

    @property
    def I_ch2(self):
        return self._get_I(2)

    @I_ch2.setter
    def I_ch2(self,current):
        self.set_I(2,current)

    @property
    def V_ch1(self):
        return self._get_V(1)

    @V_ch1.setter
    def V_ch1(self,volatge):
        self.set_V(1,volatge)

    @property
    def V_ch2(self):
        return self._get_V(2)

    @V_ch2.setter
    def V_ch2(self,volatge):
        self.set_V(2,volatge)

class HelmholtzCoil:
    def __init__(self,k):
        self.k = k
    def calibrate(self,B):
        return  self.k * B

class FakeHelmholtzCoil:
    def __init__(self):
        pass
    def calibrate(self,B):
        return  0.

class HelmholtzCoils3D:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

    def calibrate(self,Bx, By, Bz):
        return self.x.calibrate(Bx),self.y.calibrate(By),self.z.calibrate(Bz)

class Generator3D:
    def __init__(self,gen_x_func,gen_y_func,gen_z_func):
        self.gen_x=gen_x_func
        self.gen_y=gen_y_func
        self.gen_z=gen_z_func

    def generate(self, Ix, Iy ,Iz):
        self.gen_x(Ix)
        self.gen_y(Iy)
        self.gen_z(Iz)

def generate_B(Bx, By, Bz, coil, generator):
    Ix, Iy ,Iz = coil.calibrate(Bx, By, Bz)
    generator.generate(Ix, Iy ,Iz)
