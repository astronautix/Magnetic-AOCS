U = 5
R = 10e-3
mur = 100
sigma = 1/17e-9
rfilchoisi = 250e-6

import matplotlib.pyplot as plt
import numpy as np

rfil = np.linspace(0.1,1,100)

M = lambda rf : 2*U*mur*sigma*4*3.14159*rf**2*R

N = lambda I : U*sigma*2*rfilchoisi**2/(R*I)

plt.plot(rfil, N(rfil))
plt.show()
