from helmoltz import *
import time
from math import sin
#on crée les objets représentant chaque générateurs en indiquant leur ports USB
#sur linux on les découvre avec la commande 'ls /dev/ttyUSB*'
gen1 = GPD3303S("/dev/ttyUSB0")
gen2 = GPD3303S("/dev/ttyUSB1")

#on définit la tension des générateurs suffisemment haut pour ne pas être régulé en tension
gen1.V_ch1 = Q_("30V")
gen1.V_ch2 = Q_("30V")
gen2.V_ch1 = Q_("30V")

#mettre ces variable à 0 revient à éteindre l'output, ici donc on les allume!
gen1.out = 1
gen2.out = 1

#on définit l'instance de HelmholtzCoils3D qui représente notre système de bobines
helmholtz_coil_lpp=HelmholtzCoils3D(
    HelmholtzCoil(Q_("0.0183A/uT")),
    HelmholtzCoil(Q_("0.0158A/uT")),
    HelmholtzCoil(Q_("0.0176A/uT"))
)

#on définit ici quels channels de quels générateurs
gen3D=Generator3D(
    lambda I: gen1.set_I(1,I),
    lambda I: gen1.set_I(2,I),
    lambda I: gen2.set_I(1,I)
)

#on définit alors le champ que l'on veut dans les bobines en indiquant:
# - quelles sont les caractéistiques des bobines
# - quels channels controlent quelles bobines
while 1:

    generate_B(
        Q_(str(150*sin(2*3.14*.1*time.time()))+"uT"),
        Q_(str(150*sin(2*3.14*.13*time.time()))+"uT"),
        Q_(str(150*sin(2*3.14*.12*time.time()))+"uT"),
        helmholtz_coil_lpp,
        gen3D
    )
    time.sleep(1)
