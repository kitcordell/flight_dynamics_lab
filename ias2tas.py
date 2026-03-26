import numpy as np

def ias2tas(ias, alt):
    standard_atmosphere_imperial(alt)
    rho, T, p = standard_atmosphere_imperial(alt)
    