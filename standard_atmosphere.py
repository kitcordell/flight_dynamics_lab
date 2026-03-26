# Computes air density, temperature and pressure as a function of altitude with standard atmosphere conditions
import constants

def standard_atmosphere(alt):
    
    rho_0 = constants.rho_0       # standard air density at sea level [slug/ft^3]
    T_0 = constants.T_0            # standard sea level temperature [R]
    L = constants.L          # standard lapse rate, [R/ft]
    g = constants.g              # acceleration of gravity, [ft/s^2]
    R = constants.R              # ideal gas constant, # [ft*lbf/(slug*R)]
    p_0 = constants.p_0


    rho = rho_0 * ((T_0 - L * alt) / T_0)**((g/(R*L))-1)    # air density at altitude
    T = T_0 - L * alt                                       # temperature at altitude
    p = p_0 * ((T)/T_0)**(g/(R*L))

    return rho, T, p

rho, T, p = standard_atmosphere(000) 

print(rho, T, p)