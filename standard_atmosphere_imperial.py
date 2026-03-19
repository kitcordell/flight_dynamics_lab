# Computes air density, temperature and pressure as a function of altitude with standard atmosphere conditions

def standard_atmosphere_imperial(alt):
    
    rho_0 = 0.0023769       # standard air density at sea level [slug/ft^3]
    T_0 = 518.67            # standard sea level temperature [R]
    L = 0.00356616          # standard lapse rate, [R/ft]
    g = 32.174              # acceleration of gravity, [ft/s^2]
    R = 1716.0              # ideal gas constant, # [ft*lbf/(slug*R)]
    p_0 = R * T_0 * rho_0


    rho = rho_0 * ((T_0 - L * alt) / T_0)**((g/(R*L))-1)    # air density at altitude
    T = T_0 - L * alt                                       # temperature at altitude
    p = p_0 * ((T)/T_0)**(g/(R*L))

    return rho, T, p

rho, T, p = standard_atmosphere_imperial(000) 

print(rho, T, p)