""" energy model for drone 
This energy model referenced this paper: 
    A power consumption model for multi-rotor small unmanned aircraft systems, https://doi.org/10.1109/ICUAS.2017.7991310
"""
import math

# constants used in model

m = 0.907         # weight of drone
g = 9.8           # gravitational acceleration

# parameters used in model - came from experiment results in paper

k_1 = 0.8554          
k_2 = 0.3051
k_3 = 1           
c_1 = k_1 / k_2    
c_2 = 0.3177
c_3 = 0           # assumed to have 0
c_4 = 0.0296
c_5 = 0.0279
c_6 = 0           # assumed to have 0


# alpha: angle of attack. must be radian
def thrust(alpha, V_air, m_payload):
    return math.sqrt((m + m_payload)*g - math.pow(c_5*math.pow(V_air*math.cos(alpha), 2), 2) + math.pow(c_4*math.pow(V_air, 2), 2))

# T: thrust, V_vert: vertical speed of drone
def P_induced(T, V_vert):
    # threshold could be changed
    if V_vert <= 0.1:
        return c_1 * math.pow(T, 3/2)
    else:
        return k_1 * T * ( (V_vert / 2) + math.sqrt( math.pow(V_vert / 2, 2) + (T / math.pow(k_2, 2)) ) )


# T: thrust, V_air: horizontal speed of drone. assume that there are no wind.
# alpha: angle of attack. must be radian
def P_profile(T, V_air, alpha):
    return c_2 * math.pow(T, 3/2) + c_3 * math.pow(V_air * math.cos(alpha), 2) * math.pow(T, 1/2)


# T: thrust, V_air: horizontal speed of drone. assume that there are no wind.
def P_parasite(V_air):
    return c_4 * math.pow(V_air, 3)


# old energy_consumption model
def energy_consumption(v_1, v_2, m_2, h_delta, x_delta):
    g = 9.8
    rho = 1.225
    m_1 = 0.905
    A_2 = 0.322 * 0.242
    A_1 = A_2 + 0.322 * 0.084
    C_D_1 = 0.8
    C_D_2 = 1.05

    W = ((m_1 + m_2)*g + (1/2)*rho*(math.pow(v_2,2))*A_2*C_D_2)*h_delta if h_delta > 0 else 0
    W += ((1/2)*rho*(math.pow(v_1, 2))*A_1*C_D_1)*x_delta

    return W