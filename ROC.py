import numpy as np

ROC = (P_req - P_avail) / W * 60 # rate of climb [ft/min]

P_req = D * V # power required [ft*lb/s]
P_avail = T_avail * V # power available [ft*lb/s]

