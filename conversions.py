import numpy as np

def kts2fps (kts):
    fps = kts * 1.68781 # convert knots to feet per second
    return fps

def fps2kts (fps):
    fps = fps / 1.68781 # convert feet per second to knots

    