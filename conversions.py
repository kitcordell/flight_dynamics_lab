import numpy as np

def kts2fps (kts):
    fps = kts * 1.68781 # convert knots to feet per second
    return fps

def fps2kts (fps):
    fps = fps / 1.68781 # convert feet per second to knots

def hp2ftlbfps (hp):
    ftlbfps = hp * 550.0 # convert horsepower to foot-pounds per second
    return ftlbfps