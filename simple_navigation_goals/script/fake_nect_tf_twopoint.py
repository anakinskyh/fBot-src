#!/usr/bin/env python
import tf
import numpy as np

class fake_nect_tf():
    def __init__(self,px,py,perc,parent,child):
        self.px = px
        self.py = py
        self.perc = perc
        self.parent = parent
        self.child = child

if __name__ == '__main__':
    px = np.array([0,0,0,0,0,0,0])
    py = np.array([0,0,0,0,0,0,0])

    perc = 0.1
