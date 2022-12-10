import numpy as np

class Params(object):
    """
    Setting up params for computation Bat algorithm

    """
    def __init__(self) -> None:
        # dimensions
        self.dim = 6

        # bound params
        self.dodParam = [-np.pi, np.pi]

        # runs
        self.nRuns = 1000

        # generations
        self.maxGener = 100

        # population size
        self.NP = 75

        # loudness
        self.pA = 0.9

        # pulse rate
        self.pR = 0.3

        # frequency qmin - qmax
        self.pF = [-np.pi, np.pi]