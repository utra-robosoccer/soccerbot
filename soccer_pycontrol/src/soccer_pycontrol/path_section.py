import functools
import math
from soccer_pycontrol.transformation import Transformation
import numpy as np
from scipy.special import comb
import matplotlib.pyplot as plt
from copy import deepcopy

class PathSection:
    def __init__(self, start_transform: Transformation,  end_transform: Transformation):
        self.start_transform = start_transform
        self.end_transform = end_transform
        pass