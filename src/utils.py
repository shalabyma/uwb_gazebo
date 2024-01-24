import numpy as np

def is_in_range(pos1, pos2, range):
    # TODO: In the future, include obstacles when deciding if tags within range
    return np.linalg.norm(pos1 - pos2) < range