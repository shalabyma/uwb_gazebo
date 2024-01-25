import numpy as np

def compute_range_data(
    time: float, # in nanoseconds
    tag1: dict, 
    tag2: dict,
    del_t_32: float, # in nanoseconds
    del_t_53: float # in nanoseconds
):
    c = 299792458 # m/s
    distance = compute_range(tag1['position'], tag2['position'])
    tof = distance / 299792458 * 1e9 # in nanoseconds
    
    tx1 = time + tag1['offset']
    rx1 = time + tag2['offset'] + tof
    tx2 = time + tag2['offset'] + tof + tag2['skew'] * del_t_32
    rx2 = time + tag1['offset'] + 2*tof + tag1['skew'] * del_t_32
    tx3 = time + tag2['offset'] + tof + tag2['skew'] * del_t_53
    rx3 = time + tag1['offset'] + 2*tof + tag1['skew'] * del_t_53
    
    # Shalaby's double-sided two-way ranging protocol
    range = 0.5 * c / 1e9 * (
        (rx2 - tx1) - (rx3 - rx2) / (tx3 - tx2) * (tx2 - rx1)
    ) # in meters
    
    return {
        'range': range,
        'tx1': tx1,
        'rx1': rx1,
        'tx2': tx2,
        'rx2': rx2,
        'tx3': tx3,
        'rx3': rx3,
    }
    
    
def is_in_range(pos1, pos2, range):
    # TODO: In the future, include obstacles when deciding if tags within range
    return compute_range(pos1, pos2) <= range

def compute_range(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)