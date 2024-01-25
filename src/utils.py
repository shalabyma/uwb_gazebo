import numpy as np

# Speed of light
c = 299792458 # m/s

def compute_range_data(
    time: float, # in nanoseconds
    tag0: dict, # initiator 
    tag1: dict, # target
    del_t_32: float, # in nanoseconds
    del_t_53: float # in nanoseconds
):
    distance = compute_range(tag0['position'], tag1['position'])
    tof = distance / 299792458 * 1e9 # in nanoseconds
    
    tx1 = time + tag0['offset']
    rx1 = time + tag1['offset'] + tof
    tx2 = time + tag1['offset'] + tof + (1 + tag1['skew']) * del_t_32
    rx2 = time + tag0['offset'] + 2*tof + (1 + tag0['skew']) * del_t_32
    tx3 = time + tag1['offset'] + tof + (1 + tag1['skew']) * (del_t_32 + del_t_53)
    rx3 = time + tag0['offset'] + 2*tof + (1 + tag0['skew']) * (del_t_32 + del_t_53)
    
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

def compute_passive_data(
    time: float, # in nanoseconds
    tag0: dict, # initiator
    tag1: dict, # target
    tag2: dict, # passive
    del_t_32: float, # in nanoseconds
    del_t_53: float # in nanoseconds
):
    distance_range = compute_range(tag0['position'], tag1['position'])
    distance_to_initiator = compute_range(tag2['position'], tag0['position'])
    distance_to_target = compute_range(tag2['position'], tag1['position'])

    tof_range = distance_range / 299792458 * 1e9 # in nanoseconds
    tof_to_initiator = distance_to_initiator / 299792458 * 1e9 # in nanoseconds
    tof_to_target = distance_to_target / 299792458 * 1e9 # in nanoseconds

    rx1 = time + tag2['offset'] + tof_to_initiator
    rx2 = time + tag2['offset'] + tof_range + tof_to_target \
            + (1 + tag2['skew']) * del_t_32
    rx3 = time + tag2['offset'] + tof_range + tof_to_target \
            + (1 + tag2['skew']) * (del_t_32 + del_t_53)
    
    return {
        'rx1': rx1,
        'rx2': rx2,
        'rx3': rx3,
    }
    
    
def is_in_range(pos1, pos2, range):
    # TODO: In the future, include obstacles when deciding if tags within range
    return compute_range(pos1, pos2) <= range

def compute_range(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)