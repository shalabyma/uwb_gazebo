import numpy as np
import rospy

# Speed of light
c = 299792458 # m/s

# TODO: move this to launch or params file
ts_to_ns = 1e9 * (1.0 / 499.2e6 / 128.0)

def convert_ns_to_ts(ns):
    """Converts nanoseconds to timestamp
    """
    return int(np.round((1e12 + ns) / ts_to_ns))

def convert_ts_to_ns(ts):
    """Converts timestamp to nanoseconds
    """
    return int(ts * ts_to_ns)

def compute_range_data(
    tag0: dict, # initiator 
    tag1: dict, # target
    del_t_32: float, # in nanoseconds
    del_t_53: float # in nanoseconds
):
    distance = compute_range(tag0['position'], tag1['position'])
    tof = distance / c * 1e9 # in nanoseconds

    rospy.loginfo("Distance: %f m, TOF: %f ns", distance, tof)
    
    # Need to prevent negative values and round to nearest integer
    tx1 = tag0['offset']
    rx1 = tag1['offset'] + tof
    tx2 = tag1['offset'] + tof + (1 + tag1['skew']) * del_t_32
    rx2 = tag0['offset'] + 2*tof + (1 + tag0['skew']) * del_t_32
    tx3 = tag1['offset'] + tof + (1 + tag1['skew']) * (del_t_32 + del_t_53)
    rx3 = tag0['offset'] + 2*tof + (1 + tag0['skew']) * (del_t_32 + del_t_53)
    
    output = {
        range: 0,
        'tx1': convert_ns_to_ts(tx1),
        'rx1': convert_ns_to_ts(rx1),
        'tx2': convert_ns_to_ts(tx2),
        'rx2': convert_ns_to_ts(rx2),
        'tx3': convert_ns_to_ts(tx3),
        'rx3': convert_ns_to_ts(rx3),
    }

    tx1 = convert_ts_to_ns(output['tx1'])
    rx1 = convert_ts_to_ns(output['rx1'])
    tx2 = convert_ts_to_ns(output['tx2'])
    rx2 = convert_ts_to_ns(output['rx2'])
    tx3 = convert_ts_to_ns(output['tx3'])
    rx3 = convert_ts_to_ns(output['rx3'])
    # Shalaby's double-sided two-way ranging protocol
    output['range'] = 0.5 * c / 1e9 * (
        (rx2 - tx1) - (rx3 - rx2) / (tx3 - tx2) * (tx2 - rx1)
    ) # in meters

    # rospy.loginfo("tx1: %f, rx1: %f, tx2: %f, rx2: %f, tx3: %f, rx3: %f", tx1, rx1, tx2, rx2, tx3, rx3)
    
    return output

def compute_passive_data(
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

    rx1 = tag2['offset'] + tof_to_initiator
    rx2 = tag2['offset'] + tof_range + tof_to_target \
            + (1 + tag2['skew']) * del_t_32
    rx3 = tag2['offset'] + tof_range + tof_to_target \
            + (1 + tag2['skew']) * (del_t_32 + del_t_53)
    
    return {
        'rx1': convert_ns_to_ts(rx1),
        'rx2': convert_ns_to_ts(rx2),
        'rx3': convert_ns_to_ts(rx3),
    }
    
    
def is_in_range(pos1, pos2, range):
    # TODO: In the future, include obstacles when deciding if tags within range
    return compute_range(pos1, pos2) <= range

def compute_range(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)