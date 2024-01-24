import rospy
from uwb_msgs.msg import RangeStamped, PassiveStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from utils import is_in_range

class UwbTagSim:
    def __init__(self):
        rospy.init_node()
        self.node_name = rospy.get_name()

        self.load_params()

        if (self.robot_pose_topic):
            rospy.Subscriber(self.robot_pose_topic, PoseStamped, self.robot_pose_cb)

        # Need to create service to be a target for a TWR transaction (in which we check if in range)
        # and a different service to initiate TWR transactions
        
        # Need to generate passive listening measurements (where we also check if in range)

        # For each range measurement published we should also publish position of both tags 
        # to be able to 1) create visualizations 2) check if passive in range to both tags

        # Need to keep track of a clock state and update in time with the simulation
        self.simulate_clock()

    def load_params(self):
        self.uwb_range = rospy.get_param('/uwb_range')
        self.tag_id = rospy.get_param(self.node_name + '/tag_id')
        self.is_static = rospy.get_param(self.node_name + '/static')
        x = rospy.get_param(self.node_name + '/x')
        y = rospy.get_param(self.node_name + '/y')
        z = rospy.get_param(self.node_name + '/z')
        self.robot_pose_topic = rospy.get_param(self.node_name + '/robot_pose_topic')
        if (self.robot_pose_topic):
            self.moment_arm = np.array([x, y, z])
        else:
            self.position = np.array([x, y, z])

    def robot_pose_cb(self, msg):
        q = msg.pose.orientation
        q = np.array([q.x, q.y, q.z, q.w])
        p = msg.pose.position
        p = np.array([p.x, p.y, p.z])

        rospy.loginfo("TODO: SHOULD CHECK THIS! USED INVERSE")
        self.position = p + R.from_quat(q).apply(self.moment_arm, inverse=True)

    def simulate_clock(self):
        offset = np.random.uniform(0, 1)
        rate = rospy.Rate(100)
        # while not rospy.is_shutdown():
        #     rospy.
        #     t = rospy.Time.now().to_sec()
        #     t += offset
        #     rate.sleep()