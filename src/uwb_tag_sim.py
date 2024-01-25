import rospy
from uwb_msgs.msg import RangeStamped, PassiveStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from utils import is_in_range
from uwb_msgs.srv import Range, RangeRequest, RangeResponse

class UwbTagSim:
    def __init__(self):
        rospy.init_node()
        self.node_name = rospy.get_name()

        self.load_params()

        # TODO: do I need to assign this to a variable?
        rospy.Service("range_with_" + self.tag_id, Range, self.handle_range)

        range_srvs = {
            tag: rospy.ServiceProxy("range_with_" + tag, Range) for tag in self.list_of_tags
        }

        # TODO: NEED TO ADD ANOTHER SERVER FOR THE INITIATION OF TWR TRANSACTIONS

        if (self.robot_pose_topic):
            rospy.Subscriber(self.robot_pose_topic, PoseStamped, self.robot_pose_cb)

        # Need to create service to be a target for a TWR transaction (in which we check if in range)
        # and a different service to initiate TWR transactions
        
        # Need to generate passive listening measurements (where we also check if in range), by
        # subscribing to the range measurements of other tags

        # For each range measurement published we should also publish position of both tags 
        # to be able to 1) create visualizations 2) check if passive in range to both tags

        self.simulate_clock()

    def handle_range(self, req: RangeRequest):
        msg = RangeStamped()
        msg.header.stamp = rospy.Time.now()
        msg.range = None # TODO
        msg.from_id = req.id
        msg.to_id = self.tag_id
        msg.tx1 = None # TODO
        msg.rx1 = None # TODO
        msg.tx2 = None # TODO
        msg.rx2 = None # TODO
        msg.tx3 = None # TODO
        msg.rx3 = None # TODO
        msg.fpp1 = 0
        msg.fpp2 = 0
        msg.skew1 = 0
        msg.skew2 = 0
        return msg

    def load_params(self):
        self.uwb_range = rospy.get_param('/uwb_range')
        self.list_of_tags = rospy.get_param('/list_of_tags')
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
        # TODO: should make most of these parameters user configurable
        now = rospy.Time.now().to_sec()
        self.offset = np.random.uniform(-0.5e8, 0.5e8) # ns
        self.skew = np.random.uniform(-20e3, 20e3) # ppb
        rate = rospy.Rate(100)
        Q_tau = 0.4 # ns^2
        Q_gamma = 640 # ppb^2
        while not rospy.is_shutdown():
            # Get time delta
            old = now
            now = rospy.Time.now().to_sec()
            dt = now - old
            
            # Generate noise
            Qd = np.array([
                [Q_tau * dt + Q_gamma * dt**3 / 3, Q_gamma * dt**2 / 2],
                [Q_gamma * dt**2 / 2, Q_gamma * dt]
            ])
            noise = np.random.multivariate_normal([0, 0], Qd)
            
            # Update clock state
            self.offset += self.skew * dt + noise[0]
            self.skew += noise[1]

            rate.sleep()