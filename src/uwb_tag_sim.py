import rospy
from uwb_msgs.msg import RangeStamped, PassiveStamped, TagsPositionStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from utils import is_in_range, compute_range_data
from uwb_msgs.srv import Range, RangeRequest, RangeResponse
from uwb_msgs.srv import InitiateTwr, InitiateTwrRequest, InitiateTwrResponse

class UwbTagSim:
    def __init__(self):
        rospy.init_node()
        self.node_name = rospy.get_name()

        self.load_params()

        # TODO: do I need to assign this to a variable?
        rospy.Service("range_with_" + self.tag_id, Range, self.handle_range)
        rospy.Service("initiate_from_" + self.tag_id, InitiateTwr, self.handle_initiate)
        
        # Publisher for the position of the two ranging tags
        self.tags_position_pub = rospy.Publisher("tags_position", TagsPositionStamped, queue_size=10)

        self.range_srvs = {
            tag: rospy.ServiceProxy("range_with_" + tag, Range) for tag in self.list_of_tags
        }

        if (self.robot_pose_topic):
            rospy.Subscriber(self.robot_pose_topic, PoseStamped, self.robot_pose_cb)
        
        # Need to generate passive listening measurements (where we also check if in range), by
        # subscribing to the range measurements of other tags

        self.simulate_clock()

    def handle_range(self, req: RangeRequest):
        # TODO: Add noise as a user configurable parameter
        # TODO: make \Delta t^{32} and \Delta t^{53} user configurable parameters
        if is_in_range(self.position, req.position):
            # Create range message and timestamp it
            range_msg = RangeStamped()
            range_msg.header.stamp = rospy.Time.now()
            
            # Compute range data
            range_data = compute_range_data(
                range_msg.header.stamp.to_nsec(),
                {'position': self.position, 'offset': self.offset, 'skew': self.skew},
                {'position': np.array(req.position), 'offset': req.offset, 'skew': req.skew},
                del_t_32 = 300*1e3,
                del_t_53 = 1500*1e3,
            )
            
            # Publish position of both tags
            tags_pos_msg = TagsPositionStamped()
            tags_pos_msg.header.stamp = range_msg.header.stamp
            tags_pos_msg.initiator_id = req.id
            tags_pos_msg.initiator_pos = req.position
            tags_pos_msg.target_id = self.tag_id
            tags_pos_msg.target_pos = self.position
            self.tags_position_pub.publish(tags_pos_msg)
    
            # Return range data        
            range_msg.range = range_data.range
            range_msg.from_id = req.id
            range_msg.to_id = self.tag_id
            range_msg.tx1 = range_data.tx1
            range_msg.rx1 = range_data.rx1
            range_msg.tx2 = range_data.tx2
            range_msg.rx2 = range_data.rx2
            range_msg.tx3 = range_data.tx3
            range_msg.rx3 = range_data.rx3
            return RangeResponse(range_msg)
        else:
            return None
    
    def handle_initiate(self, req: InitiateTwrRequest):
        self.range_srvs[req.target_id](
            self.position, self.offset, self.skew
        )
        return InitiateTwrResponse(True)

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