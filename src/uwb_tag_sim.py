import rospy
from uwb_msgs.msg import RangeStamped, PassiveStamped, TagsStateStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from utils import is_in_range, compute_range_data, compute_passive_data
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
        self.tags_state_pub = rospy.Publisher("tags_state", TagsStateStamped, queue_size=10)

        # Publisher for the range measurements 
        self.range_pub = rospy.Publisher("/uwb/range", RangeStamped, queue_size=10)

        # Publisher for passive listening measurements
        self.passive_pub = rospy.Publisher("/uwb/passive", PassiveStamped, queue_size=10)


        self.range_srvs = {
            tag: rospy.ServiceProxy("range_with_" + tag, Range) for tag in self.list_of_tags
        }

        if (self.robot_pose_topic):
            rospy.Subscriber(self.robot_pose_topic, PoseStamped, self.robot_pose_cb)

        # Subscribe to the range measurements of other tags
        rospy.Subscriber(self.range_topic, RangeStamped, self.range_cb)

        # Subscribe to the topic publishing the position of the other tags
        self.ranging_tags_state = TagsStateStamped()
        rospy.Subscriber("tags_state", TagsStateStamped, self.tags_state_cb)

        self.simulate_clock()

    def load_params(self):
        self.range_topic = rospy.get_param('/range_topic')
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
            tags_state_msg = TagsStateStamped()
            tags_state_msg.header.stamp = range_msg.header.stamp
            tags_state_msg.initiator_id = req.id
            tags_state_msg.initiator_pos = req.position
            tags_state_msg.initiator_offset = req.offset
            tags_state_msg.initiator_skew = req.skew
            tags_state_msg.target_id = self.tag_id
            tags_state_msg.target_pos = self.position
            tags_state_msg.target_offset = self.offset
            tags_state_msg.target_skew = self.skew
            self.tags_state_pub.publish(tags_state_msg)
    
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
            
            self.range_pub.publish(range_msg)

            return RangeResponse(True)
        
        return RangeResponse(False)
    
    def handle_initiate(self, req: InitiateTwrRequest):
        self.range_srvs[req.target_id](
            self.position, self.offset, self.skew
        )
        return InitiateTwrResponse(True)

    def robot_pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        q = np.array([q.x, q.y, q.z, q.w])
        p = msg.pose.position
        p = np.array([p.x, p.y, p.z])

        rospy.loginfo("TODO: SHOULD CHECK THIS! USED INVERSE")
        self.position = p + R.from_quat(q).apply(self.moment_arm, inverse=True)

    def range_cb(self, msg: RangeStamped):
        if (msg.from_id == self.tag_id or msg.to_id == self.tag_id):
            return
        
        # Check if range message and position of ranging tags have same timestamp
        if (msg.header.stamp != self.ranging_tags_state.header.stamp):
            rospy.logwarn("Missed a passive listening measurement!")
            return
        
        # Check if the ranging tags are in range
        if (not is_in_range(
                self.position, 
                self.ranging_tags_state.target_pos
            )
        ):
            return
        if (not is_in_range(
                self.position, 
                self.ranging_tags_state.initiator_pos
            )
        ):
            return


        # Generate passive listening measurement
        passive_msg = PassiveStamped()
        passive_msg.header.stamp = msg.header.stamp
        passive_msg.my_id = self.tag_id
        passive_msg.from_id = msg.from_id
        passive_msg.to_id = msg.to_id
        passive_msg.tx1_n = msg.tx1
        passive_msg.rx1_n = msg.rx1
        passive_msg.tx2_n = msg.tx2
        passive_msg.rx2_n = msg.rx2
        passive_msg.tx3_n = msg.tx3
        passive_msg.rx3_n = msg.rx3

        passive_ts = compute_passive_data(
            passive_msg.header.stamp.to_nsec(),
            {
                'position': self.ranging_tags_state.initiator_pos, 
                'offset': self.ranging_tags_state.initiator_offset, 
                'skew': self.ranging_tags_state.initiator_skew
            },
            {
                'position': self.ranging_tags_state.target_pos, 
                'offset': self.ranging_tags_state.target_offset, 
                'skew': self.ranging_tags_state.target_skew
            },
            {
                'position': self.position, 
                'offset': self.offset, 
                'skew': self.skew
            },
            del_t_32 = 300*1e3,
            del_t_53 = 1500*1e3,
        )
        
        passive_msg.rx1 = passive_ts['rx1']
        passive_msg.rx2 = passive_ts['rx2']
        passive_msg.rx3 = passive_ts['rx3']

        self.passive_pub.publish(passive_msg)

    def tags_state_cb(self, msg: TagsStateStamped):
        if (msg.initiator_id == self.tag_id or msg.target_id == self.tag_id):
            return

        # Update position of the two ranging tags
        self.ranging_tags_state = msg

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