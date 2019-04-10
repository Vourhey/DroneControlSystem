import rospy
from wp_test.srv import FileName, FileNameResponse

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class FlyByLocalPointsService:
    def __init__(self):
        rospy.init_node('fly_by_local_points_node')
        rospy.Service('fly_by_points', FileName, self.fly_service_cb)

        rospy.loginfo('Node is loaded')

    def fly_service_cb(self, file_name):
        rospy.loginfo(file_name)
        self.load_points(file_name.file_name)
        rospy.loginfo('Points are loaded...')

        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.loginfo("subscribed to state")

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.check_local_position)

        local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.wait_for_service('/mavros/cmd/arming')
        arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('/mavros/set_mode')
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        rate = rospy.Rate(20.0)

        while (not rospy.is_shutdown()) and (not self.mavros_state.connected):
            rospy.loginfo("Waiting for a connection")
            rospy.sleep(1)

        for i in range(10):
            local_pos_pub.publish(self.desired_point)
            rospy.loginfo(i)
            rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = "OFFBOARD"

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_request = rospy.Time().now()
        rospy.loginfo(last_request)

        while (not rospy.is_shutdown()) and (not self.finish_mission):
            if self.mavros_state.mode != 'OFFBOARD' and \
                    (rospy.Time().now() - last_request > rospy.Duration(5.0)):
                response = set_mode(offb_set_mode)
                if response.mode_sent:
                    rospy.loginfo("Offboard enabled")
                last_request = rospy.Time().now()
            else:
                if not self.mavros_state.armed and \
                        (rospy.Time().now() - last_request > rospy.Duration(5.0)):
                    response = arming(arm_cmd)
                    if response.success:
                        rospy.loginfo('Vehicle armed')
                    last_request = rospy.Time().now()

            local_pos_pub.publish(self.desired_point)
            rate.sleep()

        return FileNameResponse(True)

    def load_points(self, filename):
        self.points = []

        with open(filename, 'r') as f:
            for l in f.readlines():
                line = l.strip().split(' ')
                p = PoseStamped()
                p.pose.position.x = int(line[0])
                p.pose.position.y = int(line[1])
                p.pose.position.z = int(line[2])
                self.points.append(p)

        self.points_iter = iter(self.points)
        self.desired_point = next(self.points_iter)
        self.finish_mission = False

    def state_cb(self, data):
        self.mavros_state = data

    def check_local_position(self, current):
        if self.compare_points(current, self.desired_point):
            try:
                self.desired_point = next(self.points_iter)
            except StopIteration:
                self.finish_mission = True

    def compare_points(self, pose1, pose2):
        x1 = pose1.pose.position.x
        y1 = pose1.pose.position.y
        z1 = pose1.pose.position.z

        x2 = pose2.pose.position.x
        y2 = pose2.pose.position.y
        z2 = pose2.pose.position.z

        if abs(x1 - x2) < 0.1 and \
           abs(y1 - y2) < 0.1 and \
           abs(z1 - z2) < 0.1:
            return True

        return False

    def spin(self):
        rospy.spin()

