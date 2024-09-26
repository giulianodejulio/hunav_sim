#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from hunav_msgs.msg import Agent
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

class HunavRobotStatePub:
    def __init__(self):
        rospy.init_node('hunav_robot_state_pub')
        
        self.agent_pub = rospy.Publisher('/robot_states', Agent, queue_size=10)
        
        # Subscribe to /rbkairos/amcl_pose to get the robot's position
        rospy.Subscriber('/rbkairos/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # Subscribe to /rbkairos/robotnik_base_control/diff_drive_odom to get the robot's velocity
        rospy.Subscriber('/rbkairos/robotnik_base_control/diff_drive_odom', Odometry, self.odom_callback)
        
        self.agent_msg = Agent()
        self.agent_msg.id = 1
        self.agent_msg.type = Agent.ROBOT
        self.agent_msg.name = rospy.get_param('~robot_name', 'robot')
        self.agent_msg.group_id = -1  # Assuming no group affiliation
        self.agent_msg.radius = 0.5  # Example radius, adjust as necessary

    def pose_callback(self, msg):
        self.agent_msg.position = msg.pose.pose
        # Optionally compute yaw from orientation quaternion if needed
        # self.agent_msg.yaw = ... (convert quaternion to yaw)

        # Publish the agent message
        self.agent_pub.publish(self.agent_msg)

    def odom_callback(self, msg):
        # Extract the velocity from the odometry message
        self.agent_msg.velocity.linear.x = msg.twist.twist.linear.x
        self.agent_msg.velocity.linear.y = msg.twist.twist.linear.y
        self.agent_msg.velocity.angular.z = msg.twist.twist.angular.z

        self.agent_msg.linear_vel = msg.twist.twist.linear.x
        self.agent_msg.angular_vel = msg.twist.twist.angular.z

        # Publish the updated agent message
        self.agent_pub.publish(self.agent_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = HunavRobotStatePub()
    node.run()
