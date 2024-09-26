#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from hunav_msgs.msg import Agents, Agent
from geometry_msgs.msg import Pose, Twist

class ActorPosPublishNode:

    def __init__(self):
        rospy.init_node('hunav_human_states_node')

        # Publisher for human_states topic
        self.human_states_pub = rospy.Publisher('/human_states', Agents, queue_size=10)

        # Subscriber to /gazebo/model_states
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelStatesCallback)

    def modelStatesCallback(self, msg):
        agents_msg = Agents()
        agents_msg.header.stamp = rospy.Time.now()

        for i, name in enumerate(msg.name):
            if name.startswith('actor'):
                agent_msg = Agent()
                agent_msg.id = i
                agent_msg.type = Agent.ROBOT  # Assuming these are robots; change as needed
                agent_msg.name = name
                agent_msg.position = msg.pose[i]
                agent_msg.velocity = msg.twist[i]
                agent_msg.yaw = 0.0  # Default yaw
                agent_msg.desired_velocity = 0.0  # Default desired velocity
                agent_msg.radius        = 0.0  # Default radius
                agent_msg.linear_vel    = 0.0  # Default linear velocity
                agent_msg.angular_vel   = 0.0  # Default angular velocity
                agent_msg.behavior.type = 0  # Default behavior
                agent_msg.goals = []  # No goals by default
                agent_msg.cyclic_goals = False  # Default cyclic goals
                agent_msg.goal_radius = 0.0  # Default goal radius
                agent_msg.closest_obs = []  # No closest obstacles by default

                agents_msg.agents.append(agent_msg)

        # Publish the Agents message
        self.human_states_pub.publish(agents_msg)

if __name__ == '__main__':
    try:
        node = ActorPosPublishNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
