#!/usr/bin/env python

import numpy as np
import sys
import os
import rospy
from hunav_evaluator import hunav_metrics

from hunav_msgs.msg import Agents, Agent
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped

from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
import actionlib

class HunavEvaluatorNode:

    def __init__(self):
        rospy.init_node('hunav_evaluator_node')

        name = 'hunav_evaluator'
        self.agents_list = []
        self.robot_list = []
        self.robot_goal = None
        self.metrics_to_compute = {}
        self.metrics_lists = {}
        self.number_of_behaviors = 6

        # Two modes:
        # 1- The user start/stop the recording through the
        #    the service /hunav_trigger_recording
        # 2- The recording start/stop process is semi-automatic:
        #    It starts when the first topic is received or a navigation goal is received.
        #    It stops when a certain time passes without receiving data.
        self.mode = rospy.get_param('~mode', 2)

        # Indicate the frequency of capturing the data 
        # (it must be slower than data publishing).
        # If the value is set to zero, the data is captured
        # at the same frequency as it is published.
        self.freq = rospy.get_param('~frequency', 0.0)

        # Base name of the result files
        self.result_file_path = rospy.get_param('~result_file', 'metrics')

        # Tag to identify the experiment
        self.exp_tag = rospy.get_param('~experiment_tag', '1')

        # Optionally, the data recording can be started when
        # a robot navigation goal is received
        self.use_navgoal_to_start = rospy.get_param('~use_nav_goal_to_start', True)

        # Read metrics
        for m in hunav_metrics.metrics.keys():
            ok = rospy.get_param('~metrics/' + m, True) # check if metric m must be computed or not
            if ok:
                self.metrics_to_compute[m] = 0.0

        rospy.loginfo("Hunav evaluator:")
        rospy.loginfo("mode: %i" % self.mode)
        rospy.loginfo("freq: %.1f" % self.freq)
        rospy.loginfo("use_nav_goal_to_start: %i" % self.use_navgoal_to_start)
        rospy.loginfo("result_file: %s" % self.result_file_path)
        rospy.loginfo("experiment_tag: %s" % self.exp_tag)
        rospy.loginfo("Metrics:")

        for m in self.metrics_to_compute.keys():
            # rospy.loginfo("m: %s, value: %s" % (m, self.metrics_to_compute[m]))
            rospy.loginfo("%s, value: %s" % (m, self.metrics_to_compute[m]))

        if self.freq > 0.0: # freq = 0.0 means that we store data at the same rate as it is published. This means that the record_timer
                            # is required only if we impose a certain data acquisition frequency. Instead, end_timer is always required.
            self.agents = Agents()  
            self.robot  = Agent()
            self.record_timer = rospy.Timer(rospy.Duration(1/self.freq), self.timer_record_callback)

        if self.mode == 1:
            self.recording = False
            self.recording_srv = rospy.Service('hunav_trigger_recording', Trigger, self.recording_service)

        elif self.mode == 2:
            if self.use_navgoal_to_start:
                self.recording = False
                self.ac = actionlib.SimpleActionClient('/rbkairos/move_base', MoveBaseAction)
                # Wait for the action server to start
                rospy.loginfo("Waiting for move_base action server...")
                self.ac.wait_for_server()
                rospy.loginfo(f"Connected to move_base action server. State: {self.ac.get_state()}")

            else: # here we want automatic mode but recording shouldn't start with a goal request. So what circumstance is this?
                self.recording = True

            self.init = False
            self.time_period = 3.0 # seconds
            self.last_time = rospy.get_rostime()

            self.goal_sent_time = rospy.Time()
            self.end_time = rospy.Time()
            self.reached = False

            self.end_timer = rospy.Timer(rospy.Duration(1.0), self.timer_end_callback)
        else:
            rospy.logerr("Mode not recognized. Only modes 1 or 2 are allowed")

        self.agent_sub = rospy.Subscriber('human_states', Agents, self.human_callback)
        self.robot_sub = rospy.Subscriber('robot_states', Agent, self.robot_callback)
        # self.goal_sub = rospy.Subscriber('goal_pose', PoseStamped, self.goal_callback)
        self.goal_sub = rospy.Subscriber('/rbkairos/move_base/goal', MoveBaseActionGoal, self.goal_callback)
        # Call the function to send a goal and monitor the action server

    def ask_and_send_goal(self):
        x = float(input("Enter x coordinate: "))
        y = float(input("Enter y coordinate: "))
        yaw = float(input("Enter yaw (radians): "))

        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "rbkairos_map"

        goal.goal.target_pose.header.frame_id = "rbkairos_map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y
        goal.goal.target_pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal.goal.target_pose.pose.orientation.w = np.cos(yaw / 2.0)

        self.ac.send_goal(goal.goal)
        self.goal_sent_time = rospy.get_rostime()

        rospy.loginfo("Sent goal: x = %f, y = %f, yaw = %f", x, y, yaw)

    def human_callback(self, msg):
        if self.mode == 2:
            self.init = True
            # self.last_time = rospy.get_rostime() # since it gets human agents data from gazebo/model_states, in this point of the code last_time would
                                                   #  constantly be updated. However we need it to reach a value and stop increasing, so that the end 
                                                   # timer can trigger the metrics computation. Therefore, we leave last_time update only in robot_callback.
                                                   # The reason is how it is designed: since it subscribes to /amcl_pose, and amcl_pose stops receiving
                                                   # new messages if the robot is still (either has reached the goal or has stopped for whatever reason),
                                                   # we are sure that at some point last_time will stop increasing.
        if self.recording:
            if self.freq == 0.0:
                rospy.logerr(f"agents_list len: {len(self.agents_list)}")
                self.agents_list.append(msg)
            else:
                self.agents = msg

    def robot_callback(self, msg):
        # rospy.logerr("robot_callback")
        if self.mode == 2:
            self.init = True
            # rospy.loginfo(f"robot_callback set self.init equal to: {self.init}")
            self.last_time = rospy.get_rostime()
            # rospy.logerr(f"self.last_time in robot: {self.last_time}")

        if self.recording:
            robot_msg = msg
            if self.robot_goal is not None:
                robot_msg.goals = [self.robot_goal.pose]
                robot_msg.goal_radius = 0.2

            if self.freq == 0.0:
                rospy.logerr(f"robot_list len: {len(self.robot_list)}")
                self.robot_list.append(robot_msg)
            else:
                self.robot = robot_msg

    def goal_callback_original(self, msg):
        self.robot_goal = msg
        if self.use_navgoal_to_start:
            rospy.loginfo("Goal received! Hunav evaluator started recording!")
            rospy.loginfo(f"goal_callback set self.recording equal to: {self.recording}")
            self.use_navgoal_to_start = False
            self.recording = True

    def goal_callback(self, msg):
        msg_pose_stamped = PoseStamped()
        msg_pose_stamped.header = msg.goal.target_pose.header
        msg_pose_stamped.pose   = msg.goal.target_pose.pose
        self.robot_goal = msg_pose_stamped

        if self.use_navgoal_to_start:
            rospy.loginfo("Goal received! Hunav evaluator started recording!")
            self.goal_sent_time = rospy.get_rostime()
            self.use_navgoal_to_start = False
            self.recording = True            
            rospy.loginfo(f"goal_callback set self.recording equal to: {self.recording}")

    def recording_service(self, request):
        response = TriggerResponse()
        response.success = True
        if self.recording:
            rospy.loginfo("Hunav evaluator stopping recording due to client request!")
            self.recording = False
            response.message = 'Hunav recording stopped'
            self.compute_metrics()
        else:
            rospy.loginfo("Hunav evaluator started recording due to client request!")
            self.recording = True
            response.message = 'Hunav recording started'
        return response

    def timer_end_callback_original(self, event):
        if self.init:
            secs = (rospy.get_rostime() - self.last_time).to_sec()
            if secs >= self.time_period:
                self.recording = False
                rospy.loginfo("Hunav evaluator stopping recording due to timer end!")
                self.compute_metrics()

    def timer_end_callback(self, event):
        state = self.ac.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached")
            self.goal_reached_time = rospy.get_rostime()
            self.reached = True

        if self.init and self.recording and self.reached:
            secs = (self.goal_reached_time - self.goal_sent_time).to_sec()
            rospy.logerr(f"secs: {secs}")
            self.recording = False
            rospy.loginfo("Hunav evaluator stopping recording due to timer end!")
            self.compute_metrics()

    def goal_reached(self):
        done = False
        rate = rospy.Rate(10)  # 10 Hz

        while not done:
            state = self.ac.get_state()
            rospy.logerr(f"Goal status: {state}")
            if state == actionlib.GoalStatus.SUCCEEDED or state == actionlib.GoalStatus.PREEMPTED:
                done = True
                rospy.loginfo("Goal reached")
                return done
            rate.sleep()

        return False
            
    def timer_record_callback(self, event):
        if self.recording and self.init:
            rospy.loginfo_once("Started recording with record_timer")
            rospy.logerr("appending data")
            self.agents_list.append(self.agents)
            self.robot_list.append(self.robot)

    def compute_metrics(self):
        if not self.check_data():
            rospy.loginfo("Data not collected. Not computing metrics.")
            return
        agents_size = len(self.agents_list) # they are forced to be the same by check_data.
        robot_size  = len(self.robot_list)
        rospy.loginfo(f"agents_size: {agents_size}")
        rospy.loginfo(f"robot_size: {robot_size}")
        rospy.loginfo(f"agents_list[-1]: {self.agents_list[-1]}")
        rospy.loginfo(f"robot_list[-1]: {self.robot_list[-1]}")

        rospy.loginfo("Hunav evaluator. Collected %i messages of agents and %i of robot" % (agents_size, robot_size))
        rospy.loginfo("Computing metrics...")

        # compute metrics for all agents
        self.metrics_lists['time_stamps'] = hunav_metrics.get_time_stamps(self.agents_list, self.robot_list)
        for m in self.metrics_to_compute.keys():
            metric = hunav_metrics.metrics[m](self.agents_list, self.robot_list)
            self.metrics_to_compute[m] = metric[0]
            if len(metric) > 1:
                self.metrics_lists[m] = metric[1]

        rospy.loginfo('Metrics computed:')
        rospy.loginfo(self.metrics_to_compute)
        self.store_metrics(self.result_file_path)

        # Now, filter according to the different behaviors
        # for i in range(1, (self.number_of_behaviors + 1)):
        #     self.compute_metrics_behavior(i)

        rospy.signal_shutdown("Metrics computation complete")
        sys.exit()

    def compute_metrics_behavior(self, behavior):
        beh_agents = []
        beh_robot = []
        beh_active = [0] * len(self.agents_list)
        i = 0
        for (la, lr) in zip(self.agents_list, self.robot_list):
            ag = Agents()
            ag.header = la.header
            for a in la.agents:
                if a.behavior == behavior:
                    ag.agents.append(a)
                if a.behavior_state != a.BEH_NO_ACTIVE:
                    beh_active[i] = 1
            if len(ag.agents) > 0:
                beh_agents.append(ag)
                beh_robot.append(lr)
            else:
                rospy.loginfo("No agents of behavior %i" % behavior)
                return None
            i += 1

        self.metrics_lists['behavior_active'] = beh_active
        # Then, compute the metrics for those agents
        for m in self.metrics_to_compute.keys():
            metric = hunav_metrics.metrics[m](beh_agents, beh_robot)
            self.metrics_to_compute[m] = metric[0]
            if len(metric) > 1:
                self.metrics_lists[m] = metric[1]

        rospy.loginfo('Metrics computed behavior %i:' % behavior)
        rospy.loginfo(self.metrics_to_compute)
        store_file = self.result_file_path
        if store_file.endswith(".txt"):
            store_file = store_file[:-4]
        store_file += '_beh_' + str(behavior) + '.txt'
        self.store_metrics(store_file)

    def store_metrics(self, result_file):
        # Define the desired directory
        directory = "/root/ros_ws/src/hunav_sim/hunav_evaluator/results"

        # Ensure the directory exists
        if not os.path.exists(directory):
            os.makedirs(directory)

        # Prepend the directory to the result file
        result_file = os.path.join(directory, result_file)

        rospy.loginfo("Storing metrics in file")
        rospy.loginfo(f"Result file before modification: {result_file}")
        
        list_file = result_file
        if not result_file.endswith(".txt"):
            result_file += '.txt'
            list_file += '_steps_' + str(self.exp_tag) + '.txt'
        else:
            list_file = list_file[:-4]
            list_file += '_steps_' + str(self.exp_tag) + '.txt'

        result_file = os.path.abspath(result_file)
        list_file = os.path.abspath(list_file)
        rospy.loginfo(f"Absolute result file path: {result_file}")
        rospy.loginfo(f"Absolute list file path: {list_file}")

        file_was_created = os.path.exists(result_file)
        rospy.loginfo(f"File was created: {file_was_created}")

        try:
            with open(result_file, 'a+') as file:
                if not file_was_created:
                    rospy.loginfo("Writing headers")
                    file.write('experiment_tag')
                    file.write('\t')
                    for m in self.metrics_to_compute.keys():
                        file.write(m)
                        file.write('\t')
                    file.write('\n')

                rospy.loginfo("Writing metrics")
                file.write(self.exp_tag)
                file.write('\t')
                for v in self.metrics_to_compute.values():
                    file.write(str(v))
                    file.write('\t')
                file.write('\n')
                file.flush()  # Ensure the buffer is flushed

            with open(list_file, 'w') as file2:
                rospy.loginfo("Writing list metrics")
                for m in self.metrics_lists.keys():
                    file2.write(m)
                    file2.write('\t')
                file2.write('\n')
                length = len(self.metrics_lists['time_stamps'])
                for i in range(length):
                    for m in self.metrics_lists.keys():
                        v = self.metrics_lists[m]
                        file2.write(str(v[i]))
                        file2.write('\t')
                    file2.write('\n')
                file2.flush()  # Ensure the buffer is flushed

        except Exception as e:
            rospy.logerr(f"Error while storing metrics: {str(e)}")

    def store_metrics_original(self, result_file):
        rospy.logerr("storing metrics in file")
        list_file = result_file
        if not result_file.endswith(".txt"):
            result_file += '.txt'
            list_file += '_steps_' + str(self.exp_tag) + '.txt'
        else:
            list_file = list_file[:-4]
            list_file += '_steps_' + str(self.exp_tag) + '.txt'

        file_was_created = os.path.exists(result_file)

        with open(result_file, 'a+') as file:
            if not file_was_created:
                file.write('experiment_tag')
                file.write('\t')
                for m in self.metrics_to_compute.keys():
                    file.write(m)
                    file.write('\t')
                file.write('\n')

            file.write(self.exp_tag)
            file.write('\t')
            for v in self.metrics_to_compute.values():
                file.write(str(v))
                file.write('\t')
            file.write('\n')

        with open(list_file, 'w') as file2:
            for m in self.metrics_lists.keys():
                file2.write(m)
                file2.write('\t')
            file2.write('\n')
            length = len(self.metrics_lists['time_stamps'])
            for i in range(length):
                for m in self.metrics_lists.keys():
                    v = self.metrics_lists[m]
                    file2.write(str(v[i]))
                    file2.write('\t')
                file2.write('\n')

    def check_data(self):
        agents_size = len(self.agents_list)
        robot_size = len(self.robot_list)

        if agents_size == 0 and robot_size == 0:
            return False
        if abs(agents_size - robot_size) != 0:
            while len(self.agents_list) > len(self.robot_list):
                self.agents_list.pop()
            while len(self.robot_list) > len(self.agents_list):
                self.robot_list.pop()

        return True

if __name__ == '__main__':
    node = HunavEvaluatorNode()
    node.ask_and_send_goal()
    rospy.spin()
