#! /usr/bin/env python3
import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from assignment_2_2024.msg import PoseVel
from assignment_2_2024.srv import GetLastGoal, GetLastGoalResponse
from nav_msgs.msg import Odometry
import math
import sys
import select

def input_with_timeout(prompt, timeout):
    print(prompt,'\n')
    input_ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if input_ready:
        return sys.stdin.readline().strip()
    else:
        return None  # Timeout occurred
    

class SendGoalClient:
    def __init__(self):
        rospy.loginfo("Waiting for action server...")
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to action server.")

        # Last goal variables
        self.last_goal_x = None
        self.last_goal_y = None

        # Service to provide the last goal
        self.last_goal_service = rospy.Service('/get_last_goal', GetLastGoal, self.handle_last_goal_request)

    def send_goal(self, x, y):
        """Send a goal to the action server."""
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        self.last_goal_x = x
        self.last_goal_y = y

        self.client.send_goal(goal, feedback_cb=self.check_feedback)
        rospy.loginfo(f"Goal sent: x={x}, y={y}")

    def cancel_goal(self):
        """Cancel the current goal."""
        self.client.cancel_goal()
        self.last_goal_x = 10000
        self.last_goal_y = 10000
        rospy.loginfo("Goal cancelled.")

    def check_feedback(self, feedback):
        """Feedback callback from the action server."""
        rospy.loginfo("Feedback received:")
        rospy.loginfo(f"Current state: {feedback.stat}")
        rospy.loginfo(f"Actual pose: {feedback.actual_pose}")

    def handle_last_goal_request(self, req):
        """Service callback to return the last goal."""
        if self.last_goal_x is None or self.last_goal_y is None:
            rospy.loginfo("No goal set yet.")
            return GetLastGoalResponse(x=float('nan'), y=float('nan'))
        rospy.loginfo(f"Providing last goal: x={self.last_goal_x}, y={self.last_goal_y}")
        return GetLastGoalResponse(x=self.last_goal_x, y=self.last_goal_y)



class OdomPublisher:
    def __init__(self):

        # Publisher for the custom message
        self.pub = rospy.Publisher('/custom_odom', PoseVel, queue_size=10)

        # Subscriber for the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Create a custom message instance
        custom_msg = PoseVel()

        # Extract position (x, y) and velocity (vel_x, vel_z)
        custom_msg.x = msg.pose.pose.position.x
        custom_msg.y = msg.pose.pose.position.y
        custom_msg.vel_x = msg.twist.twist.linear.x
        custom_msg.vel_z = msg.twist.twist.angular.z

        # Publish the custom message
        self.pub.publish(custom_msg)


def main():
    rospy.init_node('send_goal_client')
    client = SendGoalClient()
    odom_publisher = OdomPublisher()
    
    client.client_ui(1)

    while not rospy.is_shutdown():
        
        client.client_ui(0)
        rospy.spin()





if __name__ == '__main__':
    main()
