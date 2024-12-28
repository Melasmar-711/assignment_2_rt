#! /usr/bin/env python3
import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal


import sys
import select

def input_with_timeout(prompt, timeout):
    print(prompt, end='', flush=True)
    input_ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if input_ready:
        return sys.stdin.readline().strip()
    else:
        return None  # Timeout occurred
    


class SendGoalClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()

    def send_goal(self, goal):
        self.client.send_goal(goal, feedback_cb=self.check_feedback)
        self.client.wait_for_result()


    
    def client_ui(self):
        while True:
            goal = PlanningGoal()
            wish = input_with_timeout('Enter a new goal x,y or the letter "c" for cancel: ',10)

            if wish == 'c':
                self.client.cancel_goal()
                print("Goal cancelled.")
                return
            if wish is None:
                print("No input received.")
                return
            
            try:
                x, y = map(float, wish.split(','))
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                self.send_goal(goal)

            except ValueError:
                print("Invalid input. Please enter coordinates in the format 'x,y' or 'c' to cancel.")
                continue

        
    def check_feedback(self, feedback):
        print('Feedback received:', feedback.stat)
        print('Actual pose:', feedback.actual_pose)
        self.client_ui()


    def cancel_goal(self):
        self.client.cancel_goal()




def main():
    rospy.init_node('send_goal_client')
    client = SendGoalClient()
    while not rospy.is_shutdown():
        client.client_ui()
        rospy.spin()





if __name__ == '__main__':
    main()
