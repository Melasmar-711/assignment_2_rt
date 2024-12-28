import rospy
import actionlib
from Planning.msg import PlanningAction, PlanningGoal

class SendGoalClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()

    def send_goal(self, goal):
        self.client.send_goal(goal, feedback_cb=self.check_feedback,done_cb=self.done_callback)
    
    def check_feedback(self, feedback):
        print('Feedback received:', feedback.stat)
        print('Actual pose:', feedback.actual_pose)

    def cancel_goal(self):
        self.client.cancel_goal()

    def done_callback(self, state, result):
        print(f"Goal completed with state: {state}")
        print(f"Result: {result}")  



def main():
    rospy.init_node('send_goal_client')
    client = SendGoalClient()

    while True:
        goal = PlanningGoal()
        wish = input('Enter a new goal x,y or the letter "c" for cancel: ')

        if wish == 'c':
            client.cancel_goal()
            print("Goal cancelled.")
            continue

        try:
            x, y = map(float, wish.split(','))
            goal.x = x
            goal.y = y
        except ValueError:
            print("Invalid input. Please enter coordinates in the format 'x,y' or 'c' to cancel.")
            continue

        client.send_goal(goal)  # No blocking here
        print("Goal sent! Waiting for result asynchronously.")


if __name__ == '__main__':
    main()
