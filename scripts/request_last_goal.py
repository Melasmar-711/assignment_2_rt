#!/usr/bin/env python3
import rospy
from assignment_2_2024.srv import GetLastGoal
def get_last_goal():
    rospy.wait_for_service('/get_last_goal')
    try:
        service_client = rospy.ServiceProxy('/get_last_goal', GetLastGoal)
        response = service_client()
        if response:
            if(response.x == float('nan') and response.y == float('nan')):
                rospy.loginfo("No goal set yet.")
            if(response.x == 10000 and response.y == 10000):
                rospy.loginfo("Last Goal was cancelled.")
            else:
                rospy.loginfo(f"Last goal is : x{response.x}, y{response.y}\n")
        else:
            rospy.loginfo("Failed to retrieve the last goal.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('last_goal_client')
    rospy.loginfo("Press 'Enter' to request the last goal coordinates or type 'q' to quit.")
    
    while not rospy.is_shutdown():
        user_input = input("Enter your choice: ").strip()
        if user_input.lower() == 'q':
            rospy.loginfo("Exiting...")
            break
        elif user_input == '':
            rospy.loginfo("Requesting last goal coordinates...")
            get_last_goal()
        else:
            rospy.loginfo("Invalid input. Press 'Enter' to request or 'q' to quit.")

if __name__ == '__main__':
    main()
