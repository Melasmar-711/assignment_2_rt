#!/usr/bin/env python3

"""
ROS Service Client for Requesting Last Navigation Goal

This module provides a client to query the last goal coordinates sent to the robot
through a ROS service. It includes a simple command-line interface for interaction.

.. module:: request_last_goal
   :platform: Unix
   :synopsis: ROS service client for querying last navigation goal coordinates

.. moduleauthor:: Mahmoud_Elasmar <s5927704@studenti.unige.it>
"""

import rospy
from assignment_2_2024.srv import GetLastGoal

def get_last_goal():
    """
    Request and display the last goal coordinates from the service server.
    
    This function:
    - Connects to the '/get_last_goal' service
    - Handles different response cases (no goal, cancelled goal, valid goal)
    - Provides appropriate logging output
    
    The service response can indicate:
    - (nan, nan): No goal has been set yet
    - (10000, 10000): The last goal was cancelled
    - (x, y): Valid coordinates of the last goal
    
    Raises
    ------
    rospy.ServiceException
        If the service call fails
    """
    rospy.wait_for_service('/get_last_goal')
    try:
        # Create service client
        service_client = rospy.ServiceProxy('/get_last_goal', GetLastGoal)
        
        # Call the service
        response = service_client()
        
        if response:
            # Check for special cases
            if response.x == float('nan') and response.y == float('nan'):
                rospy.loginfo("No goal set yet.")
            elif response.x == 10000 and response.y == 10000:
                rospy.loginfo("Last Goal was cancelled.")
            else:
                rospy.loginfo(f"Last goal is: x={response.x}, y={response.y}\n")
        else:
            rospy.loginfo("Failed to retrieve the last goal.")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        raise

def main():
    """
    Main function to run the last goal client node.
    
    Provides a simple command-line interface that allows:
    - Pressing Enter to request the last goal coordinates
    - Pressing 'q' to quit the application
    
    The node continuously listens for user input until shutdown.
    """
    rospy.init_node('last_goal_client')
    
    # Print header information
    rospy.loginfo("LAST TARGET SERVICE")
    rospy.loginfo("To request the last goal press \"Enter\" and 'q' to quit.")
    
    # Main loop
    while not rospy.is_shutdown():
        try:
            user_input = input("Enter your choice: ").strip()
            
            if user_input.lower() == 'q':
                rospy.loginfo("Exiting...")
                break
            elif user_input == '':
                rospy.loginfo("Requesting last goal coordinates...")
                get_last_goal()
            else:
                rospy.loginfo("Invalid input. Press 'Enter' to request or 'q' to quit.")
                
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
            break

if __name__ == '__main__':
    main()
