#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
import math

def kill_turtle(name):
    rospy.wait_for_service('/kill')
    try:
        kill = rospy.ServiceProxy('/kill', Kill)
        kill(name)
    except rospy.ServiceException:
        rospy.logwarn(f"Could not kill {name}, it was probably already removed.")

def spawn_turtle(x, y, theta_deg, name):
    rospy.wait_for_service('/spawn')
    try:
        theta_rad = math.radians(theta_deg)  # Convert from degrees to radians
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, theta_rad, name)
        return x, y, theta_rad
    except rospy.ServiceException as e:
        rospy.logerr(f"Error while spawning turtle: {e}")
        return None

def main():
    rospy.init_node('turtle_spawn_goal', anonymous=True)

    # Ask the user for the target position (goal)
    x_goal = float(input("Enter x coordinate of the goal: "))
    y_goal = float(input("Enter y coordinate of the goal: "))
    theta_goal_deg = float(input("Enter goal theta angle (in degrees): "))

    # Remove the default turtle
    kill_turtle("turtle1")

    # Spawn a new turtle at the desired position
    result = spawn_turtle(x_goal, y_goal, theta_goal_deg, "turtle1")

    if result:
        x_current, y_current, theta_current = result

        # Calculate Distance to Goal (DTG)
        dtg = math.sqrt((x_goal - x_current)**2 + (y_goal - y_current)**2)

        # Calculate Angle to Goal (ATG) in radians and then convert to degrees
        atg_rad = math.atan2((y_goal - y_current), (x_goal - x_current))
        atg_deg = math.degrees(atg_rad)

        print(f"\nDistance to Goal (DTG): {dtg:.4f}")
        print(f"Angle to Goal (ATG): {atg_deg:.4f}°")

if __name__ == '__main__':
    main()
