
# TurtleSim Lab 3 – Position Control

## Introduction

This lab explores basic position control using ROS and the Turtlesim simulator. The main focus is on implementing and understanding the control techniques to guide the turtle from its initial pose to a target pose using feedback from the environment. Specifically, the lab covers:

1. Computing **Distance To Goal (DTG)** and **Angle To Goal (ATG)**.
2. Spawning the turtle at a specific position and orientation.
3. Sending velocity commands to the turtle based on its current position and orientation, using a simple **Proportional (P) controller**.
4. Practical application of ROS topics and services for communication between nodes.

The exercise enhances the understanding of coordinate transformations and basic feedback control in a simulated robotic environment.

---

## Theoretical Background

### Euclidean Distance and Angle

To control the turtle's movement, we calculate two key metrics:

- **Distance to Goal (DTG)**: The straight-line distance between the turtle’s current position \((x, y)\) and the goal’s position \((x_g, y_g)\). This is calculated using the Euclidean distance formula:

$$
DTG = \sqrt{(x_{goal} - x_{current})^2 + (y_{goal} - y_{current})^2}
$$

- **Angle to Goal (ATG)**: The direction in which the turtle needs to turn to face the goal. It is calculated as the angle between the turtle’s current heading and the direction to the goal:

$$
ATG = \arctan2(y_{goal} - y_{current}, x_{goal} - x_{current})
$$

These metrics provide the foundation for controlling the turtle’s movement toward the goal.

### Proportional Control

To move the turtle toward the goal, we use a **Proportional (P) controller** that adjusts the turtle's velocity based on the distance to the goal and the angle error:

- **Linear velocity** is proportional to the distance to the goal:

$$
v = K_p \cdot DTG
$$

- **Angular velocity** is proportional to the angular error (difference between current orientation and goal orientation):

$$
\omega = K_p \cdot (ATG - \theta_{current})
$$

Here, $K_p$ is the proportional gain that determines how aggressively the turtle moves toward the goal.

### ROS Communication

ROS provides a structured way to communicate between nodes, making the system modular. In this lab, we interact with the following components:

- **Topics**: The turtle’s position is published to `/turtle1/pose`, and velocity commands are sent through `/turtle1/cmd_vel`.
- **Services**: The `/spawn` service is used to place the turtle at the target pose, and the `/kill` service removes the turtle.
- **Messages**: `Pose` (position and orientation) and `Twist` (linear and angular velocities) are the key message types used.

---

## Problem 1: Spawn Turtle at Goal & Compute DTG/ATG

### Task Overview

In this problem, the goal is to spawn the turtle at a specific position and orientation, and then compute the **Distance to Goal (DTG)** and **Angle to Goal (ATG)** from the initial position. Since the turtle starts at the goal, both DTG and ATG should initially be zero.

### Code Summary

```python
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
```

1. **ROS Imports**: The necessary ROS libraries (`rospy`, `geometry_msgs`, `turtlesim`) are imported to set up the communication with ROS.
2. **Service Definitions**: The `kill_turtle()` function deletes the current turtle, while `spawn_turtle()` spawns a new turtle at the target position and orientation.
3. **Main Function**: The main program reads the goal coordinates from the user, computes the DTG and ATG using the formulas described above, and prints these values.

This part of the lab helps test the pose calculation and is useful for verifying that the goal is correctly defined and the distance and angle metrics are being calculated properly.

---

## Problem 2: Move Turtle with Proportional Control

### Task Overview

In this problem, the turtle must move toward the goal from a starting position, with its motion controlled by the **Proportional Controller**. The turtle's position is updated continuously, and velocity commands are adjusted in real time to move the turtle toward the goal.

### Code Summary

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pow, radians, degrees

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('turtle_proportional_controller', anonymous=True)
        
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.x = 0
        self.y = 0
        self.theta = 0

    def pose_callback(self, data):
        self.x = data.x
        self.y = data.y
        self.theta = data.theta

    def get_user_input(self):
        print("\nEnter new target position")
        x = float(input("Target x coordinate: "))
        y = float(input("Target y coordinate: "))
        theta_deg = float(input("Desired angle (degrees): "))
        return x, y, radians(theta_deg)

    def move_to_goal(self, goal_x, goal_y):
        vel_msg = Twist()
        Kp_linear = 1.5
        Kp_angular = 6.0

        while not rospy.is_shutdown():
            # Calculate DTG and ATG using Euclidean coordinates
            dtg = sqrt(pow(goal_x - self.x, 2) + pow(goal_y - self.y, 2))
            atg = atan2(goal_y - self.y, goal_x - self.x)
            angle_diff = atg - self.theta

            # Normalize the angle to [-pi, pi]
            angle_diff = (angle_diff + 3.14159) % (2 * 3.14159) - 3.14159

            # Speeds proportional to the error
            vel_msg.linear.x = Kp_linear * dtg
            vel_msg.angular.z = Kp_angular * angle_diff

            self.cmd_vel_pub.publish(vel_msg)

            rospy.loginfo("DTG: %.4f | ATG: %.4f°", dtg, degrees(angle_diff))

            # When very close to the target, stop
            if dtg < 0.1:
                break

            self.rate.sleep()

        # Stop completely
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(vel_msg)
        rospy.loginfo("Target reached.\n")

    def rotate_to_theta(self, desired_theta):
        vel_msg = Twist()
        Kp_theta = 4.0

        while not rospy.is_shutdown():
            error_theta = desired_theta - self.theta
            error_theta = (error_theta + 3.14159) % (2 * 3.14159) - 3.14159

            vel_msg.angular.z = Kp_theta * error_theta
            self.cmd_vel_pub.publish(vel_msg)

            rospy.loginfo("Final angle error: %.4f°", degrees(error_theta))

            if abs(error_theta) < 0.05:
                break

            self.rate.sleep()

        # Stop rotation
        vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(vel_msg)

    def run(self):
        while not rospy.is_shutdown():
            goal_x, goal_y, goal_theta = self.get_user_input()
            self.move_to_goal(goal_x, goal_y)
            self.rotate_to_theta(goal_theta)

if __name__ == '__main__':
    try:
        controller = MoveTurtleProportionalControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
```

1. **Class-based Structure**: A class is used to define the main logic, where subscribers are set up to track the turtle’s pose and publishers send velocity commands.
2. **Callback Function**: The `pose_callback()` function updates the current pose of the turtle in real time.
3. **Move to Goal**: The `move_to_goal()` function computes the DTG and ATG continuously and sends corresponding linear and angular velocities to the turtle to guide it toward the target pose.
4. **Control Loop**: The program runs a loop until the turtle is within a certain distance of the goal (e.g., DTG < 0.1), ensuring the turtle reaches the desired location.

In this part of the lab, the proportional controller adjusts the turtle’s movement based on feedback, demonstrating how to close the loop between sensors (pose data) and actuators (velocity commands).

---

## Conclusion

This lab provides a hands-on introduction to position control using ROS and the Turtlesim simulator. By implementing a proportional controller and applying feedback control, we were able to move the turtle towards a target position efficiently. The lab not only reinforced key concepts in robotic motion but also highlighted the practical use of ROS for real-time control in simulated environments.
