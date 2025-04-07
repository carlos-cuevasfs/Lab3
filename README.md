
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

Here, \(K_p)\ is the proportional gain that determines how aggressively the turtle moves toward the goal.

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
def kill_turtle(name):
    rospy.wait_for_service('/kill')
    rospy.ServiceProxy('/kill', Kill)(name)

def spawn_turtle(x, y, theta_deg, name):
    theta_rad = math.radians(theta_deg)
    rospy.wait_for_service('/spawn')
    rospy.ServiceProxy('/spawn', Spawn)(x, y, theta_rad, name)
    return x, y, theta_rad

def main():
    rospy.init_node('turtle_spawn_goal', anonymous=True)
    xg, yg, thetag_deg = map(float, input("x y theta: ").split())
    kill_turtle("turtle1")
    xc, yc, thetac = spawn_turtle(xg, yg, thetag_deg, "turtle1")
    dtg = math.hypot(xg - xc, yg - yc)
    atg = math.degrees(math.atan2(yg - yc, xg - xc))
    print(f"DTG: {dtg:.4f} | ATG: {atg:.4f}°")
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
class TurtleControl:
    def __init__(self):
        rospy.init_node('turtle_controller')
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.x = self.y = self.theta = 0
        self.rate = rospy.Rate(10)

    def pose_callback(self, msg):
        self.x, self.y, self.theta = msg.x, msg.y, msg.theta

    def get_goal(self):
        x, y, theta = map(float, input("Goal x y theta: ").split())
        return x, y, math.radians(theta)

    def move_to_goal(self, gx, gy):
        Kp_lin, Kp_ang = 1.5, 6.0
        while not rospy.is_shutdown():
            dtg = math.hypot(gx - self.x, gy - self.y)
            atg = math.atan2(gy - self.y, gx - self.x)
            diff = (atg - self.theta + math.pi) % (2 * math.pi) - math.pi
            vel = Twist()
            vel.linear.x = Kp_lin * dtg
            vel.angular.z = Kp_ang * diff
            self.cmd_pub.publish(vel)
            rospy.loginfo(f"DTG: {dtg:.2f} | ATG: {math.degrees(diff):.2f}°")
            if dtg < 0.1: break
            self.rate.sleep()
```

1. **Class-based Structure**: A class is used to define the main logic, where subscribers are set up to track the turtle’s pose and publishers send velocity commands.
2. **Callback Function**: The `pose_callback()` function updates the current pose of the turtle in real time.
3. **Move to Goal**: The `move_to_goal()` function computes the DTG and ATG continuously and sends corresponding linear and angular velocities to the turtle to guide it toward the target pose.
4. **Control Loop**: The program runs a loop until the turtle is within a certain distance of the goal (e.g., DTG < 0.1), ensuring the turtle reaches the desired location.

In this part of the lab, the proportional controller adjusts the turtle’s movement based on feedback, demonstrating how to close the loop between sensors (pose data) and actuators (velocity commands).

---

## Conclusion

This lab provides a hands-on introduction to position control using ROS and the Turtlesim simulator. By implementing a proportional controller and applying feedback control, we were able to move the turtle towards a target position efficiently. The lab not only reinforced key concepts in robotic motion but also highlighted the practical use of ROS for real-time control in simulated environments.
