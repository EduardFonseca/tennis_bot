#!/usr/bin/env python
import rospy
from pynput.keyboard import Key, Listener
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

twist = Twist()
pub = None  # Declare publisher globally
stop_robot = False

def on_press(key):
    global twist
    global stop_robot
    try:
        botao = key.char
    except AttributeError:
        botao = key.name

    lin_vel = 0.5
    ang_vel = 0.5

    # Update twist message based on key press
    if botao in ('w', 'up') and not stop_robot:
        twist.linear.x = lin_vel
        twist.angular.z = 0.0
    elif botao in ('s', 'down'):
        twist.linear.x = -lin_vel
        twist.angular.z = 0.0
    elif botao in ('a', 'left'):
        twist.linear.x = 0.0
        twist.angular.z = ang_vel
    elif botao in ('d', 'right'):
        twist.linear.x = 0.0
        twist.angular.z = -ang_vel

def on_release(key):
    global twist
    if key == Key.esc:
        return False  # Stop listener on ESC key
    
    # Stop movement when key is released
    twist.linear.x = 0.0
    twist.angular.z = 0.0

def publish_twist(event):
    """Callback function to publish Twist messages at a fixed rate."""
    global pub, twist
    pub.publish(twist)

def laser_callback(msg):
    """Callback function to process LaserScan messages."""
    global twist
    global stop_robot

    # Filter out infinite values (no obstacles detected in that range)
    valid_ranges = [r for r in msg.ranges if r != float('inf')]

    if valid_ranges:
        min_distance = min(valid_ranges)
        
        # verify position of the obstacle
        min_index = np.argmin(msg.ranges)
        min_angle = msg.angle_min + min_index * msg.angle_increment
        min_ang = np.rad2deg(min_angle)
        # rospy.loginfo(f"Minimum detected distance: {min_distance:.2f}m at {min_ang:.2f} degrees")

        # Stop movement if an obstacle is closer than 0.5m and in front of the robot (angle < 30 degrees)
        if min_distance < 0.5 and not (min_ang > 45 or min_ang < -45):
            rospy.logwarn("Obstacle detected! Stopping robot.")
            if not stop_robot:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                stop_robot = True
        else:
            rospy.logerr("No obstacle detected.")
            stop_robot = False

def robot_control():
    global pub
    rospy.init_node('teleop_key', anonymous=True)
    pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=10)

    # Subscribe to LaserScan topic
    rospy.Subscriber('/p3dx/laser/scan', LaserScan, laser_callback)

    # Start a timer to publish messages at 10Hz
    rospy.Timer(rospy.Duration(0.1), publish_twist)

    # Start keyboard listener
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()  # Keep listening until ESC is pressed

if __name__ == '__main__':
    try:
        robot_control()
    except rospy.ROSInterruptException:
        pass
