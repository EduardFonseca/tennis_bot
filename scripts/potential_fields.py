#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np 

pub = None
obstacles = []

def quaternion_to_rpz(x, y, z, w):
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
    pitch = np.arcsin(2*(w*y - z*x))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
    return roll, pitch, yaw

def adjust_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def get_robot_position(robot):
    x = robot.pose.pose.position.x
    y = robot.pose.pose.position.y
    q_x = robot.pose.pose.orientation.x
    q_y = robot.pose.pose.orientation.y
    q_z = robot.pose.pose.orientation.z
    q_w = robot.pose.pose.orientation.w
    _, _, angle = quaternion_to_rpz(q_x, q_y, q_z, q_w)
    return x, y, angle

def distance(x_0, y_0, x_1, y_1):
    return np.sqrt((x_1 - x_0)**2 + (y_1 - y_0)**2)

def repulsive_force(x, y, obstacles, K, e_0):
    F_rep = np.array([0, 0])
    for obstacle in obstacles:
        dx_o, dy_o = obstacle
        e_i = np.sqrt(dx_o**2 + dy_o**2)
        F_rep = F_rep + K*((1/e_i)**3)*((1/e_0) - (1/e_i))*np.array([dx_o, dy_o])

    return F_rep

def laser_scan_callback(msg):
    global obstacles
    
    # Convert laser scan data to NumPy array
    ranges = np.array(msg.ranges)

    # Compute angles for all measurements at once
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

    # Get indices where ranges are **finite (not inf or NaN)**
    valid_indices = np.isfinite(ranges)  # True for valid (finite) values

    # Compute dx and dy for all valid measurements at once
    dx = ranges[valid_indices] * np.cos(angles[valid_indices])
    dy = ranges[valid_indices] * np.sin(angles[valid_indices])

    # Store obstacles as a list of tuples
    obstacles = list(zip(dx, dy))


    # Optional Debugging: Uncomment to print last detected obstacle
    # if obstacles:
    #     print(f"Last detected obstacle: {obstacles[-1]}")

def callback(msg):
    global pub, obstacles

    # Constants
    K_att = 0.2
    K_rep = 0.01
    e_0 = 4

    v_max = 0.5
    w_max = np.pi/2
    DELTA = 0.01

    # Goal_position
    goal_x = 15
    goal_y = 0

    # Aquire robot position
    x, y, angle = get_robot_position(msg)

    # Calculate distance to goal
    rho = distance(x, y, goal_x, goal_y)
    

    if rho < DELTA:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
        print('Goal reached')
        return
    
    # Calculate atractive force
    F_a = K_att*np.array([goal_x - x, goal_y - y])
    
    F_r = repulsive_force(x, y, obstacles, K_rep, e_0)

    #Calculate Total force
    F = F_a + F_r
    print("F_a: ", F_a, " || F_r: ", F_r, " || F: ", F)

    #Calculate linear and angular velocity
    v = min(np.linalg.norm(F), v_max)

    w = adjust_angle(np.arctan2(F[1], F[0]) - angle)
    # w must be between -pi and pi

    w = np.sign(w)*min(abs(w), w_max)

    # Create a twist message to send to the robot
    twist = Twist()
    twist.linear.x = v
    twist.angular.z = w

    # Publish the message
    pub.publish(twist)

def potential_fields():
    global pub
    pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=10)
    rospy.init_node('odom_listner', anonymous=True)

    rospy.Subscriber('/p3dx/odom', Odometry, callback)
    rospy.Subscriber('/p3dx/laser/scan', LaserScan, laser_scan_callback)
    
    rospy.spin()

if __name__ == "__main__":
    potential_fields()