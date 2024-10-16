import math

def inverse_kinematics( x, y):
    # Calculate joint angles for given x, y
    l1 = 0.510  # Length of link 1
    l2 = 0.360 # Length of link 2
    theta2 = math.acos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))


    # Calculate possible theta1 values using atan2
    theta1a = math.atan2(y, x) - math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    theta1b = math.atan2(y, x) + math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))


    # Choose theta1 based on elbow up/down configuration
    # Elbow up: 
    theta1 = theta1b

    theta1_adj = math.pi/2 - theta1
    theta2_adj = math.pi/2 - theta2
 
    # return theta1, theta2
    return theta1_adj, theta2_adj

    # Corrected log messages
    # rospy.loginfo(f"Theta1: {theta1}")
    # rospy.loginfo(f"Theta2: {theta2}")

    

    return theta1,theta2

def fwd_kinematics():
    # Given link lengths and joint angles (in radians)
    l1 = 0.510  # Length of link 1
    l2 = 0.360 
    theta1 = 0.0  # Joint angle 1 in radians (converted from degrees)
    theta2 = 0.0 # Jo0int angle 2 in radians (converted from degrees)
    # theta1 = 0.0  # Joint angle 1 in radians (converted from degrees)
    # theta2 = 0.0 # Jo0int angle 2 in radians (converted from degrees)

    # Calculate the end effector position (x, y)
    x = l1 * math.sin(theta1) + l2 * math.cos(theta2)
    y = l1 * math.cos(theta1) + l2 * math.sin(theta2)

    print(f"x: {x}")
    print(f"y: {y}")
    
    return


def main():
    l1 = 0.510  # Length of link 1
    l2 = 0.360 # Length of link 2
    x = l2
    y = l1

    x = x + 0.01

    theta1,theta2 = inverse_kinematics(x,y)
    print(f"Theta1: {theta1}")
    print(f"Theta2: {theta2}")

    # fwd_kinematics()

if __name__ == "__main__":
    main()
