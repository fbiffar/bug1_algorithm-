#!/usr/bin/env python3

import rospy 
import math 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion 

def odomdata_callback(msg):

    global odometry_turtle; odometry_turtle = msg 

def goal_callback(msg):

    global goal; goal = msg.pose.pose.position
    global goal_received; goal_received = True 

def laserdata_callback(msg):

    #5 Hz, 1 = 11,5 grad
    global state
    global point
    global distance 
    global start 
    global goal
    global goal_received 
    #Prints the length of ranges array, in our case there are total 360 readings, one reading for each 1 degree

    #print(len(msg.ranges)) 

    #To make use of laser data, we will need directional readings. For example, front, back, left and right of the robot. In our case, 0: Front, 180: Back, 270: Right, 270: Left,  are directions of laserbeam for robot

    #print(msg.ranges[0], msg.ranges[270], msg.ranges[180], msg.ranges[270])
    
    # sample head collision avoidance algorithm(You will have to write your algorithm here)
    
    #checks if goal and postiton and orientation of the turtle bot is given
    if not (goal_received):
        #print('waiting for goal')
        return
    
    # #3 modes: LINE: moving along line until, OBSTACLE: surrounding obstacle, LEAVE: leaving obstacle at optimal point 
    if arrived_at_desired_point(goal) == True:
        print('arrived at goal')
        return

    if state == 'LINE':
        #calculate desired heading of robot
        anglular_z = calculate_heading()
        #move robot to desired heading
        move_the_bot.angular.z = anglular_z
        move_the_bot.linear.x = 0.0
        publish_to_cmd_vel.publish(move_the_bot)
        #check if no obstacle in the way if no move forward
        if  msg.ranges[0] > 0.3: # and msg.ranges[380] > 0.5 and msg.ranges[10] > 0.5  :
            print('moving towards goal')
            move_the_bot.linear.x = 0.1
            publish_to_cmd_vel.publish(move_the_bot)
        #if yes, stop the robot switch to OBSTACLE state and save the starting position and current distance to the goal
        else: 
            move_the_bot.linear.x = 0.0
            move_the_bot.angular.z = 0.0
            publish_to_cmd_vel.publish(move_the_bot)
            state = 'OBSTACLE'
            print('stopped obstacle detected')
            start = odometry_turtle.pose.pose.position
            point = odometry_turtle.pose.pose.position
            distance = distance_to_goal(goal)
    
    if state == 'OBSTACLE':

        navigate_around_obstacle(start, msg, 'LEAVE',0)


    if state == 'LEAVE':

        print ('LEAVE to', point)


        navigate_around_obstacle(point, msg, 'LINE',21)

    

def calculate_heading():
    dx = goal.x - odometry_turtle.pose.pose.position.x
    dy = goal.y - odometry_turtle.pose.pose.position.y
    desired_angle = math.atan(dy/dx)
    if (dx < 0):
        if(dy>0):
            desired_angle = 180 + math.atan(dy/dx)
        else:
            desired_angle = -180 + math.atan(dy/dx)
            
    euler = euler_from_quaternion([odometry_turtle.pose.pose.orientation.x, odometry_turtle.pose.pose.orientation.y, odometry_turtle.pose.pose.orientation.z, odometry_turtle.pose.pose.orientation.w])
    return desired_angle - euler[2]

def distance_to_goal(aim):
    dx = aim.x - odometry_turtle.pose.pose.position.x
    dy = aim.y - odometry_turtle.pose.pose.position.y
    return math.sqrt(dx*dx + dy*dy)

def update_best_point():
    global point 
    global distance 
    global goal
    if (distance > distance_to_goal(goal)):
        #print('new best point')
        distance = distance_to_goal(goal)
        point = odometry_turtle.pose.pose.position

def arrived_at_desired_point(location):
    if distance_to_goal(location) < 0.2:
        return True
    else: 
        return False 

def navigate_around_obstacle(aim, msg, next_state, counter):
    global state 
    global start 
    if (counter == 5):
        start = odometry_turtle.pose.pose.position
        print(start)
    #turn on the spot until no obstacle in the way (left turn)
    if msg.ranges[0] <= 0.4:
        print('turn on the spot until no obstacle in the way (left turn)')
        move_the_bot.angular.z = 1.0
        move_the_bot.linear.x = 0.0

    elif (msg.ranges[265] > 0.5) and (msg.ranges[275]> 0.5): #maybe not needed 
        print('turn around corner of obstacle')
        move_the_bot.angular.z = -1.0
        move_the_bot.linear.x = 0.05    
    
    elif (msg.ranges[265] - msg.ranges[275]) > 0.005:
        move_the_bot.angular.z = 0.1
        move_the_bot.linear.x = 0
        print('trig correction left')
    
    elif (msg.ranges[265] - msg.ranges[275]) < -0.005:
        move_the_bot.angular.z = -0.1
        move_the_bot.linear.x = 0
        print('trig correction right')
    
    elif msg.ranges[270] > 0.2: #0.3 before
        print('too far away from wall (right turn)')
        move_the_bot.angular.z = - 0.2
        move_the_bot.linear.x = 0.1

    elif msg.ranges[270] < 0.15: #0.2 before
        print('too close to wall (left turn)')
        move_the_bot.angular.z = 0.2
        move_the_bot.linear.x = 0.1   

    else:
        move_the_bot.angular.z = 0 #new
        move_the_bot.linear.x = 0.1 #0.05 before 
        print('fwd')
    
    
    publish_to_cmd_vel.publish(move_the_bot)
    #update the saved point and distance if better 
    update_best_point()

    #check if we arrived at desired point 
    if counter > 20:
        if arrived_at_desired_point(aim):
            print('circled around obstacle')
            #update state to the next 
            state = str(next_state)
        
    counter = counter + 1



    







if __name__ == "__main__":


    rospy.init_node('turtlebot_controller_node')
    subscribe_to_laser = rospy.Subscriber('/scan', LaserScan, callback = laserdata_callback)
    subscribe_to_odom = rospy.Subscriber('/odom', Odometry, callback = odomdata_callback)
    subscribe_to_goal = rospy.Subscriber('/goal', Odometry, callback = goal_callback)
 
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    #create an object of pose data

    move_the_bot = Twist()
    odometry_turtle = Odometry()
    global point 
    global start 
    distance = 1000
    global goal_received
    goal_received = False
    state = 'LINE'
    first_turn = True 

    rospy.spin()


