#!/usr/bin/env python3
import numpy as np
import pandas as pd
import rospy 
import math 
import matplotlib.pyplot as plt


from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion 

def odomdata_callback(msg):

    global odometry_turtle; odometry_turtle = msg 
    global linex
    global liney
    linex.append(msg.pose.pose.position.x)
    liney.append(msg.pose.pose.position.y)


def goal_callback(msg):
    global goal; goal = msg.pose.pose.position
    global goal_received; goal_received = False
    if abs(msg.pose.pose.position.y) > 2.7 or abs(msg.pose.pose.position.x) > 2.7:
        print('goal out of range, please enter new value')
    else: 
        goal_received = True 

def laserdata_callback(msg):
    
    #5 Hz, 1 = 11,5 grad
    global state
    global point
    global distance 
    global start 
    global goal
    global goal_received 
    global counter
    global first
    global linex
    global liney
    global reachable 

    #Prints the length of ranges array, in our case there are total 360 readings, one reading for each 1 degree

    #print(len(msg.ranges)) 

    #To make use of laser data, we will need directional readings. For example, front, back, left and right of the robot. In our case, 0: Front, 180: Back, 270: Right, 270: Left,  are directions of laserbeam for robot

    #print(msg.ranges[0], msg.ranges[270], msg.ranges[180], msg.ranges[270])
    
    # sample head collision avoidance algorithm(You will have to write your algorithm here)
    
    #checks if goal and postiton and orientation of the turtle bot is given
    if not (goal_received):
        print('waiting for goal')
        reachable = odometry_turtle.pose.pose.position
        return

    
    # #3 modes: LINE: moving along line until, OBSTACLE: suring obstacle, LEAVE: leaving obstacle at optimal point 
    if arrived_at_desired_point(goal,0.05) == True:
        
        print('arrived at goal', first)
        move_the_bot.linear.x = 0.0
        move_the_bot.angular.z = 0.0
        publish_to_cmd_vel.publish(move_the_bot)
        
        if first:
            print('export and plot')
            pd.Series(linex).to_csv('//home/friederike/catkin_ws/src/plots/linex2.csv')
            pd.Series(liney).to_csv('/home/friederike/catkin_ws/src/plots/liney2.csv')
            return
        return

    if state == 'LINE':
  
        counter = 0
        #calculate desired heading of robot
        anglular_z = calculate_heading()
        #move robot to desired heading
        move_the_bot.angular.z = anglular_z
        move_the_bot.linear.x = 0.0
        publish_to_cmd_vel.publish(move_the_bot)
        #check if no obstacle in the way if no move forward
        if  min(msg.ranges[315:]+ msg.ranges[:45]) > 0.2: # and msg.ranges[380] > 0.5 and msg.ranges[10] > 0.5  :
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

        navigate_a_obstacle(start, msg, 'LEAVE')


    if state == 'LEAVE':

        print ('LEAVE to', point)

        if (arrived_at_desired_point(reachable, 0.05)):
            print ('Goal not reachable')
            return

        reachable = point 
        navigate_a_obstacle(point, msg, 'LINE')

    

def calculate_heading():
    dx = goal.x - odometry_turtle.pose.pose.position.x
    dy = goal.y - odometry_turtle.pose.pose.position.y
    desired_angle = math.atan(dy/dx)
    if (dx < 0):
        if(dy>0):
            desired_angle = math.pi + math.atan(dy/dx)
        else:
            desired_angle = -math.pi + math.atan(dy/dx)
            
    euler = euler_from_quaternion([odometry_turtle.pose.pose.orientation.x, odometry_turtle.pose.pose.orientation.y, odometry_turtle.pose.pose.orientation.z, odometry_turtle.pose.pose.orientation.w])
    
    a = desired_angle - euler[2]
    if (a>180):
        a -= 360
    if(a<-180):
        a += 360
    return a

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

def arrived_at_desired_point(location, threshold):
    if distance_to_goal(location) < threshold:
        return True
    else: 
        return False 

def navigate_a_obstacle(aim, msg, next_state):
    global state 
    global start 
    global counter 

    diff_par = sum(msg.ranges[(264):(267)])/len(msg.ranges[(264):(267)]) - sum(msg.ranges[(274):(277)])/len(msg.ranges[(274):(277)])
    
    if (counter == 5):
        start = odometry_turtle.pose.pose.position
        print('start' , start)
    #turn on the spot until no obstacle in the way (left turn)

    if min(msg.ranges[340:]+ msg.ranges[:40]) <= 0.2:
        print('turn on the spot until no obstacle in the way (left turn)')
        move_the_bot.angular.z = 2 #cube = 1
        move_the_bot.linear.x = 0.0


    elif min(msg.ranges[(290):(315)]) > 1: #0.3 before
    
        print('front right no contact')
        
        move_the_bot.angular.z = - 0.7
        move_the_bot.linear.x = 0.07

    elif diff_par > 0.008:    
        move_the_bot.angular.z = 0.2 #min (0.7, 0.2 * 100 * diff_par)
        move_the_bot.linear.x = 0
        print('trig correction left')
    
    elif diff_par < -0.008:
        print('trig correction right')
        move_the_bot.angular.z = -0.3
        move_the_bot.linear.x = 0.0

    elif min(msg.ranges[269:271]) < 0.15: 
        print('too close to wall (left turn)')
        move_the_bot.angular.z = 0.4
        move_the_bot.linear.x = 0.1
    
    elif max(msg.ranges[269:271]) > 0.2: #0.3 before
        print('too far away from wall (right turn)')
        move_the_bot.angular.z = - 0.4
        move_the_bot.linear.x = 0.1

    else:
        move_the_bot.angular.z = 0 #new
        move_the_bot.linear.x = 0.05 #0.05 before 
        print('fwd')
    
    
    publish_to_cmd_vel.publish(move_the_bot)
    #update the saved point and distance if better 
    update_best_point()

    #check if we arrived at desired point 
    if counter > 100:
        if arrived_at_desired_point(aim,0.1):
            print('circled a obstacle')
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
    linex =[]
    liney =[]
    global point 
    global start 
    distance = 1000
    global goal_received
    goal_received = False
    state = 'LINE'
    first = True 
    global reachable 
    counter = 0 

    rospy.spin()


