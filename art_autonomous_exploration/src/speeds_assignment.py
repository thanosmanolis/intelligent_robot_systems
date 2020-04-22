#!/usr/bin/env python

import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds
class RobotController:

    # Constructor
    def __init__(self):

      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):

      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
      scan = self.laser_aggregation.laser_scan
      linear  = 0
      angular = 0

      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance

      leng = len(scan)                                          # Number of scans
      angle_min = self.laser_aggregation.angle_min              # Minimum angle scanned (rad)
      angle_increment = self.laser_aggregation.angle_increment  # Angle between different scans (rad)

      # Calculate the effect every single scan has on the path that the robot is going to follow.
      # The closer it is, the bigger the effect. We use sin and cos so that we can get positive or
      # negative values, in order for the robot to move front/back and right/left
      for i in range(0, leng):
          linear -= math.cos(angle_min + i*angle_increment) / scan[i]**2
          angular -= math.sin(angle_min + i*angle_increment) / (0.5*scan[i]**2)

      # Get the average value of all scans' effect
      linear = 0.3 + linear / leng  # In this case add it to something constant
      angular = angular / leng

      # Both speeds must have values in the range [-0.3, 0.3]
      linear = min(max(linear, -0.3), 0.3)
      angular = min(max(angular, -0.3), 0.3)

      ##########################################################################

      return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):

      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()

      # You must fill these
      self.linear_velocity  = 0
      self.angular_velocity = 0

      if self.move_with_target == True:
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.

        # We use a motor schema in order to combine velocities from obstacle
        # avoidance and path following procedure
        c_l = 0.2
        c_a= 0.2

        self.linear_velocity = l_goal + c_l*l_laser
        self.angular_velocity = a_goal + c_a*a_laser

        ##########################################################################
      else:
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant

        self.linear_velocity = l_laser
        self.angular_velocity = a_laser

        ##########################################################################

    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
