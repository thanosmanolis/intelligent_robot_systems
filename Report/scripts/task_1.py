leng = len(scan)    # Number of scans
angle_min = self.laser_aggregation.angle_min    # Minimum angle scanned (rad)
angle_increment = self.laser_aggregation.angle_increment  # Angle between different scans (rad)

# Calculate the effect every single scan has on the path that the robot is 
# going to follow. The closer it is, the bigger the effect. We use sin and 
# cos so that we can get positive or negative values, in order for the robot 
# to move front/back and right/left
for i in range(0, leng):
    linear -= math.cos(angle_min + i*angle_increment) / (scan[i]*scan[i])
    angular -= math.sin(angle_min + i*angle_increment) / (0.5*scan[i]*scan[i])

# Get the average value of all scans' effect
linear = 0.3 + linear / leng  # In this case add it to something constant
angular = angular / leng

# Both speeds must have values in the range [-0.3, 0.3]
linear = min(max(linear, -0.3), 0.3)
angular = min(max(angular, -0.3), 0.3)