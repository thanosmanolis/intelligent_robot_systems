# Angular and linear velocities are calculated upon the math
# types presented at 9.exploration_target_selection.pdf

dtheta = math.degrees(math.atan2(st_y - ry, st_x - rx)) - math.degrees(theta)

if dtheta >= 0 and dtheta < 180:
    angular = dtheta / 180
elif dtheta > 0 and dtheta >= 180:
    angular = (dtheta - 2 * 180) / 180
elif dtheta <= 0 and dtheta > -180:
    angular =  dtheta / 180
elif dtheta < 0 and dtheta < -180:
    angular = (dtheta + 2 * 180) / 180

# Should be angular*0.3 but then the robot moves very slow and 
# cannot reach the targets within the time set from the timer
if angular > 0.3:
    angular = 0.3
elif angular < -0.3:
    angular = -0.3

# **20 is used to avoid overshoot problem
linear = ( (1 - abs(angular))**20 ) * 0.3