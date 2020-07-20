# We use a motor schema in order to combine velocities from 
# obstacle avoidance and path following procedure
if l_laser == 0:
    c_l = 0
    c_a = 1
else:
    c_l = 0.2
    c_a= 0.2

self.linear_velocity = l_goal + c_l*l_laser
self.angular_velocity = a_goal + c_a*a_laser
