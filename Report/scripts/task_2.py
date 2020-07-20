# Multiply subgoal's coordinates with resolution and add robot.origin in order
# to translate between the (0,0) of the robot pose and (0,0) of the map
ps.pose.position.x = p[0] * self.robot_perception.resolution + self.robot_perception.origin['x']
ps.pose.position.y = p[1] * self.robot_perception.resolution + self.robot_perception.origin['y']