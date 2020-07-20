# If the robot is in the starting point it immediately sets the next subtarget
if self.next_subtarget == 0:
    self.next_subtarget += 1
    self.counter_to_next_sub = self.count_limit
else:
    # Check if there is a later subtarget, closer than the next one
    # If one is found, make it the next subtarget and update the time
    for i in range( self.next_subtarget + 1, len(self.subtargets) - 1 ):
        # Find the distance between the robot pose and the later subtarget
        dist_from_later = math.hypot(\
            rx - self.subtargets[i][0], \
            ry - self.subtargets[i][1])
        if dist_from_later < 15:
            self.next_subtarget = i
            self.counter_to_next_sub = self.count_limit + 100
            dist = dist_from_later
            break

    # If distance from subtarget is less than 5, go to the next one
    if dist < 5:
        self.next_subtarget += 1
        self.counter_to_next_sub = self.count_limit
        # Check if the final subtarget has been approached
        if self.next_subtarget == len(self.subtargets):
            self.target_exists = False