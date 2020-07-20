# Get the robot pose in pixels
[rx, ry] = [\
    robot_pose['x_px'] - \
            origin['x'] / resolution,\
    robot_pose['y_px'] - \
            origin['y'] / resolution\
            ]
g_robot_pose = [rx,ry]
# If nodes > 25 the calculation time-cost is very big
# In order to avoid time-reset we perform sampling on
# the nodes list and take a half-sized sample 
for i in range(0,len(nodes)):
    nodes[i].append(i)
if (len(nodes) > 25):
    nodes = random.sample(nodes, int(len(nodes)/2))

# Initialize weigths
w_dist = [0]*len(nodes)
w_rot = [robot_pose['th']]*len(nodes)
w_cov = [0]*len(nodes)
w_topo = [0]*len(nodes)
# Calculate weights for every node in the topological graph
for i in range(0,len(nodes)):
    # If path planning fails then give extreme values to weights
    path = self.path_planning.createPath(g_robot_pose, nodes[i], resolution)
    if not path:
        w_dist[i] = sys.maxsize
        w_rot[i] = sys.maxsize
        w_cov[i] = sys.maxsize
        w_topo[i] = sys.maxsize
    else:
        for j in range(1,len(path)):
            # Distance cost
            w_dist[i] += math.hypot(path[j][0] - path[j-1][0], path[j][1] - path[j-1][1])
            # Rotational cost
            w_rot[i] += abs(math.atan2(path[j][0] - path[j-1][0], path[j][1] - path[j-1][1]))
            # Coverage cost
            w_cov[i] += coverage[int(path[j][0])][int(path[j][1])] / (len(path))
        # Add the coverage cost of 0-th node of the path
        w_cov[i] += coverage[int(path[0][0])][int(path[0][1])] / (len(path))
        # Topological cost
        # This metric depicts wether the target node
        # is placed in open space or near an obstacle
        # We want the metric to be reliable so we also check node's neighbour cells
        w_topo[i] += brush[nodes[i][0]][nodes[i][1]]
        w_topo[i] += brush[nodes[i][0]-1][nodes[i][1]]
        w_topo[i] += brush[nodes[i][0]+1][nodes[i][1]]
        w_topo[i] += brush[nodes[i][0]][nodes[i][-1]]
        w_topo[i] += brush[nodes[i][0]][nodes[i][+1]]
        w_topo[i] += brush[nodes[i][0]-1][nodes[i][1]-1]
        w_topo[i] += brush[nodes[i][0]+1][nodes[i][1]+1]
        w_topo[i] = w_topo[i]/7

# Normalize weights between 0-1
for i in range(0,len(nodes)):
    w_dist[i] = 1 - (w_dist[i]-min(w_dist))/(max(w_dist)-min(w_dist))
    w_rot[i] = 1 - (w_rot[i]-min(w_rot))/(max(w_rot)-min(w_rot))
    w_cov[i] = 1 - (w_cov[i]-min(w_cov))/(max(w_cov)-min(w_cov))
    w_topo[i] = 1 - (w_topo[i]-min(w_topo))/(max(w_topo)-min(w_topo))

# Set cost values
# We set each cost's priority based on experimental results
# from "Cost-Based Target Selection Techniques Towards Full Space
# Exploration and Coverage for USAR Applications
# in a Priori Unknown Environments" publication
C1 = w_topo
C2 = w_dist
C3 = [1]*len(nodes)
for i in range(0,len(nodes)):
    C3[i] -= w_cov[i]
C4 = w_rot
# Priority Weight
C_PW = [0]*len(nodes)
# Smoothing Factor
C_SF = [0]*len(nodes)
# Target's Final Priority
C_FP = [0]*len(nodes)
for i in range(0,len(nodes)):
    C_PW[i] = 2**3*(1-C1[i])/.5 + 2**2*(1-C2[i])/.5 + 2**1*(1-C3[i])/.5 + 2**0*(1-C4[i])/.5
    C_SF[i] = (2**3*(1-C1[i]) + 2**2*(1-C2[i]) + 2**1*(1-C3[i]) + 2**0*(1-C4[i]))/(2**4 - 1)
    C_FP[i] = C_PW[i]*C_SF[i]

# Select the node with the smallest C_FP value
val, idx = min((val, idx) for (idx, val) in enumerate(C_FP))
target = nodes[idx]