#!/usr/bin/env python

import rospy
import random
import sys
import math
import time
import numpy as np
from timeit import default_timer as timer
from utilities import RvizHandler
from utilities import OgmOperations
from utilities import Print
from brushfires import Brushfires
from topology import Topology
import scipy
from path_planning import PathPlanning


# Class for selecting the next best target
class TargetSelection:

    # Constructor
    def __init__(self, selection_method):
        self.goals_position = []
        self.goals_value = []
        self.omega = 0.0
        self.radius = 0
        self.method = selection_method

        self.brush = Brushfires()
        self.topo = Topology()
        self.path_planning = PathPlanning()


    def selectTarget(self, init_ogm, coverage, robot_pose, origin, \
        resolution, force_random = False):

        target = [-1, -1]

        ######################### NOTE: QUESTION  ##############################
        # Implement a smart way to select the next target. You have the
        # following tools: ogm_limits, Brushfire field, OGM skeleton,
        # topological nodes.

        # Find only the useful boundaries of OGM. Only there calculations
        # have meaning
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution)

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)

        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, \
                   origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)

        # Visualization of topological nodes
        vis_nodes = []
        for n in nodes:
            vis_nodes.append([
                n[0] * resolution + origin['x'],
                n[1] * resolution + origin['y']
            ])
        RvizHandler.printMarker(\
            vis_nodes,\
            1, # Type: Arrow
            0, # Action: Add
            "map", # Frame
            "art_topological_nodes", # Namespace
            [0.3, 0.4, 0.7, 0.5], # Color RGBA
            0.1 # Scale
        )

        # Random point
        # if self.method == 'random' or force_random == True:
        #   target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
        ########################################################################

        # Get the robot pose in pixels
        [rx, ry] = [\
            robot_pose['x_px'] - \
                    origin['x'] / resolution,\
            robot_pose['y_px'] - \
                    origin['y'] / resolution\
                    ]
        g_robot_pose = [rx,ry]
        # If nodes > 15 the calculation time-cost is very big
        # In order to avoid time-reset we set a maximum threshold of 15 possible targets
        # by sampling the topological graph's nodes
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
        print "Selected Target - Node: " + str(nodes[idx][2])
        target = nodes[idx]
        return target



    def selectRandomTarget(self, ogm, coverage, brushogm, ogm_limits):
      # The next target in pixels
        tinit = time.time()
        next_target = [0, 0]
        found = False
        while not found:
          x_rand = random.randint(0, ogm.shape[0] - 1)
          y_rand = random.randint(0, ogm.shape[1] - 1)
          if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and \
              brushogm[x_rand][y_rand] > 5:
            next_target = [x_rand, y_rand]
            found = True
        Print.art_print("Select random target time: " + str(time.time() - tinit), \
            Print.ORANGE)
        return next_target
