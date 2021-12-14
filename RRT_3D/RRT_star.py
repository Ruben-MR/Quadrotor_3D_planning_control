import numpy as np
from numpy.linalg import norm
from Obstacle import collision_check_path


# Definition of Class Node
class Node:
    def __init__(self, pos, cost, parent_idx):
        self.pos = pos
        self.cost = cost                # cost from the start to the current node
        self.parent_idx = parent_idx    # parent's node index


# Definition of the RRT* search class
class RRT_star:
    '''
    Given the start pos, goal pos and map boundary,
    RRT* method is used to generate a waypoints list, 
    return the path_list.
    '''

    # Class constructor given an initial pos
    def __init__(self, x_start, num_iter, obstacles, ax, thr=0.5):
        # Set parameters
        self.num_dim = 3        # number of dimensions to search for
        self.thr = thr          # threshold for final goal
        self.pathFind = False   # boolean for found path indication
        self.neigh_dist = 5     # threshold for termination
        self.num_iter = num_iter
        self.goal_idx = 0
        self.obstacle_array = obstacles
        self.ax = ax
        # Add the first node
        self.node_list = [Node(pos=x_start, cost=0, parent_idx=-1)]

    # Method for adding new paths
    def find_path(self, x_goal, map_boundary):
        # Start iteration
        for iteration in range(self.num_iter):
            # get a new sampled point and the index of the closest node in the list
            x_new, idx = self.new_and_closest(map_boundary)

            # check whether the new point is colliding or not
            if idx is None:
                continue
            else:
                # path collision checking, if there is a collision, skip the rest and go to the next iteraion
                if collision_check_path(self.node_list[idx].pos, x_new, self.obstacle_array):
                    continue

            # Rewire the new node for optimal cost and get the close points
            neigh_list = self.rewire_node(x_new, idx, True)

            # Rewire the neighbouring points to the new sample, for existent nodes, pass its pos on the list rather
            # than their pos
            for j in neigh_list:
                self.rewire_node(j, [], False)

            # check arriving to the goal
            if (not self.pathFind) and norm(x_new - x_goal) < self.thr:
                self.pathFind = True
                # Add the final point to the node list
                self.node_list.append(Node(x_goal,
                                           self.node_list[-1].cost+norm(x_goal - x_new),
                                           len(self.node_list) - 1))
                self.goal_idx = len(self.node_list) - 1

            # show the progress
            if (iteration + 1) % 100 == 0:
                print('Search iterations:', iteration+1)

        return self.pathFind

    # Function for obtaining the path
    def get_path(self):
        # back search
        path_list = []
        if self.pathFind:
            # The index of the last element will be equal to the number of elements added
            path_idx = self.goal_idx
            # Iterate backwards appending the nodes and updating the index until the initial one
            while path_idx >= 0:
                path_list.append(self.node_list[path_idx].pos)
                path_idx = self.node_list[path_idx].parent_idx
        else:
            print('No path found')
        return np.flip(np.array(path_list), axis=0)

    # Function for generating a new sample and the index of the closest neighbor
    def new_and_closest(self, map_boundary):
        # random sampling
        x_rand = np.random.uniform(0, 1, 3) * map_boundary

        # check for collision of the sample with an obstacle
        for obstacle in self.obstacle_array:
            if obstacle.collision_check(x_rand):
                return x_rand, None

        # find the nearest node, given that the furthest one will be the goal
        nearest_dis = float('inf')
        for i in range(len(self.node_list)):
            dis = norm(self.node_list[i].pos - x_rand)
            if dis < nearest_dis:
                nearest_node = i
                nearest_dis = dis
        # Return the pos of the newly generated random point and the pos of the nearest node in the list
        return x_rand, nearest_node

    # Function for node rewiring, takes a bool to rewire depending on whether a new sample is given or an existing one
    def rewire_node(self, pos, closest_idx, is_new=True):
        if is_new:
            neigh_list = []
            optimal_neigh = closest_idx
            lowest_cost = self.node_list[closest_idx].cost + norm(pos - self.node_list[closest_idx].pos)

            n = len(self.node_list)
            self.neigh_dist = 7*((np.log(n)/n)**(1/4))

            # Iterate over the list of nodes to find those within distance threshold and in collision-free connection
            for j in range(len(self.node_list)):
                if j == closest_idx:
                    continue
                dist2xnew = norm(pos - self.node_list[j].pos)
                neighbor_cost = self.node_list[j].cost + dist2xnew
                if dist2xnew < self.neigh_dist:
                    if not collision_check_path(self.node_list[j].pos, pos, self.obstacle_array):
                        neigh_list.append(j)
                        if neighbor_cost < lowest_cost:
                            lowest_cost = neighbor_cost
                            optimal_neigh = j

            # add x_new to the tree, the parent of x_new is at index optimal_node
            self.node_list.append(Node(pos, lowest_cost, optimal_neigh))
            return neigh_list

        # If the node is existent, check whether the connection with the new node [at index -1] minimizes the cost
        else:
            node_idx = pos
            rewire_cost = self.node_list[-1].cost + norm(self.node_list[node_idx].pos - self.node_list[-1].pos)
            if rewire_cost < self.node_list[node_idx].cost:
                self.node_list[node_idx].cost = rewire_cost
                self.node_list[node_idx].parent_idx = len(self.node_list) - 1

    def plotTree(self):
        for node in range(1, len(self.node_list)):
            parent_idx = self.node_list[node].parent_idx
            self.ax.plot([self.node_list[parent_idx].pos[0], self.node_list[node].pos[0]],
                         [self.node_list[parent_idx].pos[1], self.node_list[node].pos[1]],
                         [self.node_list[parent_idx].pos[2], self.node_list[node].pos[2]])
