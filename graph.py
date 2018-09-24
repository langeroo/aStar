import numpy as np;
from graph_node import node;

class gridgraph:
    def __init__(self, grid, finish): #raph_list, graph_list):
        self.grid = grid;
        self.dim0 = grid.shape[0]
        self.dim1 = grid.shape[1]
        self.nodeDict = None;
        self.visitedDict = {};
        self.endNode = finish;

    def generate_graph(self, edge_cost):
        myNodeDict = {};
        obstacle_thresh_val = 25;
        for i in range(0, self.dim0):
            for j in range(0, self.dim1):
                cell = self.grid[i,j];
                position = [i,j];
                neighbors = [];
                costs = [];
                if not cell == 0:
                    topIsCell = not(i == 0);
                    bottomIsCell = not(i == self.dim0 - 1);
                    leftIsCell = not(j == 0);
                    rightIsCell = not(j == self.dim1 - 1);
                    if topIsCell:
                        if not self.grid[i-1,j] <= obstacle_thresh_val:
                            neighbors.append([i-1,j]);
                            costs.append(edge_cost);
                        #top left
                        if leftIsCell:
                            if not self.grid[i-1, j-1] <= obstacle_thresh_val:
                                neighbors.append([i-1, j-1]);
                                costs.append(edge_cost * np.sqrt(2));
                        #top right
                        if rightIsCell:
                            if not self.grid[i-1, j+1] <= obstacle_thresh_val:
                                neighbors.append([i-1, j+1]);
                                costs.append(edge_cost * np.sqrt(2));
                    if bottomIsCell: 
                        if not self.grid[i+1,j] <= obstacle_thresh_val:
                            neighbors.append([i+1,j]);
                            costs.append(edge_cost);
                        #top left
                        if leftIsCell:
                            if not self.grid[i+1, j-1] <= obstacle_thresh_val:
                                neighbors.append([i+1, j-1]);
                                costs.append(edge_cost * np.sqrt(2));
                        #top right
                        if rightIsCell:
                            if not self.grid[i+1, j+1] <= obstacle_thresh_val:
                                neighbors.append([i+1, j+1]);
                                costs.append(edge_cost * np.sqrt(2));
                    if leftIsCell:
                        if not self.grid[i,j-1] <= obstacle_thresh_val:
                            neighbors.append([i,j-1]);
                            costs.append(edge_cost);
                    if rightIsCell:
                        if not self.grid[i,j+1] <= obstacle_thresh_val:
                            neighbors.append([i,j+1]);
                            costs.append(edge_cost);
                
                node_out = node(neighbors, costs, position);
                idx = self.coords_to_idx([i,j]);
                myNodeDict[idx] = node_out;
                self.nodeDict = myNodeDict;

    def get_neighbors(self, coords):
        idx = self.coords_to_idx(coords);
        self.visitedDict[idx] = coords;

        node = self.coords_to_node(coords);
        return node.get_neighbors();

    def get_costs(self, coords):
        node = self.coords_to_node(coords);
        return node.get_costs();

    def set_g(self, coords, g):
        node = self.coords_to_node(coords);
        return node.set_g(g);

    def get_g(self, coords):
        node = self.coords_to_node(coords);
        return node.get_g();

    def set_parent(self, coords, parent):
        node = self.coords_to_node(coords);
        return node.set_parent(parent);

    def calc_f(self, coords):
        node = self.coords_to_node(coords);
        return node.calc_f(self.endNode);

    def coords_to_idx(self, coords):
        return coords[0] * self.dim0 + coords[1];

    def coords_to_node(self, coords):
        idx = self.coords_to_idx(coords);
        node = self.nodeDict[idx];
        return node;

    def get_visited(self):
        return self.visitedDict.values();

    def traceback(self, coords):
        path = [];
        node = self.coords_to_node(coords);
        if node.get_parent() == None:
            path.append(coords);
        else:
            path = self.traceback(node.get_parent());
            path.append(coords);
        return path;

    def traceback_non_recursive(self, coords):
        path = [];
        node = self.coords_to_node(coords);
        while not node.get_parent() == None:
            path.append(coords);
            coords = node.get_parent();
            node = self.coords_to_node(coords);
        return reversed(path);

    def print_graph(self):
        for n in self.nodeDict:
            node = self.nodeDict[n];
            print 'xy:' , node.get_pos(), '\tneighbors: ', node.get_neighbors();













