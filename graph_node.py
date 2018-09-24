import numpy as np;

class node:
    def __init__(self, neighbors, costs, position):
        self.neighbors = neighbors;
        self.costs = costs;
        self.position = position;
        self.parent = None;
        self.g = float('Inf'); #cost of the path from the start node to this node
        self.h = None; #estimated cost from this node to the end node
        self.dimension = len(position);

    def get_pos(self):
        return self.position;

    def get_costs(self):
        return self.costs;

    def get_neighbors(self):
        return self.neighbors;

    def get_parent(self):
        return self.parent;

    def set_parent(self, parent):
        self.parent = parent;
        return self.parent;

    def set_g(self, g):
        self.g = g;
        return self.g;

    def get_g(self):
        return self.g;

    def calculate_h(self, end_node_position):
        if self.h == None:
            result = 0;
            for i in range(0,self.dimension):
                result += (np.abs(self.position[i] - end_node_position[i])); #manhattan distance
            self.h = result;
        return self.h;

    def calc_f(self, end_node_position):
        return self.calculate_h(end_node_position) + self.g;