class graph_node:
    def __init__(self, neighbors, costs, position):
        self.neighbors = neighbors;
        self.costs = costs;
        self.position = position;
        self.parent = None;
        self.g = 0; #cost of the path from the start node to this node
        self.h = 0; #estimated cost from this node to the end node
        self.dimension = len(position);

    def get_pos(self):
        return self.position;

    def get_cost(self):
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

    def calculate_h(self, end_node_position):
        result = 0;
        for i in range(1,dim):
            result += self.position[i] - end_node_position[i];
        self.h = result;
        return self.h;