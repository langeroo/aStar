from graph_node import node;
from graph import gridgraph;
import numpy as np;
import heapq;
import cv2;

def load_maze(maze_file_name, kernel_size):
    gray_image = cv2.imread(maze_file_name, cv2.IMREAD_GRAYSCALE);

    kernel = np.ones((kernel_size, kernel_size),np.uint8)
    erosion = cv2.erode(gray_image,kernel,iterations = 1)

    maze_calc = np.array(erosion); #eroded white space pushes the path away from the walls
    maze_disp = np.array(gray_image);
    return maze_calc, maze_disp;

def disp_path(path, grid, visited, maze_save_name):
    for cell in visited:
        grid[cell[0], cell[1]] = 255 - 50;

    for step in path:
        grid[step[0], step[1]] = 25;

    grid_np = np.array(grid);
    cv2.imshow('Maze',grid_np);
    cv2.waitKey(0);
    cv2.destroyAllWindows();
    cv2.imwrite(maze_save_name, grid_np);

def main():

    edge_cost = 1;

    maze_name = 'maze3';
    maze_file_name = 'mazes/' + maze_name + '.jpg';
    maze_save_name = 'solved_mazes/solved_' + maze_name + '.jpg';
    kernel_size = 2;

    grid_calc, grid_disp = load_maze(maze_file_name, kernel_size)
    h,w = grid_calc.shape;

    start = [2,2];
    finish = [h-2,w-2];
    curr = start;

    print "Converting image to graph..."
    graph = gridgraph(grid_calc, finish);
    graph.generate_graph(edge_cost);
    print "Graph generated.\n"

    print "Beginning search..."
    failed = False;

    priQ = []; #priority queue -- heap
    graph.set_g(curr, 0);
    while not curr == finish:
        curr_neighbors = graph.get_neighbors(curr);
        curr_costs = graph.get_costs(curr);
        curr_g = graph.get_g(curr);

        for i in range(0,len(curr_neighbors)):
            neighb = curr_neighbors[i];
            cost = curr_costs[i];
            neighb_g0 = graph.get_g(neighb);
            curr_g = graph.get_g(curr) + cost;

            if curr_g < neighb_g0:
                graph.set_g(neighb, curr_g);
                graph.set_parent(neighb, curr)
                f = graph.calc_f(neighb)

                costNode = (f, neighb)
                heapq.heappush(priQ, costNode)
        if len(priQ) > 0:
            curr = heapq.heappop(priQ)[1];
        else:
            curr = finish;
            failed = True;
            print "!!! No Path To Goal !!!"

    path = graph.traceback_non_recursive(curr);
    # path = graph.traceback(curr);

    if not failed:
        print "Search complete! Path found!"

    visited = graph.get_visited();
    disp_path(path, grid_disp, visited, maze_save_name)

if __name__ == "__main__":
    main()

