from math import inf, sqrt
from heapq import heappop, heappush

def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    paths = {initial_position: []}          # maps cells to previous cells on path
    pathcosts = {initial_position: 0}       # maps cells to their pathcosts (found so far)
    queue = []
    heappush(queue, (0, initial_position))  # maintain a priority queue of cells
    
    while queue:
        priority, cell = heappop(queue)
        if cell == destination:
            return path_to_cell(cell, paths)
        
        # investigate children
        for (child, step_cost) in adj(graph, cell):
            # calculate cost along this path to child
            cost_to_child = priority + transition_cost(graph, cell, child)
            if child not in pathcosts or cost_to_child < pathcosts[child]:
                pathcosts[child] = cost_to_child            # update the cost
                paths[child] = cell                         # set the backpointer
                heappush(queue, (cost_to_child, child))     # put the child on the priority queue
            
    return False

def path_to_cell(cell, paths):
    if cell == []:
        return []
    return path_to_cell(paths[cell], paths) + [cell]
    



def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    res = []
    for delta in [(x, y) for x in [-1,0,1] for y in [-1,0,1] if not (x==0 and y==0)]:
        new = (cell[0] + delta[0], cell[1] + delta[1])
        if new in level['spaces']:
            res.append((new, transition_cost(level, new, cell)))
    return res

def transition_cost(level, cell, cell2):
    distance = sqrt((cell2[0] - cell[0])**2 + (cell2[1] - cell[1])**2)
    average_cost = (level['spaces'][cell] + level['spaces'][cell2])/2
    return distance * average_cost


def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")

def find_box(point, mesh):
    """
    Finds the box in the mesh that contains the point

    Args:
        point: a point to find in the mesh
        mesh: pathway constraints the path adheres to

    Returns:
        The box that contains the point if exists
        None if no box contains the point
    """

    for box in mesh["boxes"]:
        x1,x2,y1,y2 = box
        if (x1 <= point[0] <= x2) and (y1 <= point[1] <= y2):
            return box

    return None

def find_startFinish_points (source_point, destination_point, mesh):
    source_box = find_box(source_point, mesh)
    destination_box = find_box(destination_point, mesh)
    return source_box, destination_box

def breadth_first_search (source_box, destination_box, mesh):
    """
    Searches for a path from source_box to destination_box through the mesh.

    Args:
        source_box: starting box of the pathfinder
        destination_box: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:
        A tuple (box_path, explored_boxes) where box_path is a list of boxes
        from source_box to destination_box, or None if there is no path.
        explored_boxes is the list of boxes visited by the search in order.
    """

    if source_box is None or destination_box is None:
        return None, []

    if source_box == destination_box:
        return [source_box], [source_box]

    from collections import deque

    adjacency = mesh.get("adj", {})
    queue = deque([source_box])
    parents = {source_box: None}
    explored_boxes = [source_box]

    while queue:
        current = queue.popleft()
        for neighbor in adjacency.get(current, []):
            if neighbor not in parents:
                parents[neighbor] = current
                explored_boxes.append(neighbor)
                if neighbor == destination_box:
                    # Reconstruct path from source to destination
                    path = [destination_box]
                    while parents[path[-1]] is not None:
                        path.append(parents[path[-1]])
                    path.reverse()
                    return path, explored_boxes
                queue.append(neighbor)

    return None, explored_boxes

def _box_center(box):
    x1, x2, y1, y2 = box
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

def make_detail(point, box):
    """
    Creates a detailed point within the box based on the given point.

    Args:
        point: a point to refine within the box
        box: the box that contains the point

    Returns:
        A new point that is more detailed within the box.
    """
    x1, x2, y1, y2 = box
    # Ensure the new point is within the bounds of the box
    new_x = max(x1, min(point[0], x2))
    new_y = max(y1, min(point[1], y2))
    return (new_x, new_y)

def detail_path(source_point, path):
    """
    Refines the path by creating detailed points within each box.

    Args:
        source_point: the starting point of the path
        path: a list of boxes representing the path

    Returns:
        A list of detailed points representing the refined path.
    """
    detailed_points = [source_point]
    for i, box in enumerate(path, start=1):
        # Create a detailed point for each box (e.g., the closest point to the previous detailed point within the box)
        detailed_points.append(make_detail(detailed_points[i-1], box))
    return detailed_points

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    start_box, finish_box = find_startFinish_points(source_point, destination_point, mesh)
    path = []
    boxes = {start_box, finish_box}
    print("start_box: ", start_box)
    print("finish_box: ", finish_box)
    boxes, hold = breadth_first_search(start_box, finish_box, mesh)
    if boxes is None:
        print("No path found from source to destination.")
    """
    if (start_box == None or finish_box == None):
        return Exception
    
    for box in boxes:
        path.append(_box_center(box))
    """
    boxes = dijkstras_shortest_path(start_box, finish_box, mesh)
    path = detail_path(source_point, boxes)
    path.append(destination_point)
    return path, boxes
