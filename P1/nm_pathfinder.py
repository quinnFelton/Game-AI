from math import inf, sqrt
from heapq import heappop, heappush

def euclidean_distance(point1, point2):
    """ Calculates the Euclidean distance between two points.

    Args:
        point1: A tuple representing the first point (x1, y1).
        point2: A tuple representing the second point (x2, y2).

    Returns:
        The Euclidean distance between point1 and point2.
    """
    return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

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
    for i, box in enumerate(path):
        # Create a detailed point for each box (e.g., the closest point to the previous detailed point within the box)
        detailed_points.append(make_detail(detailed_points[i-1], box))
    return detailed_points

def reconstruct_box_path(end_box, parents):
    """
    Reconstructs the sequence of boxes from start to end.
    """
    path = []
    current = end_box

    while current is not None:
        path.append(current)
        current = parents[current]

    path.reverse()
    return path


def dijkstras_shortest_path(source_point, destination_point, mesh):
    """
    Runs Dijkstra's algorithm over navmesh boxes.

    The nodes are boxes.
    The edges come from mesh["adj"].
    The edge costs come from distances between detail points.
    """
    source_box, destination_box = find_startFinish_points(source_point, destination_point, mesh)
    if source_box is None or destination_box is None:
        return None, [], {}

    if source_box == destination_box:
        return [source_box], [source_box], {
            source_box: source_point,
            destination_box: destination_point
        }

    adjacency = mesh.get("adj", {})

    parents = {source_box: None}
    pathcosts = {source_box: 0}

    # detail_points[box] is the exact point used inside that box.
    detail_points = {source_box: source_point}

    explored_boxes = []

    queue = []
    heappush(queue, (0, source_box))

    while queue:
        current_cost, current_box = heappop(queue)

        # Ignore stale queue entries.
        # This matters because the same box can be pushed multiple times
        # if a cheaper path is discovered later.
        if current_cost != pathcosts[current_box]:
            continue

        explored_boxes.append(current_box)

        if current_box == destination_box:
            box_path = reconstruct_box_path(destination_box, parents)
            box_path.append(destination_box)
            return box_path, explored_boxes, detail_points

        current_detail = detail_points[current_box]

        for neighbor_box in adjacency.get(current_box, []):
            if neighbor_box == destination_box:
                neighbor_detail = destination_point
            else:
                neighbor_detail = make_detail(current_detail, neighbor_box)

            step_cost = euclidean_distance(current_detail, neighbor_detail)
            cost_to_neighbor = current_cost + step_cost

            if neighbor_box not in pathcosts or cost_to_neighbor < pathcosts[neighbor_box]:
                pathcosts[neighbor_box] = cost_to_neighbor
                parents[neighbor_box] = current_box
                detail_points[neighbor_box] = neighbor_detail
                heappush(queue, (cost_to_neighbor, neighbor_box))

    return None, explored_boxes, detail_points

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
    if start_box == finish_box:
        return [source_point, destination_point], [start_box]
    check, hold = breadth_first_search(start_box, finish_box, mesh)
    if check is None:
        print("No path found from source to destination with BFS")
        return [],[]
    """
    if (start_box == None or finish_box == None):
        return Exception
    
    for box in boxes:
        path.append(_box_center(box))
    
    boxes = dijkstras_shortest_path(start_box, finish_box, mesh)
    path = detail_path(source_point, boxes)
    path.append(destination_point)
    """
    box_path, explored_boxes, detail_points = dijkstras_shortest_path( start_box, finish_box, mesh)

    if box_path is None:
        print("No path!")
        return [], explored_boxes

    path = detail_path(source_point, box_path)
    path.append(destination_point)

    return path, box_path
