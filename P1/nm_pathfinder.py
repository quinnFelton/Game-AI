from math import inf, sqrt
from heapq import heappop, heappush

def euclidean_distance(point1, point2):
    return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def find_box(point, mesh):

    for box in mesh["boxes"]:
        x1,x2,y1,y2 = box
        if (x1 <= point[0] <= x2) and (y1 <= point[1] <= y2):
            return box

    return None

def find_startFinish_points (source_point, destination_point, mesh):
    source_box = find_box(source_point, mesh)
    destination_box = find_box(destination_point, mesh)
    return source_box, destination_box

def breadth_first_search (source_box, destination_box, mesh): #AI was used for BFS implementation 

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
    x1, x2, y1, y2 = box
    # Ensure the new point is within the bounds of the box
    new_x = max(x1, min(point[0], x2))
    new_y = max(y1, min(point[1], y2))
    return (new_x, new_y)

def detail_path(source_point, path):
    detailed_points = [source_point]
    for i, box in enumerate(path):
        detailed_points.append(make_detail(detailed_points[i-1], box))         # Create a detailed point for each box
    return detailed_points

def reconstruct_box_path(end_box, parents):
    path = []
    current = end_box

    while current is not None:
        path.append(current)
        current = parents[current]

    path.reverse()
    return path


def A_shortest_path(source_point, destination_point, mesh): #editing given dijkstra's algorithm to work with boxes just confused me compared to just rewriting using lecture pseudo code
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

    explored_boxes = set()

    queue = []
    heappush(queue, (euclidean_distance(source_point, destination_point), source_box))

    while queue:
        current_cost, current_box = heappop(queue)

        if current_box in explored_boxes:
            continue

        explored_boxes.add(current_box)
        
        current_cost_true = pathcosts[current_box]

        if current_box == destination_box:
            box_path = reconstruct_box_path(destination_box, parents)
            return box_path, explored_boxes, detail_points

        current_detail = detail_points[current_box]

        for neighbor_box in adjacency.get(current_box, []):
            if neighbor_box == destination_box:
                neighbor_detail = destination_point
            else:
                neighbor_detail = make_detail(current_detail, neighbor_box)

            step_cost = euclidean_distance(current_detail, neighbor_detail)
            cost_to_neighbor = current_cost_true + step_cost  # true cost, not priority

            if neighbor_box not in pathcosts or cost_to_neighbor < pathcosts[neighbor_box]:
                pathcosts[neighbor_box] = cost_to_neighbor
                parents[neighbor_box] = current_box
                detail_points[neighbor_box] = neighbor_detail
                priority = cost_to_neighbor + euclidean_distance(neighbor_detail, destination_point)
                heappush(queue, (priority, neighbor_box))
    return None, explored_boxes, detail_points


def bidirectional_A_star(source_point, destination_point, mesh):
    source_box, destination_box = find_startFinish_points(source_point, destination_point, mesh)
    if source_box is None or destination_box is None:
        return None, [], {}

    if source_box == destination_box:
        return [source_box], [source_box], {
            source_box: source_point,
            destination_box: destination_point
        }
    
    adjacency = mesh.get("adj", {})

    forward_parents = {source_box: None}
    backward_parents = {destination_box: None}

    forward_costs = {source_box: 0}
    backward_costs = {destination_box: 0}

    forward_details = {source_box: source_point}
    backward_details = {destination_box: destination_point}

    detail_points = {}

    explored_boxes = set()

    queue = []
    heappush(queue, (euclidean_distance(source_point, destination_point), source_box, 'destination'))
    heappush(queue, (euclidean_distance(destination_point, source_point), destination_box, 'source'))

    while queue:
        current_cost, current_box, direction = heappop(queue)

        if direction == 'destination':
            parents, pathcosts, details = forward_parents, forward_costs, forward_details
            other_parents = backward_parents
            other_details = backward_details
            target_point = destination_point
        else:
            parents, pathcosts, details = backward_parents, backward_costs, backward_details
            other_parents = forward_parents
            other_details = forward_details
            target_point = source_point
        
        if current_box in explored_boxes:
            continue

        explored_boxes.add(current_box)
        
        current_cost_true = pathcosts[current_box]

        if current_box in other_parents:
            meeting_box = current_box
            forward_half = reconstruct_box_path(meeting_box, forward_parents)        # source ... meeting_box
            backward_half = reconstruct_box_path(meeting_box, backward_parents)      # destination ... meeting_box
            backward_half.reverse()                                                  # meeting_box ... destination

            box_path = forward_half + backward_half[1:]

            meeting_detail = other_details[meeting_box]
            detail_points = {**forward_details, meeting_box: meeting_detail, **backward_details}
            return box_path, explored_boxes, detail_points

        current_detail = details[current_box]

        for neighbor_box in adjacency.get(current_box, []):
            if neighbor_box == (destination_box if direction == 'destination' else source_box):
                neighbor_detail = target_point
            else:
                neighbor_detail = make_detail(current_detail, neighbor_box)

            step_cost = euclidean_distance(current_detail, neighbor_detail)
            cost_to_neighbor = current_cost_true + step_cost  # true cost, not priority

            if neighbor_box not in pathcosts or cost_to_neighbor < pathcosts[neighbor_box]:
                pathcosts[neighbor_box] = cost_to_neighbor
                parents[neighbor_box] = current_box
                details[neighbor_box] = neighbor_detail
                priority = cost_to_neighbor + euclidean_distance(neighbor_detail, target_point)
                heappush(queue, (priority, neighbor_box, direction))
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
    #print("start_box: ", start_box)
    #print("finish_box: ", finish_box)
    if start_box == finish_box:
        return [source_point, destination_point], [start_box]
    check, hold = breadth_first_search(start_box, finish_box, mesh)
    if check is None:
        print("No path found from source to destination with BFS")
        return [],[]
    
    box_path, explored_boxes, detail_points = bidirectional_A_star( source_point, destination_point, mesh)

    if box_path is None:
        print("No path!")
        return [], explored_boxes

    path = detail_path(source_point, box_path)
    path.append(destination_point)

    return path, explored_boxes
