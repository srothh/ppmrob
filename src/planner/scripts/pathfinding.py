
import heapq


def heuristic(a, b):
    """
    Calculate the Manhattan distance between two points
    """
    return abs(b[0] - a[0]) + abs(b[1] - a[1])


def a_star(occupancy_grid, start, goal):
    """
    A* Pathfinding algorithm on a grid.

    Args:
    - occupancy_grid: 2D list representing the grid (0=free, 1=occupied)
    - start: Tuple (row, col) representing the start cell
    - goal: Tuple (row, col) representing the target cell

    Returns:
    - List of tuples as the path from start to goal
    """
    # Priority queue for currently explored paths, sorted by total cost
    frontier = []
    heapq.heappush(frontier, (0, start))

    # Dictionaries for backtracking and cost tracking
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        current = heapq.heappop(frontier)[1]

        if current == goal:
            break

        # Explore neighbors
        for next in neighbors(occupancy_grid, current):
            new_cost = cost_so_far[current] + 1  # Assuming cost between neighbors is always 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                heapq.heappush(frontier, (priority, next))
                came_from[next] = current

    # Reconstruct path from goal to start (backtracking)
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()  # Reverse the path to start to goal order
    return path


def neighbors(grid, current):
    """
    Returns the list of accessible neighbors for a cell in the grid.
    """
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4-way connectivity: up, right, down, left
    result = []
    for direction in directions:
        neighbor = (current[0] + direction[0], current[1] + direction[1])
        if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and (grid[neighbor[0]][neighbor[1]] == 0 or grid[neighbor[0]][neighbor[1]] == 1):
            result.append(neighbor)
    return result