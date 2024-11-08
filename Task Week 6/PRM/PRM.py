import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
import heapq

def generate_random_points(num_points, x_range, y_range):
    """
    Generate random points within a specified range.
    
    Args:
        num_points (int): Number of points to generate.
        x_range (tuple): Minimum and maximum x-coordinates.
        y_range (tuple): Minimum and maximum y-coordinates.
    
    Returns:
        numpy.ndarray: Array of generated points.
    """
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    y = np.random.uniform(y_range[0], y_range[1], num_points)
    return np.column_stack((x, y))

def build_graph(points, radius):
    """
    Build a graph by connecting points within a specified radius.
    
    Args:
        points (numpy.ndarray): Array of points.
        radius (float): Maximum distance for connecting points.
    
    Returns:
        dict: Graph represented as an adjacency list.
    """
    distances = cdist(points, points)
    graph = {i: [] for i in range(len(points))}
    
    for i in range(len(points)):
        for j in range(i+1, len(points)):
            if distances[i, j] <= radius:
                graph[i].append(j)
                graph[j].append(i)
    
    return graph

def dijkstra_shortest_path(graph, start, goal):
    """
    Find the shortest path between start and goal using Dijkstra's algorithm.
    
    Args:
        graph (dict): Graph represented as an adjacency list.
        start (int): Index of the start node.
        goal (int): Index of the goal node.
    
    Returns:
        list: Sequence of node indices in the shortest path.
    """
    # Initialize distances and previous nodes
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    previous = {node: None for node in graph}
    
    # Create a priority queue
    pq = [(0, start)]
    
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        
        # If we've reached the goal, reconstruct the path
        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = previous[current_node]
            return path[::-1]
        
        # Skip if we've already found a shorter path
        if current_distance > distances[current_node]:
            continue
        
        # Update distances and previous nodes for neighbors
        for neighbor in graph[current_node]:
            distance = current_distance + 1  # Assuming unit edge weights
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))
    
    # No path found
    return None

def visualize_prm(points, graph, start, goal):
    """
    Visualize the Probabilistic Roadmap (PRM).
    
    Args:
        points (numpy.ndarray): Array of points.
        graph (dict): Graph represented as an adjacency list.
        start (int): Index of the start node.
        goal (int): Index of the goal node.
    """
    # Plot the points
    plt.scatter(points[:, 0], points[:, 1], s=50, c='gray')
    
    # Plot the edges
    for node, neighbors in graph.items():
        for neighbor in neighbors:
            plt.plot([points[node, 0], points[neighbor, 0]],
                    [points[node, 1], points[neighbor, 1]],
                    c='lightgray', linewidth=1)
    
    # Plot the start and goal points
    plt.scatter(points[start, 0], points[start, 1], s=100, c='green')
    plt.scatter(points[goal, 0], points[goal, 1], s=100, c='red')
    
    # Find and plot the shortest path
    path = dijkstra_shortest_path(graph, start, goal)
    if path:
        for i in range(len(path) - 1):
            plt.plot([points[path[i], 0], points[path[i+1], 0]],
                    [points[path[i], 1], points[path[i+1], 1]],
                    c='blue', linewidth=2)
    
    plt.axis('equal')
    plt.title('Probabilistic Roadmap (PRM)')
    plt.show()

# Example usage
num_points = 50
x_range = (-10, 10)
y_range = (-10, 10)
radius = 2.0
start_idx = 0
goal_idx = 49

points = generate_random_points(num_points, x_range, y_range)
graph = build_graph(points, radius)
visualize_prm(points, graph, start_idx, goal_idx)
