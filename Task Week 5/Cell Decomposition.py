import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class CellDecomposition:
    def __init__(self, width, height, cell_size):
        """
        Initialize Cell Decomposition.
        
        Parameters:
        width: area width
        height: area height
        cell_size: size of each cell
        """
        self.width = width
        self.height = height
        self.cell_size = cell_size
        
        # Calculate the number of cells in each dimension
        self.cols = width // cell_size
        self.rows = height // cell_size
        
        # Create a grid to represent the area
        self.grid = np.zeros((self.rows, self.cols))
        
        # Dictionary to store the adjacency list
        self.graph = {}
        
    def add_obstacle(self, x, y, width, height):
        """Add a rectangular obstacle"""
        # Convert coordinates to cell indices
        start_col = max(0, x // self.cell_size)
        start_row = max(0, y // self.cell_size)
        end_col = min(self.cols, (x + width) // self.cell_size + 1)
        end_row = min(self.rows, (y + height) // self.cell_size + 1)
        
        # Mark obstructed cells
        for row in range(start_row, end_row):
            for col in range(start_col, end_col):
                self.grid[row, col] = 1
                
    def build_graph(self):
        """Build a graph from unobstructed cells"""
        # Reset the graph
        self.graph = {}
        
        # Iterate over each cell
        for row in range(self.rows):
            for col in range(self.cols):
                # Skip if cell is obstructed
                if self.grid[row, col] == 1:
                    continue
                    
                cell = (row, col)
                neighbors = []
                
                # Check 8 neighboring directions
                directions = [(-1,-1), (-1,0), (-1,1),
                              (0,-1),         (0,1),
                              (1,-1), (1,0),  (1,1)]
                
                for dr, dc in directions:
                    new_row, new_col = row + dr, col + dc
                    
                    # Check if neighbor is valid and unobstructed
                    if (0 <= new_row < self.rows and 
                        0 <= new_col < self.cols and 
                        self.grid[new_row, new_col] == 0):
                        neighbors.append((new_row, new_col))
                
                self.graph[cell] = neighbors
    
    def find_path(self, start, goal):
        """Find a path from start to goal using BFS"""
        # Convert coordinates to cell indices
        start_cell = (start[1] // self.cell_size, start[0] // self.cell_size)
        goal_cell = (goal[1] // self.cell_size, goal[0] // self.cell_size)
        
        # Check if start or goal is obstructed
        if (self.grid[start_cell] == 1 or 
            self.grid[goal_cell] == 1):
            return None
        
        # Initialize BFS
        queue = deque([[start_cell]])
        visited = {start_cell}
        
        while queue:
            path = queue.popleft()
            current = path[-1]
            
            if current == goal_cell:
                return path
            
            # Check all neighbors
            for neighbor in self.graph.get(current, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append(new_path)
        
        return None
    
    def visualize(self, path=None):
        """Visualize the area, obstacles, and path"""
        plt.figure(figsize=(10, 10))
        
        # Draw the grid
        plt.imshow(self.grid, cmap='binary')
        
        # Draw grid lines
        for x in range(self.cols + 1):
            plt.axvline(x - 0.5, color='gray', linewidth=0.5)
        for y in range(self.rows + 1):
            plt.axhline(y - 0.5, color='gray', linewidth=0.5)
        
        # Draw the path if it exists
        if path:
            path_x = [p[1] for p in path]
            path_y = [p[0] for p in path]
            plt.plot(path_x, path_y, 'r-', linewidth=2, label='Path')
            
            # Mark start and goal
            plt.plot(path_x[0], path_y[0], 'go', label='Start')
            plt.plot(path_x[-1], path_y[-1], 'ro', label='Goal')
        
        plt.grid(True)
        plt.legend()
        plt.title('Cell Decomposition Path Planning')
        plt.show()

# Example usage
if __name__ == "__main__":
    # Create a cell decomposition instance
    cd = CellDecomposition(width=200, height=200, cell_size=20)
    
    # Add obstacles
    obstacles = [
        (40, 40, 40, 40),    # (x, y, width, height)
        (120, 80, 40, 80),
        (40, 120, 80, 40)
    ]
    
    for obs in obstacles:
        cd.add_obstacle(*obs)
    
    # Build the graph
    cd.build_graph()
    
    # Define start and goal points
    start = (20, 20)
    goal = (160, 160)
    
    # Find a path
    path = cd.find_path(start, goal)
    
    if path:
        print("Path found!")
        # Convert path to cell center coordinates
        path_coords = [(p[0] * cd.cell_size + cd.cell_size // 2,
                        p[1] * cd.cell_size + cd.cell_size // 2) for p in path]
        print("Path coordinates:", path_coords)
    else:
        print("No path found")
    
    # Visualize the result
    cd.visualize(path)
