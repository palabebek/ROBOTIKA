import heapq
from collections import defaultdict

# Implementasi Algoritma A*
class AStarAlgorithm:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
        
    def set_obstacle(self, row, col):
        """Menambahkan obstacle pada grid"""
        self.grid[row][col] = 1
        
    def is_valid(self, row, col):
        """Mengecek apakah posisi valid"""
        return (0 <= row < self.grid_size and 
                0 <= col < self.grid_size and 
                self.grid[row][col] != 1)
    
    def get_neighbors(self, pos):
        """Mendapatkan tetangga yang valid"""
        row, col = pos
        directions = [(-1,0), (1,0), (0,-1), (0,1)]  # Atas, bawah, kiri, kanan
        return [(row+dr, col+dc) for dr, dc in directions 
                if self.is_valid(row+dr, col+dc)]
    
    def heuristic(self, a, b):
        """Menghitung jarak Manhattan sebagai heuristic"""
        return abs(b[0] - a[0]) + abs(b[1] - a[1])
    
    def find_path(self, start, end):
        """Mencari jalur menggunakan algoritma A*"""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = defaultdict(lambda: float('infinity'))
        g_score[start] = 0
        f_score = defaultdict(lambda: float('infinity'))
        f_score[start] = self.heuristic(start, end)
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path, g_score[end]
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, end)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None, None  # Tidak ada jalur yang ditemukan

# Contoh penggunaan A*
def test_astar():
    size = 5
    astar = AStarAlgorithm(size)
    
    # Menambahkan obstacle
    astar.set_obstacle(1, 1)
    astar.set_obstacle(1, 2)
    astar.set_obstacle(2, 2)
    
    # Mencari jalur
    start = (0, 0)
    end = (4, 4)
    path, cost = astar.find_path(start, end)
    
    if path:
        print("\nA* - Jalur yang ditemukan:")
        # Visualisasi grid dan jalur
        grid_visual = [['.' for _ in range(size)] for _ in range(size)]
        for r in range(size):
            for c in range(size):
                if (r, c) in path:
                    grid_visual[r][c] = '*'
                elif astar.grid[r][c] == 1:
                    grid_visual[r][c] = '#'
        
        for row in grid_visual:
            print(' '.join(row))
        print(f"Total cost: {cost}")
    else:
        print("Tidak ada jalur yang ditemukan")

if __name__ == "__main__":
    test_astar()
