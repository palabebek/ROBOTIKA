# Implementasi Algoritma Dijkstra
import heapq
from collections import defaultdict

class DijkstraAlgorithm:
    def __init__(self):
        self.graph = defaultdict(list)
    
    def add_edge(self, u, v, weight):
        """Menambahkan edge ke graph"""
        self.graph[u].append((v, weight))
        self.graph[v].append((u, weight))  # Untuk graph tidak terarah
    
    def find_shortest_path(self, start, end):
        """Mencari jalur terpendek menggunakan algoritma Dijkstra"""
        # Inisialisasi
        distances = {node: float('infinity') for node in self.graph}
        distances[start] = 0
        pq = [(0, start)]  # Priority queue (jarak, node)
        previous = {node: None for node in self.graph}
        
        while pq:
            current_distance, current_node = heapq.heappop(pq)
            
            # Jika sudah mencapai node tujuan
            if current_node == end:
                break
                
            # Jika menemukan jarak yang lebih besar
            if current_distance > distances[current_node]:
                continue
                
            # Cek semua tetangga
            for neighbor, weight in self.graph[current_node]:
                distance = current_distance + weight
                
                # Jika menemukan jalur yang lebih pendek
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))
        
        # Rekonstruksi jalur
        path = []
        current_node = end
        while current_node is not None:
            path.append(current_node)
            current_node = previous[current_node]
        path.reverse()
        
        return path, distances[end]

# Membuat instance dari algoritma Dijkstra
dijkstra = DijkstraAlgorithm()

# Menambahkan edges ke graph
dijkstra.add_edge('A', 'B', 4)
dijkstra.add_edge('A', 'C', 2)
dijkstra.add_edge('B', 'C', 5)
dijkstra.add_edge('B', 'D', 10)
dijkstra.add_edge('C', 'D', 3)
dijkstra.add_edge('D', 'E', 4)
dijkstra.add_edge('C', 'E', 8)

# Mencari jalur terpendek dari 'A' ke 'E'
path, cost = dijkstra.find_shortest_path('A', 'E')
print(f"Jalur terpendek: {path} dengan biaya: {cost}")
