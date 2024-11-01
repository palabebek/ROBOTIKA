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

# Contoh penggunaan Dijkstra
def test_dijkstra():
    dijkstra = DijkstraAlgorithm()
    # Menambahkan edge ke graph
    dijkstra.add_edge('A', 'B', 4)
    dijkstra.add_edge('A', 'C', 2)
    dijkstra.add_edge('B', 'C', 1)
    dijkstra.add_edge('B', 'D', 5)
    dijkstra.add_edge('C', 'D', 8)
    dijkstra.add_edge('C', 'E', 10)
    dijkstra.add_edge('D', 'E', 2)
    
    # Mencari jalur terpendek
    start = 'A'
    end = 'E'
    path, cost = dijkstra.find_shortest_path(start, end)
    print(f"\nDijkstra - Jalur terpendek dari {start} ke {end}:")
    print(f"Jalur: {' -> '.join(path)}")
    print(f"Total cost: {cost}")

# Menggunakan Dijkstra untuk mencari jalur terpendek dari A ke E
dijkstra = DijkstraAlgorithm()
dijkstra.add_edge('A', 'B', 4)
dijkstra.add_edge('A', 'C', 2)
dijkstra.add_edge('B', 'C', 1)
dijkstra.add_edge('B', 'D', 5)
dijkstra.add_edge('C', 'D', 8)
dijkstra.add_edge('C', 'E', 10)
dijkstra.add_edge('D', 'E', 2)

path, cost = dijkstra.find_shortest_path('A', 'E')
print(f"\nHasil pencarian jalur terpendek dari A ke E:")
print(f"Jalur: {' -> '.join(path)}")
print(f"Total cost: {cost}")
