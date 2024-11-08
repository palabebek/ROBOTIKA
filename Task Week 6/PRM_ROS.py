#!/usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray 
from geometry_msgs.msg import Point
import heapq

class PRMPlanner:
   def __init__(self):
       rospy.init_node('prm_planner')
       
       # Parameters
       self.num_points = 100
       self.connection_radius = 2.0
       self.x_range = (-10, 10) 
       self.y_range = (-10, 10)
       
       # Publishers
       self.marker_pub = rospy.Publisher('/prm_markers', MarkerArray, queue_size=1)
       self.path_pub = rospy.Publisher('/path_markers', Marker, queue_size=1)
       
       # Initialize PRM
       self.points = self.generate_random_points()
       self.graph = self.build_graph()
       
       # Visualization
       self.visualize_prm()
       
   def generate_random_points(self):
       """Generate random points for PRM"""
       x = np.random.uniform(self.x_range[0], self.x_range[1], self.num_points)
       y = np.random.uniform(self.y_range[0], self.y_range[1], self.num_points)
       return np.column_stack((x, y))
       
   def build_graph(self):
       """Build graph connections"""
       graph = {i: [] for i in range(len(self.points))}
       
       for i in range(len(self.points)):
           for j in range(i + 1, len(self.points)):
               dist = np.linalg.norm(self.points[i] - self.points[j])
               if dist <= self.connection_radius:
                   graph[i].append(j)
                   graph[j].append(i)
       return graph
       
   def dijkstra_shortest_path(self, start, goal):
       """Find shortest path using Dijkstra's algorithm"""
       distances = {node: float('inf') for node in self.graph}
       distances[start] = 0
       previous = {node: None for node in self.graph}
       pq = [(0, start)]
       
       while pq:
           current_distance, current_node = heapq.heappop(pq)
           
           if current_node == goal:
               path = []
               while current_node is not None:
                   path.append(current_node)
                   current_node = previous[current_node]
               return path[::-1]
           
           if current_distance > distances[current_node]:
               continue
               
           for neighbor in self.graph[current_node]:
               distance = current_distance + np.linalg.norm(
                   self.points[current_node] - self.points[neighbor])
               
               if distance < distances[neighbor]:
                   distances[neighbor] = distance
                   previous[neighbor] = current_node
                   heapq.heappush(pq, (distance, neighbor))
       
       return None
       
   def create_point_marker(self, points, color=(1.0, 1.0, 1.0)):
       """Create marker for points"""
       marker = Marker()
       marker.header.frame_id = "map"
       marker.type = Marker.POINTS
       marker.action = Marker.ADD
       marker.scale.x = 0.2
       marker.scale.y = 0.2
       marker.color.a = 1.0
       marker.color.r = color[0]
       marker.color.g = color[1]
       marker.color.b = color[2]
       
       for point in points:
           p = Point()
           p.x = point[0]
           p.y = point[1]
           p.z = 0.0
           marker.points.append(p)
           
       return marker
       
   def create_line_marker(self, points, connections, id=0, color=(0.5, 0.5, 0.5)):
       """Create marker for connections"""
       marker = Marker()
       marker.header.frame_id = "map"
       marker.id = id
       marker.type = Marker.LINE_LIST
       marker.action = Marker.ADD
       marker.scale.x = 0.05
       marker.color.a = 0.5
       marker.color.r = color[0]
       marker.color.g = color[1]
       marker.color.b = color[2]
       
       for i, neighbors in connections.items():
           for j in neighbors:
               p1 = Point()
               p1.x = points[i][0]
               p1.y = points[i][1]
               p1.z = 0.0
               
               p2 = Point()
               p2.x = points[j][0]
               p2.y = points[j][1]
               p2.z = 0.0
               
               marker.points.append(p1)
               marker.points.append(p2)
               
       return marker
       
   def visualize_prm(self):
       """Visualize PRM in RViz"""
       # Create markers for visualization
       marker_array = MarkerArray()
       
       # Add points marker
       points_marker = self.create_point_marker(self.points)
       marker_array.markers.append(points_marker)
       
       # Add connections marker
       connections_marker = self.create_line_marker(self.points, self.graph)
       marker_array.markers.append(connections_marker)
       
       # Find and visualize path
       start_idx = 0
       goal_idx = self.num_points - 1
       path = self.dijkstra_shortest_path(start_idx, goal_idx)
       
       if path:
           path_points = self.points[path]
           path_marker = Marker()
           path_marker.header.frame_id = "map"
           path_marker.type = Marker.LINE_STRIP
           path_marker.action = Marker.ADD
           path_marker.scale.x = 0.1
           path_marker.color.a = 1.0
           path_marker.color.r = 0.0
           path_marker.color.g = 1.0
           path_marker.color.b = 0.0
           
           for point in path_points:
               p = Point()
               p.x = point[0]
               p.y = point[1]
               p.z = 0.0
               path_marker.points.append(p)
               
           self.path_pub.publish(path_marker)
           
       # Publish markers
       self.marker_pub.publish(marker_array)
       
   def run(self):
       """Main run loop"""
       rate = rospy.Rate(10)
       while not rospy.is_shutdown():
           self.visualize_prm()
           rate.sleep()

if __name__ == '__main__':
   try:
       planner = PRMPlanner()
       planner.run()
   except rospy.ROSInterruptException:
       pass

<launch>
    <!-- Start RViz -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find prm_planner)/rviz/prm_config.rviz"/>
    
    <!-- Start PRM Planner -->
    <node name="prm_planner" pkg="prm_planner" type="prm_node.py" output="screen"/>
</launch>

