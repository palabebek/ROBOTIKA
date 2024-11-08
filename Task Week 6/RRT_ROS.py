#!/usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import random

class RRTPlanner:
    def __init__(self):
        rospy.init_node('rrt_planner')
        
        # RRT Parameters
        self.start = np.array([-8.0, -8.0])
        self.goal = np.array([8.0, 8.0])
        self.step_size = 0.5
        self.max_iterations = 5000
        self.goal_sample_rate = 0.1
        self.min_distance_to_goal = 0.5
        
        # Workspace bounds
        self.x_range = (-10, 10)
        self.y_range = (-10, 10)
        
        # Publishers
        self.tree_pub = rospy.Publisher('/rrt_tree', MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher('/rrt_path', Marker, queue_size=1)
        
        # Tree structure: dictionary with parent indices
        self.vertices = [self.start]
        self.edges = []
        
        # Build and visualize RRT
        self.build_rrt()
        self.visualize_rrt()
        
    def random_point(self):
        """Generate random point in workspace"""
        if random.random() < self.goal_sample_rate:
            return self.goal
            
        return np.array([
            random.uniform(self.x_range[0], self.x_range[1]),
            random.uniform(self.y_range[0], self.y_range[1])
        ])
        
    def nearest_vertex(self, point):
        """Find nearest vertex in tree to given point"""
        distances = [np.linalg.norm(vertex - point) for vertex in self.vertices]
        return np.argmin(distances)
        
    def new_vertex(self, nearest, random_point):
        """Generate new vertex in direction of random point"""
        direction = random_point - self.vertices[nearest]
        norm = np.linalg.norm(direction)
        
        if norm == 0:
            return random_point
            
        direction = direction / norm
        return self.vertices[nearest] + direction * min(self.step_size, norm)
        
    def is_collision_free(self, v1, v2):
        """Check if path between vertices is collision-free"""
        # Add obstacle checking here if needed
        return True
        
    def build_rrt(self):
        """Build RRT from start to goal"""
        for _ in range(self.max_iterations):
            # Generate random point
            random_point = self.random_point()
            
            # Find nearest vertex
            nearest_idx = self.nearest_vertex(random_point)
            
            # Generate new vertex
            new_point = self.new_vertex(nearest_idx, random_point)
            
            # Check if path is collision free
            if self.is_collision_free(self.vertices[nearest_idx], new_point):
                # Add vertex and edge
                self.vertices.append(new_point)
                self.edges.append((nearest_idx, len(self.vertices)-1))
                
                # Check if we reached the goal
                if np.linalg.norm(new_point - self.goal) < self.min_distance_to_goal:
                    self.vertices.append(self.goal)
                    self.edges.append((len(self.vertices)-2, len(self.vertices)-1))
                    rospy.loginfo("Goal reached!")
                    return True
                    
        rospy.logwarn("Max iterations reached without finding path")
        return False
        
    def find_path(self):
        """Find path from start to goal in tree"""
        if len(self.vertices) < 2:
            return None
            
        # Start from goal (last vertex)
        current_idx = len(self.vertices) - 1
        path = [self.vertices[current_idx]]
        
        # Traverse tree backwards to start
        while current_idx != 0:
            # Find parent
            for edge in self.edges:
                if edge[1] == current_idx:
                    current_idx = edge[0]
                    path.append(self.vertices[current_idx])
                    break
                    
        return path[::-1]  # Reverse path to get start->goal
        
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
        
    def create_line_marker(self, edges, id=0, color=(0.5, 0.5, 0.5)):
        """Create marker for edges"""
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
        
        for edge in edges:
            p1 = Point()
            p1.x = self.vertices[edge[0]][0]
            p1.y = self.vertices[edge[0]][1]
            p1.z = 0.0
            
            p2 = Point()
            p2.x = self.vertices[edge[1]][0]
            p2.y = self.vertices[edge[1]][1]
            p2.z = 0.0
            
            marker.points.append(p1)
            marker.points.append(p2)
            
        return marker
        
    def visualize_rrt(self):
        """Visualize RRT in RViz"""
        # Create markers for visualization
        marker_array = MarkerArray()
        
        # Add vertices marker
        vertices_marker = self.create_point_marker(self.vertices)
        marker_array.markers.append(vertices_marker)
        
        # Add edges marker
        edges_marker = self.create_line_marker(self.edges)
        marker_array.markers.append(edges_marker)
        
        # Find and visualize path
        path = self.find_path()
        if path:
            path_marker = Marker()
            path_marker.header.frame_id = "map"
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.1
            path_marker.color.a = 1.0
            path_marker.color.r = 0.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            
            for point in path:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
                path_marker.points.append(p)
                
            self.path_pub.publish(path_marker)
            
        # Publish markers
        self.tree_pub.publish(marker_array)
        
    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.visualize_rrt()
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = RRTPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
