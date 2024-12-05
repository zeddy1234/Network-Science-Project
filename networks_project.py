import osmnx as ox
import networkx as nx
import folium
from folium import plugins
import random
import pandas as pd
import numpy as np
from collections import defaultdict
import time
from datetime import datetime
import branca.colormap as cm


class Vehicle:
    def __init__(self, id, start, end, curiosity, has_navigation=False):
        self.id = id
        self.start = start
        self.end = end
        self.curiosity = curiosity
        self.has_navigation = has_navigation
        self.current_position = start
        self.path = []  # Initialize as empty list instead of None
        self.travel_time = 0
        self.distance_traveled = 0
        self.reroutes = 0
        self.stuck = False  # New flag to track if vehicle is stuck

class TrafficSimulation:
    def __init__(self, center_lat=40.4474, center_lon=-79.9920, radius=800):
        self.center = (center_lat, center_lon)
        self.radius = radius
        self.vehicles = {}
        self.edge_flows = defaultdict(int)
        self.congestion_threshold = 0.7
        self.setup_network()

    def setup_network(self):
        print("Initializing network...")
        self.G = ox.graph_from_point(
            self.center,
            dist=self.radius,
            network_type='drive',
            simplify=True
        )
        
        # Add edge attributes
        for u, v, k, data in self.G.edges(data=True, keys=True):
            self.G[u][v][k]['capacity'] = random.randint(50, 200)
            self.G[u][v][k]['base_speed'] = random.uniform(30, 50)
            self.G[u][v][k]['current_flow'] = 0
            self.G[u][v][k]['congestion_factor'] = 1.0

    def get_edge_weight(self, u, v, edge_data):
        """Calculate edge weight based on congestion"""
        try:
            keys = self.G[u][v].keys()
            if not keys:
                return float('inf')
            k = list(keys)[0]
            
            current_flow = self.edge_flows.get((u, v, k), 0)
            capacity = self.G[u][v][k]['capacity']
            length = self.G[u][v][k].get('length', 1.0)
            
            congestion_factor = 1 + (current_flow / capacity) ** 2 if capacity > 0 else 1
            return length * congestion_factor
        except Exception as e:
            print(f"Error calculating edge weight: {e}")
            return float('inf')

    def is_valid_path(self, path):
        """Check if a path is valid"""
        if not path or not isinstance(path, list):
            return False
        
        # Check if all nodes in path exist in graph
        return all(node in self.G.nodes for node in path)

    # def find_path(self, vehicle):
    #     """Find path using Dijkstra's algorithm with validation"""
    #     if vehicle.stuck:
    #         return []
            
    #     try:
    #         path = nx.shortest_path(
    #             self.G, 
    #             vehicle.current_position,
    #             vehicle.end,
    #             weight=self.get_edge_weight
    #         )
            
    #         if self.is_valid_path(path):
    #             return path
    #         else:
    #             print(f"Invalid path found for vehicle {vehicle.id}")
    #             vehicle.stuck = True
    #             return []
                
    #     except nx.NetworkXNoPath:
    #         print(f"No path found for vehicle {vehicle.id}")
    #         vehicle.stuck = True
    #         return []
    #     except Exception as e:
    #         print(f"Error finding path for vehicle {vehicle.id}: {e}")
    #         vehicle.stuck = True
    #         return []

    def find_path(self, vehicle):
        """Find path using A* algorithm with validation"""
        if vehicle.stuck:
            return []
            
        try:
            # Define a heuristic function for A* (e.g., Euclidean distance)
            def heuristic(node, target):
                x1, y1 = self.G.nodes[node]['x'], self.G.nodes[node]['y']
                x2, y2 = self.G.nodes[target]['x'], self.G.nodes[target]['y']
                return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

            # Use A* algorithm to find the path
            path = nx.astar_path(
                self.G,
                vehicle.current_position,
                vehicle.end,
                heuristic=heuristic,
                weight=self.get_edge_weight
            )

            # Validate the path
            if self.is_valid_path(path):
                return path
            else:
                print(f"Invalid path found for vehicle {vehicle.id}")
                vehicle.stuck = True
                return []
                
        except nx.NetworkXNoPath:
            print(f"No path found for vehicle {vehicle.id}")
            vehicle.stuck = True
            return []
        except Exception as e:
            print(f"Error finding path for vehicle {vehicle.id}: {e}")
            vehicle.stuck = True
            return []


    def initialize_vehicles(self, num_vehicles, nav_percentage):
        print(f"Initializing {num_vehicles} vehicles ({nav_percentage}% with navigation)...")
        nodes = list(self.G.nodes())
        
        for i in range(num_vehicles):
            start = random.choice(nodes)
            end = random.choice(nodes)
            while end == start:
                end = random.choice(nodes)
            
            has_nav = random.random() < (nav_percentage / 100)
            curiosity = random.random() if not has_nav else 0.8
            
            vehicle = Vehicle(
                id=i,
                start=start,
                end=end,
                curiosity=curiosity,
                has_navigation=has_nav
            )
            self.vehicles[i] = vehicle
            
            # Initialize path for vehicle
            initial_path = self.find_path(vehicle)
            vehicle.path = initial_path if initial_path else []

    def update_edge_flows(self):
        """Update traffic flow on all edges with path validation"""
        self.edge_flows.clear()
        for vehicle in self.vehicles.values():
            if len(vehicle.path) > 1:  # Safe now since path is always a list
                u, v = vehicle.path[0], vehicle.path[1]
                try:
                    k = list(self.G[u][v].keys())[0]
                    self.edge_flows[(u, v, k)] += 1
                except (KeyError, IndexError) as e:
                    print(f"Error updating edge flow for vehicle {vehicle.id}: {e}")
                    vehicle.stuck = True
                    vehicle.path = []

    def should_reroute(self, vehicle):
        """Determine if vehicle should reroute based on congestion"""
        if vehicle.stuck or not vehicle.path or len(vehicle.path) < 2:
            return False
            
        try:
            u, v = vehicle.path[0], vehicle.path[1]
            k = list(self.G[u][v].keys())[0]
            
            current_flow = self.edge_flows.get((u, v, k), 0)
            capacity = self.G[u][v][k]['capacity']
            
            congestion = current_flow / capacity if capacity > 0 else 0
            reroute_threshold = 0.3 if vehicle.has_navigation else 0.7
            
            return congestion > self.congestion_threshold and random.random() < (vehicle.curiosity * reroute_threshold)
        except Exception as e:
            print(f"Error checking reroute for vehicle {vehicle.id}: {e}")
            return False

    def simulate_step(self):
        """Simulate one time step with improved error handling"""
        self.update_edge_flows()
        
        active_vehicles = 0
        stuck_vehicles = 0
        
        for vehicle in self.vehicles.values():
            if vehicle.stuck:
                stuck_vehicles += 1
                continue
                
            if vehicle.current_position == vehicle.end:
                continue
                
            active_vehicles += 1
            
            if self.should_reroute(vehicle):
                new_path = self.find_path(vehicle)
                if new_path:
                    vehicle.path = new_path
                    vehicle.reroutes += 1
            
            if not vehicle.path:
                vehicle.path = self.find_path(vehicle)
                if not vehicle.path:
                    continue
            
            if len(vehicle.path) > 1:
                try:
                    u, v = vehicle.path[0], vehicle.path[1]
                    k = list(self.G[u][v].keys())[0]
                    vehicle.current_position = vehicle.path[1]
                    vehicle.distance_traveled += self.G[u][v][k]['length']
                    vehicle.path = vehicle.path[1:]
                except Exception as e:
                    print(f"Error moving vehicle {vehicle.id}: {e}")
                    vehicle.stuck = True
                    vehicle.path = []
        
        print(f"Active vehicles: {active_vehicles}, Stuck vehicles: {stuck_vehicles}")

    def create_visualization(self):
        m = folium.Map(location=self.center, zoom_start=15)
        
        colormap = cm.LinearColormap(
            colors=['green', 'yellow', 'red'],
            vmin=0,
            vmax=1,
            caption='Congestion Level'
        )
        m.add_child(colormap)
        
        edges = ox.graph_to_gdfs(self.G, nodes=False)
        for (u, v, k), row in edges.iterrows():
            try:
                congestion = min(1.0, self.edge_flows.get((u, v, k), 0) / self.G[u][v][k]['capacity'])
                color = colormap(congestion)
                
                if row.geometry:
                    points = [(lat, lon) for lon, lat in row.geometry.coords]
                    folium.PolyLine(
                        points,
                        color=color,
                        weight=2 + (congestion * 4),
                        opacity=0.8,
                        popup=f"Flow: {self.edge_flows.get((u, v, k), 0)}<br>Capacity: {self.G[u][v][k]['capacity']}"
                    ).add_to(m)
            except Exception as e:
                print(f"Error visualizing edge ({u}, {v}, {k}): {e}")
        
        for vehicle in self.vehicles.values():
            try:
                node = self.G.nodes[vehicle.current_position]
                color = 'red' if vehicle.stuck else ('blue' if vehicle.has_navigation else 'gray')
                
                folium.CircleMarker(
                    location=(node['y'], node['x']),
                    radius=3,
                    color=color,
                    fill=True,
                    popup=f"Vehicle {vehicle.id}<br>Nav: {vehicle.has_navigation}<br>Reroutes: {vehicle.reroutes}<br>Stuck: {vehicle.stuck}"
                ).add_to(m)
            except Exception as e:
                print(f"Error visualizing vehicle {vehicle.id}: {e}")
        
        return m

def main():
    sim = TrafficSimulation()
    
    num_vehicles = 50
    nav_percentage = 50
    num_steps = 20
    
    sim.initialize_vehicles(num_vehicles, nav_percentage)
    
    print(f"Running simulation for {num_steps} steps...")
    for step in range(num_steps):
        print(f"\nStep {step + 1}/{num_steps}")
        sim.simulate_step()
        
        if (step + 1) % 5 == 0 or step == 0:
            m = sim.create_visualization()
            m.save(f'traffic_step_{step + 1}.html')
    
    # Calculate final statistics
    total_reroutes = sum(v.reroutes for v in sim.vehicles.values())
    active_vehicles = sum(1 for v in sim.vehicles.values() if not v.stuck)
    stuck_vehicles = sum(1 for v in sim.vehicles.values() if v.stuck)
    avg_distance = sum(v.distance_traveled for v in sim.vehicles.values()) / num_vehicles
    
    print("\nSimulation Results:")
    print(f"Total reroutes: {total_reroutes}")
    print(f"Active vehicles: {active_vehicles}")
    print(f"Stuck vehicles: {stuck_vehicles}")
    print(f"Average distance traveled: {avg_distance:.2f} meters")
    print(f"Maps generated: traffic_step_X.html")

if __name__ == '__main__':
    main()