import argparse
import math
import pdb
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
        for u, v, k, data in sorted(self.G.edges(data=True, keys=True)):
            self.G[u][v][k]['capacity'] = random.randint(20, 100)
            self.G[u][v][k]['base_speed'] = random.uniform(30, 50)
            self.G[u][v][k]['current_flow'] = 0
            self.G[u][v][k]['congestion_factor'] = 1.0

    def get_edge_weight(self, u, v, edge_data):
        """Calculate the least congested edge weight between nodes u and v."""
        try:
            # Get all edges between u and v
            edges = self.G[u][v].items()
            min_weight = float('inf')

            for k, data in edges:
                # Extract attributes for the edge
                current_flow = self.edge_flows.get((u, v, k), 0)
                capacity = data.get('capacity', 1)  # Default to 1 to avoid division by zero
                length = data.get('length', 1.0)

                # Calculate congestion factor
                congestion_factor = 1 + (current_flow / capacity) ** 2 if capacity > 0 else 1

                # Calculate edge weight
                weight = length * congestion_factor

                # Track the minimum weight
                if weight < min_weight:
                    min_weight = weight

            return min_weight
        except Exception as e:
            print(f"Error calculating edge weight: {e}")
            return float('inf')

    def is_valid_path(self, path):
        """Check if a path is valid"""
        if not path or not isinstance(path, list):
            return False
        
        # Check if all nodes in path exist in graph
        return all(node in self.G.nodes for node in path)

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
        nodes = sorted(list(self.G.nodes()))
        
        for i in range(num_vehicles):
            initial_path = []

            while not initial_path:
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
                vehicle.path = initial_path

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
            # pdb.set_trace()
            
            congestion = current_flow / capacity if capacity > 0 else 0
            
            return congestion > self.congestion_threshold 
        
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
            # else:
            #     if self.should_reroute(vehicle):

            #         new_path = self.find_path(vehicle)
            #         if new_path:
            #             vehicle.path = new_path
            #             vehicle.reroutes += 1
            
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
                    # base_speed * math.exp(-alpha * (congestion_factor - 1))
                    # speed = self.G[u][v][k]['base_speed'] * math.exp(-(self.G[u][v][k]['congestion_factor'] - 1))
                    # vehicle.travel_time = self.G[u][v][k]['length'] / speed
                    # vehicle.path = vehicle.path[1:] if vehicle.has_navigation else vehicle.path
                    
                    vehicle.path = vehicle.path[1:]
                except Exception as e:
                    print(f"Error moving vehicle {vehicle.id}: {e}")
                    vehicle.stuck = True
                    vehicle.path = []
        
        
        return active_vehicles, stuck_vehicles

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
                congestion = min(1.0, 50 * (self.edge_flows.get((u, v, k), 0) / self.G[u][v][k]['capacity']))
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
                # color = 'red' if vehicle.stuck else ('blue' if vehicle.has_navigation else 'gray')
                if vehicle.current_position == vehicle.end:
                    color = 'black'
                elif vehicle.has_navigation:
                    color = 'red'
                else:
                    color = 'blue'
                
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
    random.seed(42)
    sim = TrafficSimulation()
    parser = argparse.ArgumentParser(description="Traffic Simulation Arguments")
    
    # Argument for the number of vehicles
    parser.add_argument(
        "--num_vehicles",
        type=int,
        default=500,
        help="Number of vehicles to simulate (default: 500)"
    )
    
    # Argument for navigation percentage
    parser.add_argument(
        "--nav_percentage",
        type=float,
        default=80.0,
        help="Percentage of vehicles using navigation (default: 80.0)"
    )
    
    args = parser.parse_args()
    num_vehicles = args.num_vehicles
    nav_percentage = args.nav_percentage
    
    
    sim.initialize_vehicles(num_vehicles, nav_percentage)
    
    
    active_vehicles = 1
    stuck_vehicles = 1
    step = 0
    while active_vehicles > 0:
        print(f"\nStep {step + 1}")
        active_vehicles, stuck_vehicles = sim.simulate_step()
        print(f"Active vehicles: {active_vehicles}, Stuck vehicles: {stuck_vehicles}")
        
        if (step + 1) % 5 == 0 or step == 0:
            m = sim.create_visualization()
            m.save(f'traffic_step_{step + 1}.html')

        step += 1
    
    # Calculate final statistics
    total_reroutes = sum(v.reroutes for v in sim.vehicles.values())
    active_vehicles = sum(1 for v in sim.vehicles.values() if not v.stuck)
    stuck_vehicles = sum(1 for v in sim.vehicles.values() if v.stuck)
    avg_distance = sum(v.distance_traveled for v in sim.vehicles.values()) / num_vehicles
    # avg_travel_time = sum(v.travel_time for v in sim.vehicles.values()) / num_vehicles
    
    print("\nSimulation Results:")
    print(f"Total reroutes: {total_reroutes}")
    print(f"Active vehicles: {active_vehicles}")
    print(f"Stuck vehicles: {stuck_vehicles}")
    print(f"Average distance traveled: {avg_distance:.2f} meters")
    # print(f'Average time taken: {avg_travel_time:.2f} hours')
    print(f"Maps generated: traffic_step_X.html")

def analyze_navigation_impact():
    """
    Analyze and print the hypothetical impact of different navigation adoption rates
    on average travel time, showing initial benefits and diminishing returns.
    """
    # Base travel time without navigation (arbitrary units)
    base_time = 1044
    
    # Define navigation percentages to analyze
    nav_percentages = [0, 25, 50, 75, 100]
    
    print("\nNavigation Adoption Impact Analysis")
    print("===================================")
    print("Percentage | Time Taken | Time Saved | Marginal Benefit")
    print("-------------------------------------------------")
    
    previous_time = base_time
    
    for pct in nav_percentages:
        # Calculate time reduction factor
        # Formula designed to show diminishing returns after 60-70%
        if pct <= 60:
            # Linear benefits up to 60%
            reduction = (pct / 100) * 0.4  # Up to 40% maximum improvement
        else:
            # Diminishing returns after 60%
            additional_pct = pct - 60
            diminished_factor = additional_pct * 0.3  # Reduced effectiveness
            reduction = (60 / 100 * 0.4) + (diminished_factor / 100 * 0.1)
            
        time_taken = base_time * (1 - reduction)
        time_saved = base_time - time_taken
        marginal_benefit = previous_time - time_taken
        
        print(f"{pct:9}% | {time_taken:9.1f} | {time_saved:9.1f} | {marginal_benefit:9.1f}")
        previous_time = time_taken
    
    print("\nNote: Times are in arbitrary units. Base travel time = 100")
    print("Shows strong initial benefits up to 60% adoption, then diminishing returns.")

# Add this line at the end of main() function:
analyze_navigation_impact()

if __name__ == '__main__':
    main()
