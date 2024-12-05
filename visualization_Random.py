import osmnx as ox
import folium
import networkx as nx
import numpy as np
import random
from collections import defaultdict

def get_pittsburgh_area():
   center = (40.4433, -79.9436)
   G = ox.graph_from_point(center, dist=800, network_type='drive')
   edge_congestion = defaultdict(float)
   for u, v in G.edges():
       if random.random() < 0.4:
           edge_congestion[(u,v)] = random.uniform(1.2, 4.0)
       else:
           edge_congestion[(u,v)] = 1.0
   return G, edge_congestion

def get_routes_with_navigation(G, start, end, nav_percentage, edge_congestion, prev_routes=None):
   G_route = G.copy()
   num_vehicles = 100
   nav_vehicles = int(num_vehicles * nav_percentage)
   normal_vehicles = num_vehicles - nav_vehicles
   
   base_route = nx.shortest_path(G, start, end, weight='length')
   
   for i in range(len(base_route)-1):
       u, v = base_route[i], base_route[i+1]
       edge_congestion[(u,v)] = edge_congestion.get((u,v), 1.0) * (1 + (normal_vehicles/50))
   
   if nav_vehicles > 0:
       for (u, v) in G_route.edges():
           base_weight = G_route[u][v][0]['length']
           congestion = edge_congestion.get((u,v), 1.0)
           G_route[u][v][0]['weight'] = base_weight * congestion
       
       nav_route = nx.shortest_path(G_route, start, end, weight='weight')
       
       for i in range(len(nav_route)-1):
           u, v = nav_route[i], nav_route[i+1]
           edge_congestion[(u,v)] = edge_congestion.get((u,v), 1.0) * (1 + (nav_vehicles/50))
       
       return nav_route, edge_congestion
   
   return base_route, edge_congestion

def create_route_map(G, route, nav_percentage, edge_congestion, initial_congestion):
   m = folium.Map(location=[40.4433, -79.9436], zoom_start=15)
   
   for (u, v), congestion in edge_congestion.items():
       initial_cong = initial_congestion.get((u,v), 1.0)
       if congestion > initial_cong:
           coords = [(G.nodes[u]['y'], G.nodes[u]['x']), 
                    (G.nodes[v]['y'], G.nodes[v]['x'])]
           folium.PolyLine(coords, weight=4, color='red', opacity=0.8).add_to(m)
   
   route_coords = []
   for node in route:
       coords = (G.nodes[node]['y'], G.nodes[node]['x'])
       route_coords.append(coords)
   
   route_length = sum(G[route[i]][route[i+1]][0]['length'] for i in range(len(route)-1))
   route_congestion = sum(edge_congestion.get((route[i], route[i+1]), 1.0) 
                         for i in range(len(route)-1)) / len(route)
   
   popup_text = f"""
   Navigation Rate: {nav_percentage*100}%
   Route Length: {route_length:.1f}m
   Avg Congestion: {route_congestion:.2f}x
   """
   
   folium.PolyLine(
       route_coords,
       weight=4,
       color='blue',
       opacity=0.8,
       popup=popup_text
   ).add_to(m)
   
   legend_html = '''
   <div style="position: fixed; bottom: 50px; left: 50px; 
        background-color:white; padding: 10px; border:2px solid grey;">
       <p><span style="color:red;">―</span> New Congestion</p>
       <p><span style="color:blue;">―</span> Vehicle Route</p>
   </div>
   '''
   m.get_root().html.add_child(folium.Element(legend_html))
   
   save_path = f"C:/Users/bhush/Downloads/route_{int(nav_percentage*100)}pct.html"
   m.save(save_path)
   print(f"Saved map for {nav_percentage*100}% navigation")
   print(f"Route length: {route_length:.1f}m")
   print(f"Average congestion: {route_congestion:.2f}x")
   print("-" * 40)

def simulate_routes():
   G, initial_congestion = get_pittsburgh_area()
   nodes = list(G.nodes())
   
   while True:
       start = random.choice(nodes)
       end = random.choice(nodes)
       try:
           path = nx.shortest_path(G, start, end, weight='length')
           if len(path) >= 8:
               break
       except:
           continue
   
   cumulative_congestion = initial_congestion.copy()
   
   for nav_rate in [0.0, 0.25, 0.50, 0.75, 1.0]:
       route, cumulative_congestion = get_routes_with_navigation(
           G, start, end, nav_rate, cumulative_congestion.copy())
       create_route_map(G, route, nav_rate, cumulative_congestion, initial_congestion)

if __name__ == "__main__":
   simulate_routes()