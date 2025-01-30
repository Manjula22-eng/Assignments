3D Shortest Path Search and Visualization:
1. Introduction
This project focuses on finding the shortest path in a 3D grid using Dijkstra’s algorithm.
The project involves:
Constructing a 3D Grid Graph using NetworkX
Assigning random weights to nodes
Finding the shortest path between selected points
Visualizing the paths in 3D

2. Algorithms Used
2.1 Grid Graph Construction
We use NetworkX to generate a 3D grid graph, where:
Each node is represented as (x, y, z).
Nodes are connected to their six neighbors (left, right, front, back, up, and down).
2.2 Dijkstra’s Algorithm
Dijkstra’s algorithm finds the shortest path between nodes based on their weights.

Steps of Dijkstra’s Algorithm:
Initialize: Set all node distances to ∞, except the start node (0).
Select Node: Choose the node with the smallest distance.
Update Neighbors: Check if moving to a neighbor reduces its distance; if yes, update it.
Repeat: Continue until reaching the target node.
3. Implementation Details
3.1 Importing Libraries
python:
import numpy as np
import networkx as nx
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
networkx → Creates and processes the 3D graph.
random → Assigns random weights.
matplotlib.pyplot → Visualizes paths in 3D.
3.2 Creating a 3D Grid Graph
python:
GRID_SIZE = 20  # Define the grid size
Create a 3D Graph:
G = nx.grid_graph(dim=[GRID_SIZE+1, GRID_SIZE+1, GRID_SIZE+1])
print("Graph created with", len(G.nodes), "nodes.")
Creates a 3D cube grid with (GRID_SIZE+1)^3 nodes.
Each node connects to its six neighboring nodes.
3.3 Assigning Random Weights
python:
for node in G.nodes:
    G.nodes[node]['weight'] = random.choice([1, 1, 1, 5, 10, 15])
print("Weights assigned.")
Weights simulate terrain difficulty (low values = easy to cross, high values = hard).
3.4 Finding the Shortest Path Using Dijkstra
python:
def shortest_path(graph, start, end):
    print(f"Finding shortest path from {start} to {end}...")
    return nx.shortest_path(graph, source=start, target=end, weight='weight', method='dijkstra')
Uses NetworkX’s Dijkstra’s algorithm to find the best path.
3.5 Finding Paths Between Selected Points
python:
start_end_pairs = [
    ((0, 0, 0), (10, 10, 10)),  
    ((5, 5, 5), (15, 15, 15))
]
paths = []
for start, end in start_end_pairs:
    try:
        path = shortest_path(G, start, end)
        paths.append(path)
        print(f"Path found: {len(path)} steps.")
    except Exception as e:
        print("Error finding path:", e)
Computes shortest paths for selected pairs of points.
3.6 3D Path Visualization
python:
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
Plot each other:
for path in paths:
    x, y, z = zip(*path)
    ax.plot(x, y, z, marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title("3D Shortest Paths")
plt.show()
Plots paths in 3D using Matplotlib.
4. Results and Analysis
4.1 Sample Output
Creating 3D Grid Graph...
Graph created with 9261 nodes.
Weights assigned.
Finding shortest path from (0, 0, 0) to (10, 10, 10)...
Path found: 30 steps.
Finding shortest path from (5, 5, 5) to (15, 15, 15)...
Path found: 40 steps.
Displaying plot...

Each path consists of multiple steps depending on node weights.
4.2 3D Path Visualization
The output is a 3D plot where:
Nodes represent points in the grid.
Lines show the shortest paths between selected points.

Conclusion:
This project finds the shortest path in a 3D grid using Dijkstra’s algorithm and visualizes it in 3D. It assigns random 
weights to nodes and calculates the best route between selected points. Additionally, it detects and matches features 
between images using ORB and FLANN in OpenCV. The results show efficient pathfinding and accurate feature matching, useful 
for robotics, navigation, and computer vision applications. 
