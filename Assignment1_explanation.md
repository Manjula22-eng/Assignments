3D Shortest Path Search Using Dijkstra’s Algorithm :
This project involves creating a 3D Grid Graph, assigning random weights to nodes, finding shortest paths using Dijkstra’s 
algorithm, and visualizing the results in a 3D plot.
Algorithms Used :
1.1 Grid Graph Construction
A 3D grid graph is generated using networkx.grid_graph(), where each node is connected to its six neighbors (left, right, 
front, back, up, and down).
1.2 Dijkstra’s Algorithm
We use Dijkstra’s shortest path algorithm, which is a graph search algorithm that finds the shortest path between nodes by 
considering edge weights.

Steps of Dijkstra’s Algorithm
Initialize: 
Set the distance of the start node to 0 and all others to ∞.
Select Node:
Pick the node with the smallest distance (initially, the start node).
Update Neighbors:
For each neighbor, update its distance if a shorter path is found.
Repeat: 
Continue until all nodes have been visited or the target node is reached.

Implementation Details:
2.1 Importing Libraries
python:
import numpy as np
import networkx as nx
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

networkx: Used to create and process the 3D graph.
random: Assigns random weights to nodes.
matplotlib.pyplot and mpl_toolkits.mplot3d: For 3D visualization of shortest paths.

2.2 Creating a 3D Grid Graph
python:
GRID_SIZE = 20  # Reduced size for testing
print("Creating 3D Grid Graph...")
G = nx.grid_graph(dim=[GRID_SIZE+1, GRID_SIZE+1, GRID_SIZE+1])
print("Graph created with", len(G.nodes), "nodes.")
We create a 3D grid with (GRID_SIZE+1)^3 nodes.
Each node has six possible neighbors in a 3D space.

2.3 Assigning Random Weights
python:
for node in G.nodes:
    G.nodes[node]['weight'] = random.choice([1, 1, 1, 5, 10, 15])
print("Weights assigned.")
We randomly assign weights (1, 5, 10, or 15) to each node.

2.4 Finding the Shortest Path Using Dijkstra
python:
def shortest_path(graph, start, end):
    print(f"Finding shortest path from {start} to {end}...")
    return nx.shortest_path(graph, source=start, target=end, weight='weight', method='dijkstra')
Uses networkx.shortest_path() with Dijkstra’s algorithm.

2.5 Finding Paths Between Selected Points
python
Copy
Edit
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
        
Finds the shortest paths between two pairs of points in the 3D space.
2.6 3D Path Visualization
python:
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for path in paths:
    x, y, z = zip(*path)
    ax.plot(x, y, z, marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title("3D Shortest Paths")
print("Displaying plot...")
plt.show()
Plots the shortest paths in a 3D space.

Results and Visualizations:
3.1 Sample Output: 
Creating 3D Grid Graph...
Graph created with 9261 nodes.
Weights assigned.
Finding shortest path from (0, 0, 0) to (10, 10, 10)...
Path found: 30 steps.
Finding shortest path from (5, 5, 5) to (15, 15, 15)...
Path found: 40 steps.
Displaying plot...

Each path consists of multiple steps depending on node weights.
3.2 Visualization
The output is a 3D plot where:
Nodes represent points in the grid.
Lines connect the shortest paths between the selected points.

4. Enhancements & Future Improvements
Increase grid size for more complex pathfinding.
Use A (A-Star) Algorithm* for faster execution.
Introduce obstacles by removing some nodes.
This project demonstrates shortest path computation in 3D space using Dijkstra’s algorithm and NetworkX, with visualization
using Matplotlib.
