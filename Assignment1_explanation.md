Documentation of the Dijkstra's Algorithm in 3D Grid with Visualization Objective:
The objective of this implementation is to find the shortest paths between multiple pairs of start and end points in a 3D 
grid using Dijkstra's Algorithm. 
The grid contains cells with weights that impact the path cost, and the goal is to compute the least cost path between given start and end points. 
Additionally, the paths are visualized in 3D.

Algorithm Explanation:
Dijkstra's Algorithm is a classical algorithm for finding the shortest paths from a start node to all other nodes in graph. 
In this implementation, the graph is represented as a 3D grid, and the pathfinding involves navigating through grid points to find the minimum-weight path.

Key components:

Grid Representation: 
The 3D grid is represented as a 3D array (weights) where each point has a weight that determines the cost of moving 
throught that cell.

Movement Directions: 
Movement is allowed in 6 possible directions: positive and negative steps along each of the 3 axes (x, y, z).

Priority Queue: 
The algorithm uses a priority queue (min-heap) to ensure that the node with the smallest accumulated cost is processed 
 first.

Path Reconstruction: Once the end point is reached, the algorithm backtracks to reconstruct the path from start to end.

Step-by-Step Implementation:

Grid Initialization: 
The 3D grid size is defined as GRID_SIZE = 101. Weights are randomly generated for each point in the grid, with values 
between 1 and 10.

python:
np.random.seed(42)
weights = np.random.randint(1, 10, (GRID_SIZE, GRID_SIZE, GRID_SIZE))

Movement Directions: 
A list of possible movements in 3D space is defined, allowing movement in all 6 directions (positive and negative along x, 
y, z).

python:
DIRECTIONS = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]

Dijkstra's Algorithm: 
The algorithm uses a priority queue to explore the grid. For each point, the shortest cost to reach neighboring points is 
calculated and stored. The path is reconstructed once the destination is reached.

python:
def dijkstra_3d(start, end):
    pq = []
    heapq.heappush(pq, (0, start))  # (cost, (x, y, z))
    distances = np.full((GRID_SIZE, GRID_SIZE, GRID_SIZE), np.inf)
    distances[start] = 0
    parent = {}

    while pq:
        cost, (x, y, z) = heapq.heappop(pq)
        if (x, y, z) == end:
            path = []
            while (x, y, z) in parent:
                path.append((x, y, z))
                x, y, z = parent[(x, y, z)]
            path.append(start)
            path.reverse()
            return path
        
        for dx, dy, dz in DIRECTIONS:
            nx, ny, nz = x + dx, y + dy, z + dz
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and 0 <= nz < GRID_SIZE:
                new_cost = cost + weights[nx, ny, nz]
                if new_cost < distances[nx, ny, nz]:
                    distances[nx, ny, nz] = new_cost
                    parent[(nx, ny, nz)] = (x, y, z)
                    heapq.heappush(pq, (new_cost, (nx, ny, nz)))

    return None  # No path found

Multiple Start and End Points: 
Multiple start and end points are defined. The dijkstra_3d function is executed for each pair, and the resulting paths are collected.

python
start_points = [(0, 0, 0), (50, 50, 50)]
end_points = [(100, 100, 100), (80, 80, 80)]
paths = []

for start, end in zip(start_points, end_points):
    path = dijkstra_3d(start, end)
    if path:
        paths.append(path)

3D Path Visualization: 

The paths are visualized in 3D using matplotlib. Each path is plotted with markers at each step.

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for path in paths:
    xs, ys, zs = zip(*path)
    ax.plot(xs, ys, zs, marker='o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

Results and Visualization:

Paths Visualized in 3D: 
After running the code, the output will show a 3D plot where each line represents a path from a start point to an end point.

Feature Matches: 
The paths represent how the algorithm navigates through the 3D grid, with each point connected in sequence to show the path 
from start to end.

Example Output:

Two paths are computed and visualized:

From (0, 0, 0) to (100, 100, 100).
From (50, 50, 50) to (80, 80, 80).

3D Plot:
The plot will display two 3D lines with markers at each step, showing the movement across the grid. Each point on the path 
represents a coordinate in the grid that was traversed by the algorithm.

Conclusion:
This approach successfully finds the shortest path in a 3D grid with weighted points, visualizing the paths in 3D. 
Dijkstra's algorithm ensures optimal paths are chosen based on grid weights, while visualization helps in understanding the 
pathfinding process.
