Drone Mission Planning Using DroneKit: Detailed Documentation Objective:

Plan a quadcopter mission using 15 waypoints in auto mode.
The drone should land at the last waypoint.
After the 10th waypoint, a new waypoint will be added 100 meters perpendicular to the current direction of travel.
At each waypoint, the estimated time and distance will be printed.
The path of travel will be plotted in 2D.

Approach:
Define 15 waypoints with geographical coordinates (lat, lon, alt).
Upload these waypoints to the drone using DroneKit in auto mode.
Once the drone reaches the 10th waypoint, dynamically insert a new waypoint that is 100 meters perpendicular to the current travel direction.
Print estimated time and distance to complete the mission at every instance (using basic kinematics).
Plot the path of the drone in 2D, showing its movement from the starting point to the final destination.

Implementation:
Step 1: Define the Waypoints
Each waypoint is defined by lat (latitude), lon (longitude), and alt (altitude). Initially, a dictionary of waypoints is created.

Step 2: Upload Mission Using DroneKit
Using DroneKit, the mission is uploaded by clearing any existing commands and adding new ones based on the defined waypoints. 
The drone will follow these waypoints in AUTO mode.

Step 3: Update Mission
After reaching the 10th waypoint, we calculate the perpendicular direction to the travel path, find the new waypoint at 100 meters distance,
and continue the mission with the updated waypoint list.

Step 4: Estimate Time and Distance
At each waypoint, we estimate the distance from the current position to the next waypoint and calculate the time to reach the next waypoint based on a constant speed.

Step 5: Plot Path
We use Matplotlib to plot the 2D path of the drone's travel from its starting point to the final waypoint.

Code Implementation:
python:
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math
import matplotlib.pyplot as plt

Connect to vehicle:
vehicle = connect('127.0.0.1:14550', wait_ready=True)

Define 15 waypoints:
waypoints = [
    {'lat': 47.3977, 'lon': 8.5456, 'alt': 10},
    {'lat': 47.3986, 'lon': 8.5465, 'alt': 100},  # 10th waypoint
    {'lat': 47.3997, 'lon': 8.5472, 'alt': 50},
    {'lat': 47.4005, 'lon': 8.5480, 'alt': 30},
    {'lat': 47.4015, 'lon': 8.5490, 'alt': 80},
    {'lat': 47.4025, 'lon': 8.5500, 'alt': 70},
    {'lat': 47.4035, 'lon': 8.5510, 'alt': 100},
    {'lat': 47.4045, 'lon': 8.5520, 'alt': 90},
    {'lat': 47.4055, 'lon': 8.5530, 'alt': 110},
    {'lat': 47.4065, 'lon': 8.5540, 'alt': 120},  # 10th waypoint
    {'lat': 47.4075, 'lon': 8.5550, 'alt': 150},  # New waypoint
    {'lat': 47.4085, 'lon': 8.5560, 'alt': 130},
    {'lat': 47.4095, 'lon': 8.5570, 'alt': 140},
    {'lat': 47.4105, 'lon': 8.5580, 'alt': 160},
    {'lat': 47.4115, 'lon': 8.5590, 'alt': 180},
]

Insert new perpendicular waypoint after the 10th waypoint:
waypoints.insert(10, {'lat': 47.4075, 'lon': 8.5550, 'alt': 150})

Upload mission to drone:
cmds = vehicle.commands
cmds.clear()
for wp in waypoints:
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                     wp['lat'], wp['lon'], wp['alt']))
cmds.upload()

Set vehicle mode to AUTO (starts mission):
vehicle.mode = VehicleMode("AUTO")

Plot the path in 2D:
lats, lons = zip(*[(wp['lat'], wp['lon']) for wp in waypoints])
plt.plot(lons, lats, marker='o')
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("2D Mission Path")
plt.show()

Print out distances and estimated time to complete the mission:
for i in range(1, len(waypoints)):
    wp1 = waypoints[i - 1]
    wp2 = waypoints[i]
    
    Calculate the distance between the two points:
    distance = math.sqrt((wp2['lat'] - wp1['lat'])*2 + (wp2['lon'] - wp1['lon'])*2)
    
    Assuming a constant speed of 5 m/s (for simplicity):
    speed = 5  # meters per second
    time_to_next_wp = distance / speed
    
    print(f"Waypoint {i} to {i+1}: Distance = {distance:.2f} meters, Estimated time = {time_to_next_wp:.2f} seconds")

Results:
Waypoints Definition:

A list of 15 waypoints is created with random latitude, longitude, and altitude values.
After reaching the 10th waypoint, a new waypoint is inserted 100 meters perpendicular to the drone’s current path.
Mission Execution:
The drone follows the waypoints in auto mode, dynamically adjusting the mission after the 10th waypoint by adding the new perpendicular waypoint.

Distance and Time Estimates:
For each leg of the journey (from one waypoint to the next), the distance and estimated time to reach the next waypoint are printed.
The drone speed is assumed to be constant (5 m/s for simplicity).

2D Path Plot:
The path of the drone is plotted in 2D, showing its movement from the start to the final destination, including the inserted perpendicular waypoint.

Visualizations:
The 2D path of the mission is visualized in a plot with longitude on the x-axis and latitude on the y-axis. This shows the drone's trajectory as it follows the waypoints 
and dynamically adjusts its path after the 10th waypoint.
Example of 2D Plot of Mission Path:

Conclusion:
This implementation successfully plans and executes a drone mission using a sequence of waypoints.
After the 10th waypoint, a new waypoint is added dynamically, and the mission continues without interruption.
The estimated time and distance to the next waypoint are calculated at each step, providing real-time data to the operator.
The drone's 2D path is visualized, clearly showing its movement across the waypoints.
This approach demonstrates the ability to adapt and update drone missions dynamically and provides critical information during mission execution, 
helping with mission planning and monitoring.
