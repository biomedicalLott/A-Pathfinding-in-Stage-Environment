# A-Star-Pathfinding-in-Stage-Environment
C++ Implementation of A* pathfinding for navigating around a Stage map environment

No libraries are used, a custom node object is utilized. 

![image](https://user-images.githubusercontent.com/40275933/211929803-cdbb7890-37e7-4356-a014-49ed37df8765.png)

The objective of the assignment is simple. Use A-* planning algorithm to find a route
from a default start point (-8.0, -2.0) to a default goal (4.5, 9.0) and then to randomize the start and goal for testing. An occupancy grid was created from the map file itself having every pixel represent a whole number point on the map. 

Occupied spots would be marked as occupied and an averaging filter was applied across the default map to create a cost map to discourage movement near walls. 
