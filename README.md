# AutonomousRobotsPathPlanning

# Dijkstra's Algorithm for Path Planning

This Python application uses Dijkstra's algorithm to show path planning in a two-dimensional environment for a point robot. The algorithm avoids barriers and determines the shortest path between a starting point and a goal location.

## Overview of the Code

### Libraries `matplotlib} and `numpy` are used for numerical computations and plotting.
A polygon-defining obstacle definition tool is `matplotlib.patches`.
- 'math', 'copy', 'heapq', and 'time' for different utility functions.
- 'matplotlib.animation' to generate path animations.

### Constants and Variables
- {Workspace}: A list that indicates the workspace's measurements.
- The variables "node, CurrentNode, ParentNodeIndex, CurrentNodeIndex, Path, and NodePath are used to store path and node information.
- {clearance}: The distance needed to avoid obstructions.
- {BlankMap}: The blank map is represented by a numpy array.
- {obstacles}: Dictionary which represents polygon obstacles

### Features

1. {plot_obstacles_with_clearance(ax)}: This function plots obstacles on the specified axes that have clearance.
2. The function GenerateMap(ax) creates a map containing clearance zones and obstacles.
3. The function get_motion_model() defines the robot's potential motions.
4. {Dijkstra(start, goal, Map)}: This function applies Dijkstra's path-finding method.
5. {backtrack(target, distances)}: This function extracts the path by going backwards from the objective to the starting point.
6. {GetUserInput()}: This function allows user to enter goal and start positions.
7. {animate_solution(path, Map)}: An animation function for the path of the solution.

The script asks the user to enter the starting and goal coordinates during execution.
- The shortest path is then calculated and visualized using Dijkstra's algorithm.
- Additionally, a path animation is created and stored as "path_animation.mp4".

Executing the Program

To execute the code:
Install matplotlib and numpy, the necessary libraries.
Execute the script.
Enter the start and goal points by following the directions.


Note:- 
Although Dijkstra's algorithm ensures that the shortest path in terms of total cost will be found, it might not be the most effective in big environments or with intricate obstacle arrangements.
