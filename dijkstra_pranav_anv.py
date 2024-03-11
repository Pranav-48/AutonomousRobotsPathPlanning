import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import copy
import heapq
import time
import math
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

Workspace = [1200, 500]
node = []
CurrentNode = []
ParentNodeIndex = []
CurrentNodeIndex = 0
Path = []
NodePath = []
clearance = 5
BlankMap = np.zeros((500, 1200))

obstacles = {
    'obstacle1': [(100, 100), (100, 500), (175, 500), (175, 100)],
    'obstacle2': [(275, 0), (275, 400), (350, 400), (350, 0)],
    'obstacle3': [(520, 325), (650, 400), (780, 325), (780, 175), (650, 100), (520, 175)],
    'obstacle4': [(900, 450), (1100, 450), (1100, 50), (900, 50), (900, 125), (1020, 125), (1020, 375), (900, 375)],
}

def plot_obstacles_with_clearance(ax):
    for key, vertices in obstacles.items():
        expanded_vertices = [(x - clearance, y - clearance) for x, y in vertices]
        polygon = patches.Polygon(expanded_vertices, closed=True, color='gray', fill=True, alpha=0.3)
        ax.add_patch(polygon)

        outline_vertices = [(x, y) for x, y in vertices]
        buffered_polygon = patches.Polygon(expanded_vertices, closed=True, linewidth=2, edgecolor='black', fill=False)
        ax.add_patch(buffered_polygon)

    ax.set_xticks(np.arange(0, Workspace[0] + 1, 100))
    ax.set_yticks(np.arange(0, Workspace[1] + 1, 100))
    ax.set_xticklabels(np.arange(0, Workspace[0] + 1, 100))
    ax.set_yticklabels(np.arange(0, Workspace[1] + 1, 100))

def GenerateMap(ax):
    Map = copy.deepcopy(BlankMap)
    for obstacle, vertices in obstacles.items():
        for x in range(max(0, int(min(v[0] for v in vertices)) - clearance),
                       min(Workspace[0], int(max(v[0] for v in vertices)) + clearance + 1)):
            for y in range(max(0, int(min(v[1] for v in vertices)) - clearance),
                           min(Workspace[1], int(max(v[1] for v in vertices)) + clearance + 1)):
                Map[y][x] = 1
    return Map

def get_motion_model():
    motion = [
        [1, 0, 1],
        [0, 1, 1],
        [-1, 0, 1],
        [0, -1, 1],
        [-1, -1, math.sqrt(2)],
        [-1, 1, math.sqrt(2)],
        [1, -1, math.sqrt(2)],
        [1, 1, math.sqrt(2)]
    ]
    return motion

def Dijkstra(start, goal, Map):
    global Workspace

    motion_model = get_motion_model()
    distances = [[float('inf')] * Workspace[0] for _ in range(Workspace[1])]
    distances[start[1]][start[0]] = 0
    visited = [[False] * Workspace[0] for _ in range(Workspace[1])]

    pq = [(0, start)]
    heapq.heapify(pq)

    while pq:
        current_dist, (current_x, current_y) = heapq.heappop(pq)
        if (current_x, current_y) == goal:
            break

        if visited[current_y][current_x]:
            continue

        visited[current_y][current_x] = True

        for dx, dy, cost in motion_model:
            new_x, new_y = current_x + dx, current_y + dy

            if (0 <= new_x < Workspace[0] and 0 <= new_y < Workspace[1] and
                Map[new_y][new_x] != 1 and not visited[new_y][new_x]):
                new_dist = current_dist + cost

                if new_dist < distances[new_y][new_x]:
                    distances[new_y][new_x] = new_dist
                    heapq.heappush(pq, (new_dist, (new_x, new_y)))

    return distances

def backtrack(goal, distances):
    global Workspace

    path = [goal]
    current_x, current_y = goal

    while distances[current_y][current_x] != 0:
        motion_model = get_motion_model()
        min_dist = float('inf')
        next_step = None
        for dx, dy, _ in motion_model:
            new_x, new_y = current_x + dx, current_y + dy
            if 0 <= new_x < Workspace[0] and 0 <= new_y < Workspace[1]:
                if distances[new_y][new_x] < min_dist:
                    min_dist = distances[new_y][new_x]
                    next_step = (new_x, new_y)
        path.append(next_step)
        current_x, current_y = next_step

    return path[::-1]

def GetUserInput():
    while True:
        print("Enter the x coordinate of the starting node:")
        start_x = int(input())
        print("Enter the y coordinate of the starting node:")
        start_y = int(input())

        if any(polygon.contains_point((start_x, start_y)) for vertices in obstacles.values() for polygon in [patches.Polygon(vertices)]):
            print("Point is inside an obstacle. Please enter a valid point.")
            continue

        break

    while True:
        print("Enter the x coordinate of the goal node:")
        goal_x = int(input())
        print("Enter the y coordinate of the goal node:")
        goal_y = int(input())

        if any(polygon.contains_point((goal_x, goal_y)) for vertices in obstacles.values() for polygon in [patches.Polygon(vertices)]):
            print("Point is inside an obstacle. Please enter a valid point.")
            continue

        break

    return [start_x, start_y], [goal_x, goal_y]

StartNode, GoalNode = GetUserInput()

fig, ax = plt.subplots(figsize=(12, 5))
ax.set_xlim(0, Workspace[0])
ax.set_ylim(0, Workspace[1])
ax.set_aspect('equal')

plot_obstacles_with_clearance(ax)
ax.plot(StartNode[0], StartNode[1], "rx", markersize='15', label='Start Node')
ax.plot(GoalNode[0], GoalNode[1], "go", markersize='15', label='Goal Node')
Map = GenerateMap(ax)

start_time = time.time()
distances = Dijkstra(StartNode, GoalNode, Map)
end_time = time.time()
print("Time taken for Dijkstra's Algorithm: ", end_time - start_time)
path = backtrack(GoalNode, distances)


for i in range(len(path) - 1):
    ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], 'b-', linewidth=2)

plt.grid(True)
plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=2, fancybox=True, shadow=True)
plt.show()

def animate_solution(path, Map):
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_xlim(0, Workspace[0])
    ax.set_ylim(0, Workspace[1])
    ax.set_aspect('equal')

    plot_obstacles_with_clearance(ax)
    ax.plot(StartNode[0], StartNode[1], "rx", markersize='15', label='Start Node')
    ax.plot(GoalNode[0], GoalNode[1], "go", markersize='15', label='Goal Node')

    line, = ax.plot([], [], 'b-', linewidth=2)

    def init():
        line.set_data([], [])
        return line,

    def update(frame):
        x = [path[i][0] for i in range(frame + 1)]
        y = [path[i][1] for i in range(frame + 1)]
        line.set_data(x, y)
        return line,

    anim = FuncAnimation(fig, update, frames=len(path), init_func=init, blit=True)
    anim.save('path_animation.mp4', writer='ffmpeg', fps=60)

animate_solution(path, Map)
