# getting all the necessary libraries
import pygame
import heapq
import itertools
import random
import time

# defining colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# defining grid cell size and dimensions
CELL_SIZE = 40
GRID_WIDTH = 15
GRID_HEIGHT = 15
WINDOW_SIZE = (GRID_WIDTH * CELL_SIZE, GRID_HEIGHT * CELL_SIZE)


# using A* algorithm with Manhattan distance as heuristic
def astar_pathfind(grid, start, goal):
    def manhattan_distance(pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def reconstruct_path(start_point, present_cell):
        path = []
        while present_cell in start_point:
            path.append(present_cell)
            present_cell = start_point[present_cell]
        path.append(start)
        path.reverse()
        return path

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current_cost, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in grid.neighbors(current):
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + manhattan_distance(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return None


# GridWorld Class to create the grid like environment for the agents
class GridWorld:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[0] * height for _ in range(width)]

    def neighbors(self, pos):
        x, y = pos
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.width > nx >= 0 == self.grid[nx][ny] and 0 <= ny < self.height:
                    yield nx, ny

    def set_obstacle(self, pos):
        x, y = pos
        self.grid[x][y] = 1

    def clear_obstacle(self, pos):
        x, y = pos
        self.grid[x][y] = 0

    def random_obstacles(self, num_obstacles):
        positions = random.sample([(x, y) for x in range(self.width) for y in range(self.height)], num_obstacles)
        for pos in positions:
            self.set_obstacle(pos)


# conflict-based Search (CBS) algorithm
class CBS:
    def __init__(self, world, agents):
        self.world = world
        self.agents = agents

    def find_path(self):
        start_positions = [agent['start'] for agent in self.agents]
        goal_positions = [agent['goal'] for agent in self.agents]

        while True:
            paths = []
            for i, agent in enumerate(self.agents):
                start = start_positions[i]
                goal = goal_positions[i]
                path = astar_pathfind(self.world, start, goal)
                if not path:
                    return None
                paths.append(path)

            conflicts = self.find_conflicts(paths)
            if not conflicts:
                return paths

            for conflict in conflicts:
                agent1, agent2, timestep = conflict
                if agent1 != agent2:
                    start_positions[agent1] = paths[agent1][timestep]
                    start_positions[agent2] = paths[agent2][timestep]

    def find_conflicts(self, paths):
        conflicts = []
        for (agent1, path1), (agent2, path2) in itertools.combinations(enumerate(paths), 2):
            for timestep in range(min(len(path1), len(path2))):
                if path1[timestep] == path2[timestep]:
                    conflicts.append((agent1, agent2, timestep))
        return conflicts


# using pygame to visualize
def draw_grid(screen, world, agents_positions):
    screen.fill(WHITE)

    # drawing grid lines
    for x in range(world.width + 1):
        pygame.draw.line(screen, BLACK, (x * CELL_SIZE, 0), (x * CELL_SIZE, world.height * CELL_SIZE))
    for y in range(world.height + 1):
        pygame.draw.line(screen, BLACK, (0, y * CELL_SIZE), (world.width * CELL_SIZE, y * CELL_SIZE))

    # for drawing obstacles
    for x in range(world.width):
        for y in range(world.height):
            if world.grid[x][y] == 1:
                pygame.draw.rect(screen, BLACK, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Draw agents
    for agent_idx, (x, y) in enumerate(agents_positions):
        color = (RED, GREEN, BLUE)[agent_idx]
        pygame.draw.circle(screen, color, (x * CELL_SIZE + CELL_SIZE // 2, y * CELL_SIZE + CELL_SIZE // 2),
                           CELL_SIZE // 4)

    pygame.display.flip()


def main():
    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Multi-Agent Pathfinding with CBS")

    world = GridWorld(GRID_WIDTH, GRID_HEIGHT)
    world.random_obstacles(20)  # placing 'n' random obstacles

    # set the starting and ending positions of each of the agents, here I am taking 3 agents (RGB)
    agents = [{'start': (1, 1), 'goal': (10, 9)},
              {'start': (14, 14), 'goal': (3, 4)},
              {'start': (1, 14), 'goal': (7, 2)}]
    cbs = CBS(world, agents)
    paths = cbs.find_path()

    if paths:
        running = True
        for timestep in range(max(len(path) for path in paths)):  # Iterate through each timestep
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            if not running:
                break

            current_positions = []
            for path in paths:
                if timestep < len(path):
                    current_positions.append(path[timestep])
                else:
                    current_positions.append(path[-1])  # Stay at last position if reached goal

            draw_grid(screen, world, current_positions)
            time.sleep(0.5)  # Add a delay of 0.5 seconds between each timestep

    else:
        print("No valid path found!")


if __name__ == "__main__":
    main()
