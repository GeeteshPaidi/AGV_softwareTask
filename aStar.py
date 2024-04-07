from pyamaze import maze, agent, textLabel, COLOR
from queue import PriorityQueue
from random import randint


# f(n) = g(n) + h(n)
# g(n) = cost of the path from the start node to n
# h(n) = heuristic that estimates the cost of the cheapest path from n to the goal
def h(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return abs(x1 - x2) + abs(y1 - y2)  # Manhattan distance


maze_rows, maze_cols = 30, 30
start = (randint(1, maze_rows), randint(1, maze_cols))
goal = (randint(1, maze_rows), randint(1, maze_cols))


def a_Star(my_maze):
    g_score = {cell: float('inf') for cell in my_maze.grid}  # because we don't know the cost of the path to that cell
    f_score = {cell: float('inf') for cell in my_maze.grid}
    g_score[start] = 0  # the cost of the path to the start cell is 0
    f_score[start] = h(start, goal)

    open = PriorityQueue()
    # open.put(total cost, heuristic, cell) - the data of the cell for the priority
    open.put((f_score[start], h(start, goal), start))
    a_path = {}

    while not open.empty():
        curr_cell = open.get()[2]
        if curr_cell == goal:
            break
        for direction in 'ESNW':
            if my_maze.maze_map[curr_cell][direction] == 1:
                if direction == 'E':
                    neighbor = (curr_cell[0], curr_cell[1] + 1)
                elif direction == 'S':
                    neighbor = (curr_cell[0] + 1, curr_cell[1])
                elif direction == 'W':
                    neighbor = (curr_cell[0], curr_cell[1] - 1)
                else:
                    neighbor = (curr_cell[0] - 1, curr_cell[1])

                temp_g_score = g_score[curr_cell] + 1
                temp_f_score = temp_g_score + h(neighbor, goal)

                if temp_f_score < f_score[neighbor]:
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_f_score
                    open.put((f_score[neighbor], h(neighbor, goal), neighbor))

                    a_path[neighbor] = curr_cell
    forwardPath = {}
    cell = goal
    while cell != start:
        forwardPath[a_path[cell]] = cell
        cell = a_path[cell]
    return forwardPath


if __name__ == "__main__":
    m = maze(maze_rows, maze_cols)
    m.CreateMaze(x=goal[0], y=goal[1], loopPercent=50,
                 theme=COLOR.light)  # implies that there can be multiple paths to the final position

    path = a_Star(m)
    a = agent(m, start[0], start[1], footprints=True)
    m.tracePath({a: path})
    length = textLabel(m, "A* Algorithm path length", len(path) + 1)
    m.run()
