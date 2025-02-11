from z3 import *
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtCore import Qt

# Define the maze size
maze_size = 5

# The maze as a 2D list
# 0 = empty cell, 1 = wall
maze = [
    [1, 0, 1, 1, 1],
    [1, 0, 1, 1, 1],
    [1, 0, 0, 0, 1],
    [1, 1, 1, 0, 1],
    [1, 1, 1, 0, 1]
]

# Entrance and Exit
entrance = (1, 0)  # (x, y) format
exit = (3, 4)

# Ensure the entrance and exit are unoccupied
maze[entrance[1]][entrance[0]] = 0
maze[exit[1]][exit[0]] = 0

# Print the initial maze
def print_maze(maze):
    print("Initial Maze:")
    for row in range(maze_size):
        line = ""
        for col in range(maze_size):
            if (col, row) == entrance:
                line += " S "  # Mark the start
            elif (col, row) == exit:
                line += " E "  # Mark the end
            elif maze[row][col] == 1:
                line += " # "
            else:
                line += " . "
        print(line)

print_maze(maze)

# PyQt visualization of the maze
class MazeWindow(QMainWindow):
    def __init__(self, maze, solution_path):
        super().__init__()
        self.maze = maze
        self.solution_path = solution_path
        self.cell_size = 40
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Maze")
        self.setGeometry(100, 100, len(self.maze[0]) * self.cell_size, len(self.maze) * self.cell_size)
        self.show()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("grey"))

        for row in range(len(self.maze)):
            for col in range(len(self.maze[0])):
                x = col * self.cell_size
                y = row * self.cell_size
                if (col, row) == entrance:
                    painter.fillRect(x, y, self.cell_size, self.cell_size, QColor("red"))
                elif (col, row) == exit:
                    painter.fillRect(x, y, self.cell_size, self.cell_size, QColor("green"))
                elif self.maze[row][col] == 1:
                    painter.fillRect(x, y, self.cell_size, self.cell_size, QColor("blue"))
                elif self.maze[row][col] == 2:
                    painter.fillRect(x, y, self.cell_size, self.cell_size, QColor("grey"))
                else:
                    painter.fillRect(x, y, self.cell_size, self.cell_size, QColor("white"))
                painter.drawRect(x, y, self.cell_size, self.cell_size)

        # Draw the solution path
        pen = QPen(QColor("green"), 3)
        painter.setPen(pen)
        for i in range(len(self.solution_path) - 1):
            start = self.solution_path[i]
            end = self.solution_path[i + 1]
            start_x = start[0] * self.cell_size + self.cell_size // 2
            start_y = start[1] * self.cell_size + self.cell_size // 2
            end_x = end[0] * self.cell_size + self.cell_size // 2
            end_y = end[1] * self.cell_size + self.cell_size // 2
            painter.drawLine(start_x, start_y, end_x, end_y)

# Solve the maze with Z3
def solve_maze():
    solver = Solver()

    # Define Z3 variables for each cell
    cells = [[Bool(f"cell_{x}_{y}") for y in range(maze_size)] for x in range(maze_size)]
    weights = [[-999 if maze[y][x] == 1 else 1 for y in range(maze_size)] for x in range(maze_size)]

    # Constraints for the maze
    for y in range(maze_size):
        for x in range(maze_size):
            if maze[y][x] == 1:  # Walls must not be part of the path
                solver.add(Not(cells[x][y]))

    # Entrance and Exit must be part of the path
    solver.add(cells[entrance[0]][entrance[1]])
    solver.add(cells[exit[0]][exit[1]])

    # A valid path must have adjacent steps (no diagonal moves)
    for y in range(maze_size):
        for x in range(maze_size):
            if maze[y][x] == 0:  # Only consider empty cells
                neighbors = []
                if x > 0:
                    neighbors.append(cells[x - 1][y])
                if x < maze_size - 1:
                    neighbors.append(cells[x + 1][y])
                if y > 0:
                    neighbors.append(cells[x][y - 1])
                if y < maze_size - 1:
                    neighbors.append(cells[x][y + 1])
                solver.add(Implies(cells[x][y], Or(*neighbors)))

    # Calculate the total weight of the path
    path_weight = Sum([If(cells[x][y], weights[x][y], 0) for x in range(maze_size) for y in range(maze_size)])
    solver.add(path_weight >= 0)  # Ensure the path weight is non-negative

    # Solve the maze and extract the solution path
    solution_path = []
    if solver.check() == sat:
        model = solver.model()
        solution_path = [(x, y) for x in range(maze_size) for y in range(maze_size) if model.evaluate(cells[x][y])]
        for x, y in solution_path:
            maze[y][x] = 2
        print("Solution Path:", solution_path)
        print("Path Weight:", sum(weights[p[0]][p[1]] for p in solution_path))
    else:
        print("No solution found")
    return solution_path

# Get the solution path
solution_path = solve_maze()

# Run the maze visualization
app = QApplication(sys.argv)
window = MazeWindow(maze, solution_path)
sys.exit(app.exec_())
