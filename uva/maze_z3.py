from z3 import *
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtCore import Qt

# Define the maze size
maze_size = 10

# Define the maze as a 2D list
# 0 = empty cell, 1 = wall
maze = [
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 1, 1, 1, 1],
    [1, 0, 1, 1, 1, 0, 0, 0, 1, 1],
    [1, 0, 0, 0, 1, 1, 1, 0, 1, 1],
    [1, 1, 1, 0, 1, 0, 0, 0, 1, 1],
    [1, 0, 0, 0, 1, 1, 1, 0, 1, 1],
    [1, 0, 1, 1, 1, 0, 1, 0, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 1, 0, 1, 1, 1, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1]
]

# Entrance and Exit
entrance = (0, 1)
exit = (9, 7)

# Ensure the entrance and exit are unoccupied
maze[entrance[0]][entrance[1]] = 0
maze[exit[0]][exit[1]] = 0

# Visualize the maze using PyQt
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
                if self.maze[row][col] == 1:
                    painter.fillRect(x, y, self.cell_size, self.cell_size, QColor("blue"))
                elif self.maze[row][col] == 2:
                    painter.fillRect(x, y, self.cell_size, self.cell_size, QColor("green"))
                else:
                    painter.fillRect(x, y, self.cell_size, self.cell_size, QColor("grey"))
                painter.drawRect(x, y, self.cell_size, self.cell_size)

        # Draw the solution path
        pen = QPen(QColor("green"), 3)
        painter.setPen(pen)
        for i in range(len(self.solution_path) - 1):
            start = self.solution_path[i]
            end = self.solution_path[i + 1]
            start_x = start[1] * self.cell_size + self.cell_size // 2
            start_y = start[0] * self.cell_size + self.cell_size // 2
            end_x = end[1] * self.cell_size + self.cell_size // 2
            end_y = end[0] * self.cell_size + self.cell_size // 2
            painter.drawLine(start_x, start_y, end_x, end_y)

# Solve the maze with Z3
solver = Solver()

# Define Z3 variables for each cell
cells = [[Bool(f"cell_{r}_{c}") for c in range(maze_size)] for r in range(maze_size)]

# Constraints for the maze
for r in range(maze_size):
    for c in range(maze_size):
        if maze[r][c] == 1:  # Walls must not be part of the path
            solver.add(Not(cells[r][c]))

# Entrance and Exit must be part of the path
solver.add(cells[entrance[0]][entrance[1]])
solver.add(cells[exit[0]][exit[1]])

# A valid path must have adjacent steps (no diagonal moves)
for r in range(maze_size):
    for c in range(maze_size):
        if maze[r][c] == 0:  # Only consider empty cells
            neighbors = []
            if r > 0:
                neighbors.append(cells[r - 1][c])
            if r < maze_size - 1:
                neighbors.append(cells[r + 1][c])
            if c > 0:
                neighbors.append(cells[r][c - 1])
            if c < maze_size - 1:
                neighbors.append(cells[r][c + 1])
            solver.add(Implies(cells[r][c], Or(*neighbors)))

# Solve the maze
solution_path = []
if solver.check() == sat:
    model = solver.model()
    r, c = entrance
    while (r, c) != exit:
        solution_path.append((r, c))
        maze[r][c] = 2
        if r > 0 and model.evaluate(cells[r - 1][c]):
            r, c = r - 1, c
        elif r < maze_size - 1 and model.evaluate(cells[r + 1][c]):
            r, c = r + 1, c
        elif c > 0 and model.evaluate(cells[r][c - 1]):
            r, c = r, c - 1
        elif c < maze_size - 1 and model.evaluate(cells[r][c + 1]):
            r, c = r, c + 1
    solution_path.append(exit)
    maze[exit[0]][exit[1]] = 2
    print("Solution Path:", solution_path)
else:
    print("No solution found")

# Run the maze visualization
app = QApplication(sys.argv)
window = MazeWindow(maze, solution_path)
sys.exit(app.exec_())
