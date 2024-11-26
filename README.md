
# 8-Puzzle Solver

This project is a graphical user interface (GUI) application for solving the 8-puzzle problem using various algorithms. Users can generate a random initial state, visualize solutions, and measure algorithm performance.

---

## Features
- Solve the 8-puzzle problem using:
  - **A*** with Manhattan heuristic
  - **A*** with Euclidean heuristic
  - **Breadth-First Search (BFS)**
  - **Depth-First Search (DFS)**
  - **Iterative Deepening Search (IDS)**
- GUI for interacting with the puzzle.
- Solution visualization step-by-step with metrics (nodes expanded, search depth, time).

---

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/AhmedSamir423/8_puzzle_game.git
   cd 8-puzzle-solver
   ```

2. Install Python 3.x and required libraries:
   ```bash
   pip install tk
   ```

---

## How to Run
1. Navigate to the project folder.
2. Execute the GUI application:
   ```bash
   python GUI.py
   ```

---

## File Structure
- `GUI.py`: Handles the GUI and user interactions.
- `State.py`: Contains the logic for puzzle states, move generation, and search algorithms.

---

## Usage
1. Run the application.
2. Choose a solving algorithm by clicking the corresponding button.
3. Generate a new random board by clicking **"Generate New Board"**.
4. Visualize step-by-step solution paths on the grid.

---

## Example Output
**Initial State**  
```
3 1 2
4 8 0
6 5 7
```

**Goal State**  
```
0 1 2
3 4 5
6 7 8
```

---

## Algorithms and Metrics
Each algorithm outputs the following metrics in the console:
- **Path to Goal**: Moves taken to solve the puzzle.
- **Nodes Expanded**: Total nodes explored during the search.
- **Search Depth**: Maximum depth reached.
- **Execution Time**: Total time to find the solution.

---

## Project Dependencies
- Python 3.x
- Tkinter (standard GUI library for Python)

---

## Example Visualization

### GUI Example:

1. **Initial Board**:  
   ![image](https://github.com/user-attachments/assets/c8af9a62-b045-4e64-9a51-e907230b1526)
 

2. **Solution**:  
   ![image](https://github.com/user-attachments/assets/8996d453-37dd-431e-a778-1c529892d8a8)

---

## Contributions
Feel free to fork and submit pull requests for improvements!

---

