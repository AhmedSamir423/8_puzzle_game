goal_State = [[7,2,4], [5,0,6], [8,3,1]]

class State:
    def __init__(self, board):
        self.board = board
        self.zerpos = self.get_zero_position()

    def get_possible_moves(self):
        directions = {
            (-1, 0): "up", 
            (1, 0): "down", 
            (0, -1): "left", 
            (0, 1): "right"
        }
        x, y = self.get_zero_position()
        moves = []
        for d, name in directions.items():
            if 0 <= x + d[0] < 3 and 0 <= y + d[1] < 3:
                moves.append((d, name))
        return moves

    def get_zero_position(self):
        for i in range(3):
            for j in range(3):
                if self.board[i][j] == 0:
                    return i, j

    def apply_move(self, move):
        direction, name = move
        x, y = self.get_zero_position()
        new_board = [row[:] for row in self.board]
        new_board[x][y], new_board[x + direction[0]][y + direction[1]] = new_board[x + direction[0]][y + direction[1]], new_board[x][y]
        return State(new_board), name

    def is_goal(self):
        return self.board == goal_State

    def __hash__(self):
        return hash(tuple(tuple(row) for row in self.board))

    def __eq__(self, other):
        return self.board == other.board

class Node:
    def __init__(self, state, parent=None, action=None, cost=0, depth=0):
        self.state = state
        self.parent = parent
        self.action = action  # Store move name here (up, down, left, right)
        self.cost = cost
        self.depth = depth

    def get_path(self):
        path = []
        current_node = self
        while current_node.parent is not None:
            path.append(current_node.action)
            current_node = current_node.parent
        path.reverse()  # Reverse to get actions from start to goal
        return path

def bfs(initial_state):
    queue = [Node(initial_state)]
    visited = set()

    while queue:
        current_node = queue.pop(0)
        current_state = current_node.state

        if current_state.is_goal():
            return current_node.get_path()

        visited.add(current_state)

        for move in current_state.get_possible_moves():
            new_state, move_name = current_state.apply_move(move)
            if new_state not in visited:
                queue.append(Node(new_state, current_node, move_name, current_node.cost, current_node.depth + 1))

    return None

def dfs(initial_state):
    stack = [Node(initial_state)]
    visited = set()

    while stack:
        current_node = stack.pop()
        current_state = current_node.state

        if current_state.is_goal():
            return current_node.get_path()

        visited.add(current_state)

        for move in current_state.get_possible_moves():
            new_state, move_name = current_state.apply_move(move)
            if new_state not in visited:
                stack.append(Node(new_state, current_node, move_name, current_node.cost, current_node.depth + 1))

    return None

def ids(initial_state):
    depth_limit = 0

    while True:
        stack = [Node(initial_state)]
        visited = set()
        found_solution = False

        while stack:
            current_node = stack.pop()
            current_state = current_node.state

            if current_state.is_goal():
                found_solution = True
                return current_node.get_path()

            visited.add(current_state)

            if current_node.depth < depth_limit:
                for move in current_state.get_possible_moves():
                    new_state, move_name = current_state.apply_move(move)
                    if new_state not in visited:
                        stack.append(Node(new_state, current_node, move_name, current_node.cost, current_node.depth + 1))

        depth_limit += 1

def manhattan_heuristic(state):
    distance = 0
    for i in range(3):
        for j in range(3):
            if state.board[i][j] != 0:
                goal_row = (state.board[i][j] - 1) // 3
                goal_col = (state.board[i][j] - 1) % 3
                distance += abs(i - goal_row) + abs(j - goal_col)
    return distance

def eclidean_heuristic(state):
    distance = 0
    for i in range(3):
        for j in range(3):
            if state.board[i][j] != 0:
                goal_row = (state.board[i][j] - 1) // 3
                goal_col = (state.board[i][j] - 1) % 3
                distance += ((i - goal_row) ** 2 + (j - goal_col) ** 2) ** 0.5
    return distance


# Test Code
initial_board = [[1, 2, 3], [4, 0, 5], [7, 8, 6]]
initial_state = State(initial_board)

# Run searches and print paths
solution_path_bfs = bfs(initial_state)
solution_path_dfs = dfs(initial_state)
solution_path_ids = ids(initial_state)

# Print the solution paths
if solution_path_bfs:
    print("BFS Solution found! Moves:", solution_path_bfs)
    print("BFS number of moves:", len(solution_path_bfs))
else:
    print("BFS: No solution found.")

if solution_path_dfs:
#    print("DFS Solution found! Moves:", solution_path_dfs)
    print("DFS number of moves:", len(solution_path_dfs))
else:
   print("DFS: No solution found.")

if solution_path_ids:
    print("IDFS Solution found! Moves:", solution_path_ids)
    print("IDFS number of moves:", len(solution_path_ids))
else:
    print("IDFS: No solution found.")
