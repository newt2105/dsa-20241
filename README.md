# Path Finding Algorithms

## Algorithms

### 1. Initialize Data Structures

- Retrieve `nodes` and `adjacencyList` from the `graph`.
- Find the `startNode` and `endNode` using their IDs (`start` and `end`).

- Create:
  - A map `gScore` to store the cost to reach each node from the start. Initialize all values to `infinity` except for `start` (`gScore[start] = 0`).
  - A map `fScore` to store the estimated total cost of the path through each node. Initialize all values to `infinity` except for `start` (`fScore[start] = heuristic(startNode, endNode)`).
  - A priority queue `openSet` to store nodes to be explored, sorted by `fScore`. Add the `start` node with its `fScore`.
  - A map `previous` to store the parent of each node in the path.

---

### 2. While `openSet` is Not Empty

1. Extract the node `current` with the lowest `fScore` from `openSet`.

2. If `current == end`:
   - Break the loop as the goal has been reached.

3. Iterate over each `neighbor` of `current`:
   - Compute `tentativeGScore = gScore[current] + edge.weight`.

4. If `tentativeGScore < gScore[neighbor]`:
   - Update `gScore[neighbor] = tentativeGScore`.
   - Update `fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, endNode)`.
   - Update `previous[neighbor] = current`.
   - If `neighbor` is not already in `openSet`, add it with its `fScore`.

---

### 3. Path Reconstruction

1. If `previous` does not contain a path to `end`, return an empty list (no path exists).
2. Start from `end` and backtrack using `previous` to build the path:
   - Add each node to the `path` until `start` is reached.
3. Reverse the `path` to ensure it starts at `start` and ends at `end`.

---

### 4. Return the Result

- Return the reconstructed `path`.
- Record and return the execution time for performance metrics.

---

## Dependencies

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install build-essential cmake ninja-build graphviz


```

## Clone Repository

```bash
git clone https://github.com/newt2105/dsa-20241.git
```

## Compile and Run

- Before compiling, update the file path of data.txt in src/ultis.cpp to your absolute path.
- Stay in the root directory where CMakeLists.txt is located and run this command:

## Build

```bash
cmake --preset=cpp-x86_64-release
cd ./out/build/cpp-x86_64-release
ninja
```

### Run

```bash
./PathFinding
```

## Test

## Notes

- Ensure that all dependencies are correctly installed to avoid issues with the build process.
- If you encounter issues with tests, verify that your file paths (such as `data.txt`) are correctly set.
