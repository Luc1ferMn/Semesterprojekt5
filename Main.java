import java.util.*;

public class Main {
    public static void main(String[] args) {
        // Create and solve 4 different mazes with increasing dimensions
        int[][] mazeDimensions = { {20, 20}, {40, 40}, {60, 60}, {100, 100} };
        for (int i = 0; i < mazeDimensions.length; i++) {
            int width = mazeDimensions[i][0];
            int height = mazeDimensions[i][1];
            System.out.println("\nMaze " + (i + 1) + " (" + width + "x" + height + "):");
            Maze maze = new Maze(width, height);
            maze.createMaze();
            System.out.println("Initial Maze:");
            maze.showMaze();
            System.out.println("Path exists: " + maze.hasPath());
            
            System.out.println("\nDijkstra's Algorithm Path for Maze " + (i + 1) + ":");
            int[] dijkstraPath = maze.findShortestPath();
            maze.showPath(dijkstraPath);
            
            System.out.println("\nA* Algorithm Path for Maze " + (i + 1) + ":");
            int[] aStarPath = maze.findAStarPath();
            maze.showPath(aStarPath);
            
            System.out.println("\n=====================================================");
        }
    }
}

class Maze {
    private final int width, height;
    private final int[] maze;
    
    public Maze(int width, int height) {
        // Constructor initializes maze dimensions and creates an empty maze array.
        // The decision to use a 1D array instead of a 2D array was made to simplify indexing
        // and memory management, as Java arrays are naturally 1D and we can calculate positions manually.
        this.width = width;
        this.height = height;
        this.maze = new int[width * height];
    }

    public void createMaze() {
        // Create approximately 50% walls, ensuring there is always a path.
        // The wall creation is followed by a check to ensure a valid path still exists.
        // This ensures the maze is challenging but always solvable.
        for (int i = 0; i < (width * height) / 2; i++) {
            int index = random(1, width * height - 1);
            maze[index] = 1; // Create wall
            if (!hasPath()) {
                maze[index] = 0; // Remove wall if it blocks the path
            }
        }
    }

    private int random(int min, int max) {
        // Generates a random number between min (inclusive) and max (exclusive).
        // Math.random() was chosen for simplicity; in practice, more secure random functions could be used.
        return min + (int) (Math.random() * (max - min));
    }

    private boolean isWall(int x, int y) {
        // Checks if a particular position in the maze is a wall.
        // Using a separate method for wall checking improves code readability and encapsulates the logic.
        return maze[y * width + x] == 1;
    }

    public void showMaze() {
        // Displays the maze in a visual format where 'X' represents walls and '_' represents open paths.
        // This method provides a simple visualization for debugging and understanding the maze structure.
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                System.out.print(isWall(x, y) ? "X" : "_");
            }
            System.out.println();
        }
    }

    public boolean hasPath() {
        // Determines if there is a path from the start (top-left) to the end (bottom-right).
        // This method uses depth-first search (DFS) to verify connectivity.
        boolean[] visited = new boolean[width * height];
        depthFirstSearch(visited, 0, 0);
        return visited[width * height - 1];
    }

    private void depthFirstSearch(boolean[] visited, int x, int y) {
        // Recursively visits nodes in the maze to determine connectivity.
        // We use DFS because it is straightforward and efficient for determining simple reachability.
        // Boundary checks and wall checks ensure we do not move out of bounds or into walls.
        if (x < 0 || x >= width || y < 0 || y >= height || isWall(x, y) || visited[y * width + x]) {
            return;
        }
        visited[y * width + x] = true;
        
        // Visit all four neighbors (left, right, up, down) to explore all possible paths.
        depthFirstSearch(visited, x - 1, y);
        depthFirstSearch(visited, x + 1, y);
        depthFirstSearch(visited, x, y - 1);
        depthFirstSearch(visited, x, y + 1);
    }

    public int[] findShortestPath() {
        // Finds the shortest path from the start to the end using Dijkstra's algorithm.
        // Dijkstra's was chosen because it guarantees finding the shortest path in weighted or unweighted graphs.
        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingInt(node -> node.distance));
        boolean[] visited = new boolean[width * height];
        int[] distances = new int[width * height];
        int[] previous = new int[width * height];
        Arrays.fill(distances, Integer.MAX_VALUE);
        distances[0] = 0;
        queue.add(new Node(0, 0));

        while (!queue.isEmpty()) {
            Node current = queue.poll();
            int index = current.index;
            if (visited[index]) continue;
            visited[index] = true;

            // Calculate neighbors of the current node to explore possible moves.
            // The neighbors are calculated by considering all four possible directions.
            int[] neighbors = getNeighbors(index);
            for (int neighbor : neighbors) {
                if (neighbor == -1 || isWall(neighbor % width, neighbor / width) || visited[neighbor]) continue;
                int newDist = distances[index] + 1;
                if (newDist < distances[neighbor]) {
                    // If a shorter path to the neighbor is found, update the distance and path.
                    distances[neighbor] = newDist;
                    previous[neighbor] = index;
                    queue.add(new Node(neighbor, newDist));
                }
            }
        }

        // Trace the path from the end to the start using the previous array.
        // This reconstructs the path from the destination back to the start.
        int[] path = new int[width * height];
        int currentIndex = width * height - 1;
        if (distances[currentIndex] == Integer.MAX_VALUE) {
            System.out.println("No path found");
            return path;
        }
        System.out.println("Path length: " + distances[currentIndex]);
        while (currentIndex != 0) {
            path[currentIndex] = 1;
            currentIndex = previous[currentIndex];
        }
        path[0] = 1;
        return path;
    }

    public int[] findAStarPath() {
        // Finds the shortest path from the start to the end using the A* algorithm.
        // A* is chosen here because it typically finds paths faster by using a heuristic to guide the search.
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingInt(node -> node.distance));
        boolean[] visited = new boolean[width * height];
        int[] gScore = new int[width * height]; // Cost from start to this node
        int[] fScore = new int[width * height]; // Estimated total cost from start to end through this node
        int[] previous = new int[width * height];
        Arrays.fill(gScore, Integer.MAX_VALUE);
        Arrays.fill(fScore, Integer.MAX_VALUE);
        gScore[0] = 0;
        fScore[0] = heuristic(0);
        openSet.add(new Node(0, fScore[0]));

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();
            int index = current.index;
            if (index == width * height - 1) {
                // Reached the goal
                break;
            }
            if (visited[index]) continue;
            visited[index] = true;

            // Calculate neighbors of the current node to explore possible moves.
            int[] neighbors = getNeighbors(index);
            for (int neighbor : neighbors) {
                if (neighbor == -1 || isWall(neighbor % width, neighbor / width) || visited[neighbor]) continue;
                int tentativeGScore = gScore[index] + 1;
                if (tentativeGScore < gScore[neighbor]) {
                    // Update path and scores if a better path is found
                    previous[neighbor] = index;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + heuristic(neighbor);
                    openSet.add(new Node(neighbor, fScore[neighbor]));
                }
            }
        }

        // Trace the path from the end to the start using the previous array.
        // This reconstructs the path from the destination back to the start.
        int[] path = new int[width * height];
        int currentIndex = width * height - 1;
        if (gScore[currentIndex] == Integer.MAX_VALUE) {
            System.out.println("No path found");
            return path;
        }
        System.out.println("Path length: " + gScore[currentIndex]);
        while (currentIndex != 0) {
            path[currentIndex] = 1;
            currentIndex = previous[currentIndex];
        }
        path[0] = 1;
        return path;
    }

    private int heuristic(int index) {
        // Heuristic function for A* algorithm.
        // Here we use the Manhattan distance between the current node and the goal as the heuristic.
        int x = index % width;
        int y = index / width;
        int goalX = (width * height - 1) % width;
        int goalY = (width * height - 1) / width;
        return Math.abs(x - goalX) + Math.abs(y - goalY);
    }

    private int[] getNeighbors(int index) {
        // Returns the neighbors of the given index in the maze.
        // This method encapsulates neighbor calculation logic, ensuring that all boundary conditions are handled consistently.
        int x = index % width;
        int y = index / width;
        return new int[] {
            (x > 0) ? index - 1 : -1,           // Left neighbor (if not on left boundary)
            (x < width - 1) ? index + 1 : -1,   // Right neighbor (if not on right boundary)
            (y > 0) ? index - width : -1,       // Top neighbor (if not on top boundary)
            (y < height - 1) ? index + width : -1 // Bottom neighbor (if not on bottom boundary)
        };
    }

    public void showPath(int[] path) {
        // Displays the shortest path in the maze where 'X' represents the path and '_' represents open space.
        // This visualization helps to verify if the correct path was found.
        System.out.println();
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                System.out.print(path[x + y * width] > 0 ? "X" : "_");
            }
            System.out.println();
        }
    }

    private static class Node {
        int index;
        int distance;

        Node(int index, int distance) {
            // Represents a node with its index in the maze and distance from the start.
            // This class is used to prioritize nodes in the PriorityQueue based on distance.
            this.index = index;
            this.distance = distance;
        }
    }
}
