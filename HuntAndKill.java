package com.example.mazegameproject1;

import javafx.application.Application;
import javafx.geometry.*;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.layout.*;
import javafx.scene.text.Font;
import javafx.stage.Stage;
import javafx.scene.input.KeyCode;
import java.util.*;

public class HuntAndKill extends Application {

    // Game configuration constants
    private static final int HEIGHT = 77; // Must be odd for maze generation
    private static final int WIDTH = 77;  // Must be odd for maze generation

    // UI components
    private GridPane gridPane;
    private Label[][] cellLabels; // Labels for each cell

    // Maze and entities
    private Maze maze;
    private Player player;
    private List<Enemy> enemies; // List to hold all enemies
    private Walker walker; // Walker enemy

    // Player move counter
    private int playerMoveCount = 0; // Tracks the number of player moves

    // Shortest path from (1,1) to goal
    private List<Cell> shortestPath;
    private int bfsStepsExplored; // Total steps BFS took

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) {
        // Initialize the maze and entities
        maze = new Maze(WIDTH, HEIGHT);
        maze.generateMaze();

        player = new Player(maze.getStartCell());
        cellLabels = new Label[HEIGHT][WIDTH];

        // Initialize enemies
        enemies = new ArrayList<>();

        // Initialize the Walker but do not add it to the enemies list yet
        walker = new Walker(maze, player);

        // Create the main layout
        BorderPane root = new BorderPane();

        // Create the game grid pane
        gridPane = new GridPane();
        gridPane.setAlignment(Pos.CENTER);
        gridPane.setHgap(0);
        gridPane.setVgap(0);

        // Initialize the cell labels
        initializeGrid();

        // Find and mark the shortest path
        PathResult pathResult = maze.findShortestPath();
        if (pathResult.path != null) {
            shortestPath = pathResult.path;
            bfsStepsExplored = pathResult.stepsExplored;
            markShortestPath(shortestPath);
            printPathToConsole(shortestPath, bfsStepsExplored);
        } else {
            System.out.println("No path found from start to goal.");
        }

        // Update the grid to reflect initial positions
        updateGrid();

        // Add the grid pane to the center of the BorderPane
        ScrollPane scrollPane = new ScrollPane(gridPane);
        scrollPane.setFitToWidth(true);
        scrollPane.setFitToHeight(true);
        root.setCenter(scrollPane);

        // Create the button panel
        HBox buttonPanel = new HBox(10);
        buttonPanel.setAlignment(Pos.CENTER);
        buttonPanel.setPadding(new Insets(10));

        // Create the movement buttons
        Button leftButton = new Button(" <- ");
        Button upButton = new Button(" /\\ ");
        Button downButton = new Button(" \\/ ");
        Button rightButton = new Button(" -> ");
        Button newMazeButton = new Button("Generate New Maze");

        int buttonW = 75;
        int buttonH = 40;

        leftButton.setPrefSize(buttonW, buttonH);
        upButton.setPrefSize(buttonW, buttonH);
        downButton.setPrefSize(buttonW, buttonH);
        rightButton.setPrefSize(buttonW, buttonH);
        newMazeButton.setPrefSize(150, buttonH);

        // Add buttons to the panel
        buttonPanel.getChildren().addAll(leftButton, upButton, downButton, rightButton, newMazeButton);

        // Add the button panel to the bottom of the BorderPane
        root.setBottom(buttonPanel);

        // Set up the scene and stage
        Scene scene = new Scene(root);
        primaryStage.setTitle("The Coffee Maze Game");
        primaryStage.setScene(scene);
        primaryStage.setMaximized(true);
        primaryStage.show();

        // Add action handlers for the buttons
        rightButton.setOnAction(e -> movePlayer(Direction.RIGHT));
        leftButton.setOnAction(e -> movePlayer(Direction.LEFT));
        upButton.setOnAction(e -> movePlayer(Direction.UP));
        downButton.setOnAction(e -> movePlayer(Direction.DOWN));
        newMazeButton.setOnAction(e -> restartGame());

        // Add key event handler for keyboard controls
        scene.setOnKeyPressed(event -> {
            KeyCode code = event.getCode();
            if (code == KeyCode.W || code == KeyCode.UP) {
                movePlayer(Direction.UP);
            } else if (code == KeyCode.S || code == KeyCode.DOWN) {
                movePlayer(Direction.DOWN);
            } else if (code == KeyCode.A || code == KeyCode.LEFT) {
                movePlayer(Direction.LEFT);
            } else if (code == KeyCode.D || code == KeyCode.RIGHT) {
                movePlayer(Direction.RIGHT);
            }
        });

        // Ensure focus is on the scene to receive key events
        root.requestFocus();
    }

    // Initialize the grid with labels
    private void initializeGrid() {
        gridPane.getChildren().clear(); // Clear existing cells
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                Label cellLabel = new Label();
                cellLabel.setPrefSize(15, 15); // Adjust cell size as needed
                cellLabel.setMaxSize(15, 15);
                cellLabel.setMinSize(15, 15);
                cellLabel.setAlignment(Pos.CENTER);
                cellLabel.setFont(Font.font("Monospaced", 10));
                cellLabels[y][x] = cellLabel;
                gridPane.add(cellLabel, x, y);
            }
        }
    }

    // Update the grid based on the maze and entities
    private void updateGrid() {
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                Label cellLabel = cellLabels[y][x];
                char cellValue = maze.getSymbolAt(x, y);

                if (x == player.getX() && y == player.getY()) {
                    // Player's position
                    cellLabel.setText(String.valueOf(player.getSymbol()));
                    cellLabel.setStyle("-fx-background-color: lightblue; -fx-border-color: black;");
                } else if (maze.isGoal(x, y)) {
                    // Goal cell
                    cellLabel.setText(String.valueOf(maze.getGoalSymbol()));
                    cellLabel.setStyle("-fx-background-color: gold; -fx-border-color: black;");
                } else if (maze.isWall(x, y)) {
                    // Wall cells
                    cellLabel.setText(String.valueOf(maze.getWallSymbol()));
                    cellLabel.setStyle("-fx-background-color: black; -fx-text-fill: black; -fx-border-color: black;");
                } else if (isCellInPath(x, y)) {
                    // Cells in the shortest path
                    cellLabel.setText(" ");
                    cellLabel.setStyle("-fx-background-color: lightgreen; -fx-border-color: black;");
                } else {
                    // Open path cells
                    cellLabel.setText(" ");
                    cellLabel.setStyle("-fx-background-color: white; -fx-border-color: black;");
                }
            }
        }

        // Draw enemies
        for (Enemy enemy : enemies) {
            int x = enemy.getX();
            int y = enemy.getY();
            Label cellLabel = cellLabels[y][x];
            cellLabel.setText(String.valueOf(enemy.getSymbol()));
            cellLabel.setStyle("-fx-background-color: lightcoral; -fx-border-color: black;");
        }
    }

    // Check if a cell is part of the shortest path
    private boolean isCellInPath(int x, int y) {
        if (shortestPath == null) return false;
        for (Cell cell : shortestPath) {
            if (cell.x == x && cell.y == y) {
                return true;
            }
        }
        return false;
    }

    // Move the player in the specified direction
    private void movePlayer(Direction direction) {
        if (maze.isValidMove(player.getX() + direction.getDx(), player.getY() + direction.getDy())) {
            player.move(direction);
            playerMoveCount++; // Increment player move count
            updateGameState();
        }
    }

    // Update the game state and check for win/loss conditions
    private void updateGameState() {
        updateGrid();

        // Check if the Walker needs to be spawned after 5 moves
        if (playerMoveCount == 5 && !enemies.contains(walker)) {
            // Ensure player is not at the initial spawn point
            if (player.getX() != maze.getStartX() || player.getY() != maze.getStartY()) {
                // Spawn the Walker at the player's initial spawn point
                walker.relocate(maze.getStartX(), maze.getStartY());
                walker.updatePath(); // Initialize pathfinding
                enemies.add(walker);
                System.out.println("Walker spawned at: (" + walker.getX() + ", " + walker.getY() + ")");
            } else {
                // If player is still at the initial position, delay spawning
                System.out.println("Player is at the initial position; delaying Walker spawn.");
            }
        }

        // Move enemies
        for (Enemy enemy : enemies) {
            enemy.move();
        }

        // After all enemies have moved, check for lose condition
        for (Enemy enemy : enemies) {
            if (enemy.getX() == player.getX() && enemy.getY() == player.getY()) {
                lose();
                return;
            }
        }

        updateGrid(); // Update grid after enemies move

        if (maze.isGoal(player.getX(), player.getY())) {
            win();
        }
    }

    // Handle the lose condition
    private void lose() {
        Alert alert = new Alert(Alert.AlertType.WARNING);
        alert.setTitle("GAME OVER");
        alert.setHeaderText(null);
        alert.setContentText("The Walker caught you! Try again?");
        ButtonType retryButton = new ButtonType("Retry");
        ButtonType exitButton = new ButtonType("Exit");
        alert.getButtonTypes().setAll(retryButton, exitButton);

        alert.showAndWait().ifPresent(type -> {
            if (type == retryButton) {
                restartGame();
            } else {
                System.exit(0);
            }
        });
    }

    // Display win message and ask to restart or exit
    private void win() {
        Alert alert = new Alert(Alert.AlertType.CONFIRMATION);
        alert.setTitle("YOU WIN!");
        alert.setHeaderText(null);
        alert.setContentText("You found the coffee!\nDo you want to play again?");
        ButtonType yesButton = new ButtonType("Yes");
        ButtonType noButton = new ButtonType("No");
        alert.getButtonTypes().setAll(yesButton, noButton);

        alert.showAndWait().ifPresent(type -> {
            if (type == yesButton) {
                restartGame();
            } else {
                System.exit(0);
            }
        });
    }

    // Restart the game
    private void restartGame() {
        // Regenerate the maze
        maze.generateMaze();

        // Reset player position
        player.setPosition(maze.getStartCell());

        // Reset player move count
        playerMoveCount = 0;

        // Reset enemies
        enemies.clear();

        // Reinitialize the Walker but do not add it to the enemies list yet
        walker = new Walker(maze, player);

        // Find and mark the shortest path
        PathResult pathResult = maze.findShortestPath();
        if (pathResult.path != null) {
            shortestPath = pathResult.path;
            bfsStepsExplored = pathResult.stepsExplored;
            markShortestPath(shortestPath);
            printPathToConsole(shortestPath, bfsStepsExplored);
        } else {
            System.out.println("No path found from start to goal.");
        }

        updateGrid();
    }

    // Mark the shortest path in the grid
    private void markShortestPath(List<Cell> path) {
        this.shortestPath = path;
    }

    // Print the shortest path and step counts to the console
    private void printPathToConsole(List<Cell> path, int stepsExplored) {
        System.out.println("Shortest path from (" + maze.getStartX() + ", " + maze.getStartY() + ") to ("
                + maze.getGoalX() + ", " + maze.getGoalY() + "):");
        int stepNumber = 1;
        for (Cell cell : path) {
            System.out.print("Step " + stepNumber + ": (" + cell.x + "," + cell.y + ") ");
            stepNumber++;
        }
        System.out.println(); // Newline after printing the path
        System.out.println("Total steps in the shortest path: " + (path.size() - 1));
        System.out.println("Total BFS exploration steps: " + stepsExplored);
    }

    // Enum for player movement directions
    enum Direction {
        UP(0, -1), DOWN(0, 1), LEFT(-1, 0), RIGHT(1, 0);

        private final int dx;
        private final int dy;

        Direction(int dx, int dy) {
            this.dx = dx;
            this.dy = dy;
        }

        public int getDx() { return dx; }
        public int getDy() { return dy; }
    }

    // Class to store BFS path results
    class PathResult {
        List<Cell> path;
        int stepsExplored;

        PathResult(List<Cell> path, int stepsExplored) {
            this.path = path;
            this.stepsExplored = stepsExplored;
        }
    }

    // Maze class using the Hunt and Kill algorithm
    class Maze {
        private final int width;
        private final int height;
        private final char[][] grid;

        private final char wall = '█';
        private final char path = ' ';
        private final char goalSymbol = '⛾';
        private final int startX = 1;
        private final int startY = 1;
        private int goalX;
        private int goalY;

        public Maze(int width, int height) {
            this.width = width;
            this.height = height;
            grid = new char[height][width];
        }

        // Generate maze using Hunt and Kill algorithm
        public void generateMaze() {
            // Initialize maze with walls
            for (int y = 0; y < height; y++) {
                Arrays.fill(grid[y], wall);
            }

            Random rand = new Random();

            int currentX = startX;
            int currentY = startY;

            grid[currentY][currentX] = path;

            while (true) {
                // Random Walk
                List<int[]> neighbors = getUnvisitedNeighbors(currentX, currentY);

                if (!neighbors.isEmpty()) {
                    int[] nextCell = neighbors.get(rand.nextInt(neighbors.size()));
                    int nx = nextCell[0];
                    int ny = nextCell[1];

                    // Remove wall between current cell and next cell
                    int wallX = currentX + (nx - currentX) / 2;
                    int wallY = currentY + (ny - currentY) / 2;
                    grid[wallY][wallX] = path;
                    grid[ny][nx] = path;

                    currentX = nx;
                    currentY = ny;
                } else {
                    // Hunt for unvisited cell
                    boolean found = false;
                    for (int y = 1; y < height; y += 2) {
                        for (int x = 1; x < width; x += 2) {
                            if (grid[y][x] == wall && hasVisitedNeighbor(x, y)) {
                                grid[y][x] = path;

                                // Connect to a random visited neighbor
                                List<int[]> visitedNeighbors = getVisitedNeighbors(x, y);
                                int[] neighbor = visitedNeighbors.get(rand.nextInt(visitedNeighbors.size()));
                                int nx = neighbor[0];
                                int ny = neighbor[1];

                                // Remove wall between cells
                                int wallXBetween = x + (nx - x) / 2;
                                int wallYBetween = y + (ny - y) / 2;
                                grid[wallYBetween][wallXBetween] = path;

                                currentX = x;
                                currentY = y;
                                found = true;
                                break;
                            }
                        }
                        if (found) {
                            break;
                        }
                    }
                    if (!found) {
                        break; // Maze generation complete
                    }
                }
            }

            // Place the goal at the bottom-right corner
            goalX = width - 2;
            goalY = height - 2;
            grid[goalY][goalX] = goalSymbol;
        }

        // Get unvisited neighbors (2 cells away)
        private List<int[]> getUnvisitedNeighbors(int x, int y) {
            List<int[]> neighbors = new ArrayList<>();

            int[][] directions = {
                    {0, -2}, // Up
                    {0, 2},  // Down
                    {-2, 0}, // Left
                    {2, 0}   // Right
            };

            for (int[] dir : directions) {
                int nx = x + dir[0];
                int ny = y + dir[1];

                if (isInBounds(nx, ny) && grid[ny][nx] == wall) {
                    neighbors.add(new int[]{nx, ny});
                }
            }
            return neighbors;
        }

        // Check if the cell has any visited neighbors
        private boolean hasVisitedNeighbor(int x, int y) {
            int[][] directions = {
                    {0, -2}, // Up
                    {0, 2},  // Down
                    {-2, 0}, // Left
                    {2, 0}   // Right
            };

            for (int[] dir : directions) {
                int nx = x + dir[0];
                int ny = y + dir[1];

                if (isInBounds(nx, ny) && grid[ny][nx] == path) {
                    return true;
                }
            }
            return false;
        }

        // Get visited neighbors (2 cells away)
        private List<int[]> getVisitedNeighbors(int x, int y) {
            List<int[]> neighbors = new ArrayList<>();

            int[][] directions = {
                    {0, -2}, // Up
                    {0, 2},  // Down
                    {-2, 0}, // Left
                    {2, 0}   // Right
            };

            for (int[] dir : directions) {
                int nx = x + dir[0];
                int ny = y + dir[1];

                if (isInBounds(nx, ny) && grid[ny][nx] == path) {
                    neighbors.add(new int[]{nx, ny});
                }
            }
            return neighbors;
        }

        public boolean isInBounds(int x, int y) {
            return x >= 0 && x < width && y >= 0 && y < height;
        }

        public boolean isValidMove(int x, int y) {
            return isInBounds(x, y) && grid[y][x] != wall;
        }

        public boolean isWall(int x, int y) {
            return grid[y][x] == wall;
        }

        public boolean isGoal(int x, int y) {
            return x == goalX && y == goalY;
        }

        public char getSymbolAt(int x, int y) {
            return grid[y][x];
        }


        public char getWallSymbol() {
            return wall;
        }

        public char getPathSymbol() {
            return path;
        }

        public char getGoalSymbol() {
            return goalSymbol;
        }

        public int getStartX() {
            return startX;
        }

        public int getStartY() {
            return startY;
        }

        public int getGoalX() {
            return goalX;
        }

        public int getGoalY() {
            return goalY;
        }

        // Method to get the start cell
        public Cell getStartCell() {
            return new Cell(startX, startY);
        }

        // Method to get the goal cell
        public Cell getGoalCell() {
            return new Cell(goalX, goalY);
        }

        // Method to get random open positions in the maze
        public List<Cell> getOpenCells() {
            List<Cell> openCells = new ArrayList<>();
            for (int y = 1; y < height - 1; y++) {
                for (int x = 1; x < width - 1; x++) {
                    if (grid[y][x] == path) {
                        openCells.add(new Cell(x, y));
                    }
                }
            }
            return openCells;
        }

        // **BFS Implementation to Find the Shortest Path with Step Counting**
        public PathResult findShortestPath() {
            boolean[][] visited = new boolean[height][width];
            Queue<Cell> queue = new LinkedList<>();
            Cell start = getStartCell();
            queue.add(start);
            visited[start.y][start.x] = true;

            // To reconstruct the path
            Map<String, Cell> parentMap = new HashMap<>();

            int stepsExplored = 0; // Counter for BFS exploration steps

            while (!queue.isEmpty()) {
                Cell current = queue.poll();
                stepsExplored++; // Increment steps explored

                if (current.x == goalX && current.y == goalY) {
                    // Reached the goal, reconstruct the path
                    List<Cell> path = reconstructPath(current, parentMap);
                    return new PathResult(path, stepsExplored);
                }

                // Explore neighbors
                int[][] directions = {
                        {0, -1}, // Up
                        {0, 1},  // Down
                        {-1, 0}, // Left
                        {1, 0}   // Right
                };

                for (int[] dir : directions) {
                    int nx = current.x + dir[0];
                    int ny = current.y + dir[1];

                    if (isInBounds(nx, ny) && !visited[ny][nx] && grid[ny][nx] != wall) {
                        visited[ny][nx] = true;
                        Cell neighbor = new Cell(nx, ny);
                        queue.add(neighbor);
                        parentMap.put(nx + "," + ny, current);
                    }
                }
            }

            // No path found
            return new PathResult(null, stepsExplored);
        }

        // Reconstruct the path from goal to start
        private List<Cell> reconstructPath(Cell goal, Map<String, Cell> parentMap) {
            List<Cell> path = new ArrayList<>();
            Cell current = goal;

            while (current != null) {
                path.add(current);
                String key = current.x + "," + current.y;
                current = parentMap.get(key);
            }

            Collections.reverse(path); // Reverse to get path from start to goal
            return path;
        }
    }

    // Cell class representing a cell in the maze
    class Cell {
        final int x, y;
        Cell parent; // For pathfinding

        Cell(int x, int y) {
            this.x = x;
            this.y = y;
        }

        Cell(int x, int y, Cell parent) {
            this.x = x;
            this.y = y;
            this.parent = parent;
        }
    }

    // Player class representing the player
    class Player {
        private int x, y;
        private final char symbol = '▣';

        public Player(Cell startCell) {
            this.x = startCell.x;
            this.y = startCell.y;
        }

        // Encapsulate movement logic
        public void move(Direction direction) {
            this.x += direction.getDx();
            this.y += direction.getDy();
            // Additional logic (e.g., boundary checks) can be included here
        }

        public char getSymbol() {
            return symbol;
        }

        // Provide getters only if necessary
        public int getX() { return x; }
        public int getY() { return y; }

        public void setPosition(Cell cell) {
            this.x = cell.x;
            this.y = cell.y;
        }
    }

    // Abstract Enemy class
    abstract class Enemy {
        protected int x, y;
        protected final char symbol;
        protected final Maze maze;
        protected final Player player;

        public Enemy(Maze maze, Player player, char symbol) {
            this.maze = maze;
            this.player = player;
            this.symbol = symbol;
        }

        public abstract void move();

        // Encapsulate relocation behavior
        public void relocate(int newX, int newY) {
            if (maze.isValidMove(newX, newY)) {
                this.x = newX;
                this.y = newY;
            }
        }

        public char getSymbol() {
            return symbol;
        }

        public int getX() { return x; }
        public int getY() { return y; }

        public abstract void reset();
    }

    // Walker class implementing A* pathfinding
    class Walker extends Enemy {

        private List<Cell> walkerPath;

        public Walker(Maze maze, Player player) {
            super(maze, player, 'W');
            walkerPath = new ArrayList<>(); // Initialize walkerPath
        }

        @Override
        public void move() {
            updatePath();
            moveAlongPath();
        }

        @Override
        public void reset() {
            walkerPath.clear(); // Clear previous path
            updatePath(); // Update path to the player's current position
        }

        // Update the path using A* algorithm
        public void updatePath() {
            walkerPath = findAStarPath(x, y, player.getX(), player.getY());
        }

        // Move along the path by one step
        private void moveAlongPath() {
            if (walkerPath != null && !walkerPath.isEmpty()) {
                Cell nextStep = walkerPath.remove(0);
                // Allow the Walker to move into the player's tile
                this.x = nextStep.x;
                this.y = nextStep.y;
            }
        }

        // A* Pathfinding Algorithm
        private List<Cell> findAStarPath(int startX, int startY, int goalX, int goalY) {
            PriorityQueue<AStarNode> openSet = new PriorityQueue<>(Comparator.comparingDouble(n -> n.fCost));
            boolean[][] closedSet = new boolean[maze.height][maze.width];

            AStarNode startNode = new AStarNode(startX, startY, null, 0, heuristic(startX, startY, goalX, goalY));
            openSet.add(startNode);

            while (!openSet.isEmpty()) {
                AStarNode current = openSet.poll();

                if (current.x == goalX && current.y == goalY) {
                    return reconstructPath(current);
                }

                closedSet[current.y][current.x] = true;

                for (int[] dir : new int[][]{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}) {
                    int nx = current.x + dir[0];
                    int ny = current.y + dir[1];

                    if (!maze.isValidMove(nx, ny) || closedSet[ny][nx]) {
                        continue;
                    }

                    double tentativeGCost = current.gCost + 1;

                    AStarNode neighbor = new AStarNode(nx, ny, current, tentativeGCost, heuristic(nx, ny, goalX, goalY));

                    boolean skip = false;
                    for (AStarNode node : openSet) {
                        if (node.x == neighbor.x && node.y == neighbor.y && node.fCost <= neighbor.fCost) {
                            skip = true;
                            break;
                        }
                    }
                    if (!skip) {
                        openSet.add(neighbor);
                    }
                }
            }

            // No path found
            System.out.println("Walker: No path found to player.");
            return new ArrayList<>();
        }

        // Heuristic function (Manhattan distance)
        private double heuristic(int x, int y, int goalX, int goalY) {
            return Math.abs(x - goalX) + Math.abs(y - goalY);
        }

        // Reconstruct the path from goal to start
        private List<Cell> reconstructPath(AStarNode node) {
            List<Cell> path = new ArrayList<>();
            AStarNode current = node;
            while (current.parent != null) {
                path.add(new Cell(current.x, current.y));
                current = current.parent;
            }
            Collections.reverse(path);
            return path;
        }
    }

    // AStarNode class used in the Walker's pathfinding
    class AStarNode {
        int x, y;
        AStarNode parent;
        double gCost; // Cost from start to current node
        double hCost; // Heuristic cost to goal
        double fCost; // Total cost

        AStarNode(int x, int y, AStarNode parent, double gCost, double hCost) {
            this.x = x;
            this.y = y;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = this.gCost + this.hCost;
        }
    }
}
