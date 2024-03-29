
#include <LinkedList.h>

struct Cell {
  bool walls[4]; // Array to represent walls: 0 -> right, 1 -> left, 2 -> up, 3 -> down
  int x; // X coordinate of the cell
  int y; // Y coordinate of the cell
  int weight; // weight of cell
};

Cell maze[8][8] = {
  {
    {{true, true, false, true}, 0, 0, 0}, 
    {{false, false, false, true}, 1, 0, 0}, 
    {{false, false, false, true}, 2, 0, 0}, 
    {{false, false, false, true}, 3, 0, 0}, 
    {{false, false, false, true}, 4, 0, 0}, 
    {{false, false, false, true}, 5, 0, 0}, 
    {{false, false, false, true}, 6, 0, 0}, 
    {{true, true, true, true}, 7, 0, 0}
  },

  {
    {{true, false, false, false}, 0, 1, 0}, 
    {{false, false, false, false}, 1, 1, 0}, 
    {{false, false, false, false}, 2, 1, 0}, 
    {{false, false, false, false}, 3, 1, 0}, 
    {{false, false, false, false}, 4, 1, 0}, 
    {{false, false, false, false}, 5, 1, 0}, 
    {{false, false, false, false}, 6, 1, 0}, 
    {{false, true, false, false}, 7, 1, 0}
  },

  {
    {{true, false, false, false}, 0, 2, 0}, 
    {{false, false, false, false}, 1, 2, 0}, 
    {{false, false, false, false}, 2, 2, 0}, 
    {{false, false, false, false}, 3, 2, 0}, 
    {{false, false, false, false}, 4, 2, 0}, 
    {{false, false, false, false}, 5, 2, 0}, 
    {{false, false, false, false}, 6, 2, 0}, 
    {{false, true, false, false}, 7, 2, 0}
  },

  {
    {{true, false, false, false}, 0, 3, 0}, 
    {{false, false, false, false}, 1, 3, 0}, 
    {{false, false, false, false}, 2, 3, 0}, 
    {{false, false, false, false}, 3, 3, 0}, 
    {{false, false, false, false}, 4, 3, 0}, 
    {{false, false, false, false}, 5, 3, 0}, 
    {{false, false, false, false}, 6, 3, 0}, 
    {{false, true, false, false}, 7, 3, 0}
  },

  {
    {{true, false, false, false}, 0, 4, 0}, 
    {{false, false, false, false}, 1, 4, 0}, 
    {{false, false, false, false}, 2, 4, 0}, 
    {{false, false, false, false}, 3, 4, 0}, 
    {{false, false, false, false}, 4, 4, 0}, 
    {{false, false, false, false}, 5, 4, 0}, 
    {{false, false, false, false}, 6, 4, 0}, 
    {{false, true, false, false}, 7, 4, 0}
  },

  {
    {{true, false, false, false}, 0, 5, 0}, 
    {{false, false, false, false}, 1, 5, 0}, 
    {{false, false, false, false}, 2, 5, 0}, 
    {{false, false, false, false}, 3, 5, 0}, 
    {{false, false, false, false}, 4, 5, 0}, 
    {{false, false, false, false}, 5, 5, 0}, 
    {{false, false, false, false}, 6, 5, 0}, 
    {{false, true, false, false}, 7, 5, 0}
  },

  {
    {{true, false, false, false}, 0, 6, 0}, 
    {{false, false, false, false}, 1, 6, 0}, 
    {{false, false, false, false}, 2, 6, 0}, 
    {{false, false, false, false}, 3, 6, 0}, 
    {{false, false, false, false}, 4, 6, 0}, 
    {{false, false, false, false}, 5, 6, 0}, 
    {{false, false, false, false}, 6, 6, 0}, 
    {{false, true, false, false}, 7, 6, 0}
  },

  {
    {{true, false, true, false}, 0, 7, 0}, 
    {{false, false, true, false}, 1, 7, 0}, 
    {{false, false, true, false}, 2, 7, 0}, 
    {{false, false, true, false}, 3, 7, 0}, 
    {{false, false, true, false}, 4, 7, 0}, 
    {{false, false, true, false}, 5, 7, 0}, 
    {{false, false, true, false}, 6, 7, 0}, 
    {{false, true, true, false}, 7, 7, 0}
  }
};

void printMaze(const struct Cell (&maze)[8][8]) {
  for(int y = 0; y < 8; ++y){ // for every row in the grid
    Serial.print("\n");
    for(int x = 0; x < 8; ++x){ // for every cell in the row
        if(maze[x][y].walls[3]){
          Serial.print("|");
        }
        
        Serial.print(maze[x][y].x);
        if(maze[x][y].walls[0]){
          Serial.print("¯");
        } else{
          Serial.print(",");
        }
        if(maze[x][y].walls[1]){
          Serial.print("_");
        } else{
          Serial.print(",");
        }
        Serial.print(maze[x][y].y);

        if(maze[x][y].walls[2]){
          Serial.print("|");
        }
        if (maze[x][y].x < 10) { 
          Serial.print("  "); 
        } else {
          Serial.print(" "); 
        }
    }
  }
}

void printMazeWeights(const struct Cell (&maze)[8][8]) {
  for(int y = 0; y < 8; ++y){ // for every row in the grid
    Serial.print("\n");
    for(int x = 0; x < 8; ++x){ // for every cell in the row
        if(maze[x][y].walls[3]){
          Serial.print("|");
        }
        if(maze[x][y].weight > -1){
          Serial.print(maze[x][y].weight);
        } else{
          Serial.print("?");
        }
        
        if(maze[x][y].walls[0]){
          Serial.print("¯");
        } else{
          Serial.print(",");
        }
        if(maze[x][y].walls[1]){
          Serial.print("_");
        } else{
          Serial.print(",");
        }

        if(maze[x][y].walls[2]){
          Serial.print("|");
        }

        if (maze[x][y].x < 10) { 
          Serial.print("  "); 
        } else {
          Serial.print(" "); 
        }
    }
  }
}

void printMaze1(const struct Cell (&maze)[8][8]) {
  for(int y = 7; y >= 0; --y){ // Iterate from bottom to top for y-axis
    Serial.print("\n");
    for(int x = 0; x < 8; ++x){ // Iterate from left to right for x-axis
        if(maze[x][y].walls[2]){ // Check the top wall of the current cell
          Serial.print("---");
        } else {
          Serial.print("   ");
        }
    }
    Serial.println(); // Move to the next line for printing the next row of cells
    for(int x = 0; x < 8; ++x){ // Iterate over the cells in the current row
        if(maze[x][y].walls[1]){ // Check the left wall of the current cell
          Serial.print("|");
        } else {
          Serial.print(" ");
        }
        Serial.print("(");
        Serial.print(maze[x][y].x);
        Serial.print(",");
        Serial.print(maze[x][y].y);
        Serial.print(")");
    }
    // Print the right wall of the last cell in the row
    if(maze[7][y].walls[0]){
      Serial.print("|");
    } else {
      Serial.print(" ");
    }
  }
  Serial.println(); // Print a newline after printing the entire maze
}
