#include "planning.h"
#include <queue>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>
#include <cmath>

using namespace std;

// Directions: up, down, left, right
const vector<pair<int, int>> directions = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// Constructor
Planner::Planner(const vector<vector<bool>> &grid) : grid(grid)
{
  rows = grid.size();
  cols = grid[0].size();
}

// Check if cell is within bounds and not blocked
bool Planner::isvalid(int x, int y) const
{
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

// Heuristic for A* (not used in BFS, but can be handy later)
double Planner::heuristic(int x1, int y1, int x2, int y2) const
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// BFS-based path planning
vector<pair<int, int>> Planner::pathplanning(pair<int, int> start, pair<int, int> goal)
{
  vector<pair<int, int>> path;
  vector<vector<bool>> visited(rows, vector<bool>(cols, false));
  map<pair<int, int>, pair<int, int>> parent;

  queue<pair<int, int>> q;
  q.push(start);
  visited[start.first][start.second] = true;

  bool found = false;

  while (!q.empty())
  {
    auto current = q.front();
    q.pop();

    if (current == goal)
    {
      found = true;
      break;
    }

    for (auto dir : directions)
    {
      int nr = current.first + dir.first;
      int nc = current.second + dir.second;

      if (isvalid(nr, nc) && !visited[nr][nc])
      {
        visited[nr][nc] = true;
        parent[{nr, nc}] = current;
        q.push({nr, nc});
      }
    }
  }

  if (!found)
    return path; // empty if no path

  // Reconstruct path from goal -> start
  pair<int, int> curr = goal;
  while (curr != start)
  {
    path.push_back(curr);
    curr = parent[curr];
  }
  path.push_back(start);

  reverse(path.begin(), path.end()); // start -> goal
  return path;
}
