#include "planning.h"
#include <queue>
#include <vector>
#include <map>
#include <utility>
#include <cmath>
#include <algorithm>

using namespace std;

// Directions for 8-connected movement (diagonals + axis-aligned)
const vector<pair<int, int>> directions = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

// Constructor
Planner::Planner(const vector<vector<bool>> &g) : grid(g)
{
  rows = grid.size();
  cols = grid[0].size();
}

// Check if a cell is inside grid and free
bool Planner::isvalid(int x, int y) const
{
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

// Heuristic: Euclidean distance
double Planner::heuristic(int x1, int y1, int x2, int y2) const
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// Path planning with A* algorithm
vector<pair<int, int>> Planner::pathplanning(pair<int, int> start, pair<int, int> goal)
{
  struct Node
  {
    int x, y;
    double f; // f = g + h
    bool operator>(const Node &other) const { return f > other.f; }
  };

  map<pair<int, int>, pair<int, int>> parent;
  vector<vector<double>> gscore(rows, vector<double>(cols, 1e9));
  priority_queue<Node, vector<Node>, greater<Node>> open;

  gscore[start.first][start.second] = 0.0;
  open.push({start.first, start.second, heuristic(start.first, start.second, goal.first, goal.second)});

  bool found = false;

  while (!open.empty())
  {
    Node current = open.top();
    open.pop();

    if (make_pair(current.x, current.y) == goal)
    {
      found = true;
      break;
    }

    for (auto dir : directions)
    {
      int nx = current.x + dir.first;
      int ny = current.y + dir.second;

      if (!isvalid(nx, ny))
        continue;

      double move_cost = (dir.first != 0 && dir.second != 0) ? 1.4142 : 1.0; // diagonal cost = sqrt(2)
      double tentative_g = gscore[current.x][current.y] + move_cost;

      if (tentative_g < gscore[nx][ny])
      {
        gscore[nx][ny] = tentative_g;
        parent[{nx, ny}] = {current.x, current.y};
        double f = tentative_g + heuristic(nx, ny, goal.first, goal.second);
        open.push({nx, ny, f});
      }
    }
  }

  vector<pair<int, int>> path;
  if (!found)
    return path;

  // Reconstruct path from goal to start
  pair<int, int> curr = goal;
  while (curr != start)
  {
    path.push_back(curr);
    curr = parent[curr];
  }
  path.push_back(start);
  reverse(path.begin(), path.end());

  return path;
}
