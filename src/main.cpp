#include "gridmap.h"
#include "odometry.h"
#include "planning.h"
#include "ublox_reader.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

// Helper to convert angle to unit direction
pair<double, double> directionFromAngle(double angle_deg)
{
  double rad = angle_deg * M_PI / 180.0;
  return {cos(rad), sin(rad)};
}

int main(int argc, char *argv[])
{
  if (argc < 3)
  {
    cerr << "Usage: " << argv[0] << " <gps_data_file> <odom_output_file>" << endl;
    return 1;
  }

  string gps_data = argv[1];
  string odom_commands = argv[2];

  // --- Task 1: Decode GPS Data ---
  auto result = readUbloxFile(gps_data);
  if (static_cast<int>(result.first.lat) == 0 && static_cast<int>(result.first.lon) == 0 &&
      static_cast<int>(result.second.lat) == 0 && static_cast<int>(result.second.lon) == 0)
  {
    cout << "Error: Invalid GPS Coordinates" << endl;
    return 1;
  }

  cout << "Start -> Lat: " << result.first.lat << " Lon: " << result.first.lon << endl;
  cout << "Goal  -> Lat: " << result.second.lat << " Lon: " << result.second.lon << endl;

  // --- Initialize Gridmapper ---
  GPS origin = {result.first.lat, result.first.lon};
  double cellsize = 1.0; // meters per grid cell
  int rows = 10, cols = 10;
  Gridmapper grid(origin, cellsize, rows, cols);

  pair<int, int> start = grid.gpstogrid(result.first);
  pair<int, int> goal = grid.gpstogrid(result.second);

  cout << "Start (grid) -> (" << start.first << "," << start.second << ")" << endl;
  cout << "Goal  (grid) -> (" << goal.first << "," << goal.second << ")" << endl;

  // --- Task 2: Path Planning ---
  Planner planner(grid.getGrid());
  auto path = planner.pathplanning(start, goal);

  cout << "Planned Path:" << endl;
  for (auto &p : path)
  {
    cout << "(" << p.first << "," << p.second << ") ";
  }
  cout << endl;

  // --- Task 3: Odometry Commands ---
  double wheel_radius = 0.05; // meters
  double rpm = 120;           // wheel speed
  Odometry odo(wheel_radius, rpm);
  auto commands = odo.computeCommands(path);

  // Print odometry commands to terminal
  cout << "\nOdometry Commands:" << endl;
  cout << "Total Time (s): " << commands.time_sec << endl;
  cout << "Total Angle (deg): " << commands.angle_deg << endl;

  // Write odometry commands to file
  ofstream result_file(odom_commands);
  if (!result_file.is_open())
  {
    cerr << "Error: cannot open file " << odom_commands << endl;
    return 1;
  }
  result_file << commands.time_sec << endl
              << commands.angle_deg << endl;
  result_file.close();

  return 0;
}
