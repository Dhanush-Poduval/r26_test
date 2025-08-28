#include "odometry.h"
#include <cmath>
#include <vector>
using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm)
{
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Odometry::angle(int x1, int y1, int x2, int y2)
{
  return atan2((y2 - y1), (x2 - x1)) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path)
{
  MotionCommand res = {0.0, 0.0};
  if (path.size() < 2)
    return res;

  double prev_angle = angle(path[0].first, path[0].second, path[1].first, path[1].second);

  for (size_t i = 1; i < path.size(); i++)
  {
    int x1 = path[i - 1].first, y1 = path[i - 1].second;
    int x2 = path[i].first, y2 = path[i].second;

    double dist = distance(x1, y1, x2, y2);
    res.time_sec += dist / linear_vel;

    double current_angle = angle(x1, y1, x2, y2);

    double delta = current_angle - prev_angle;
    delta = fmod(delta + 180.0, 360.0) - 180.0; // maps delta to [-180,180]
    res.angle_deg += fabs(delta);
  }

  return res;
}
