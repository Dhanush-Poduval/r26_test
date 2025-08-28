#include "odometry.h"
#include <cmath>
#include <ctime>
#include <iterator>
#include <numeric>

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm)
{
  // Linear velocity (m/s) =(wheel circumference * revolutions per second)
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Odometry::angle(int x1, int y1, int x2, int y2)
{
  // atan2 returns radians, convert to degrees
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path)
{
  MotionCommand res = {0.0, 0.0};

  if (path.size() < 2)
    return res;

  double prev_angle = angle(path[0].first, path[0].second, path[1].first, path[1].second);

  for (size_t i = 1; i < path.size(); i++)
  {
    int x1 = path[i - 1].first;
    int y1 = path[i - 1].second;
    int x2 = path[i].first;
    int y2 = path[i].second;

    // 1️⃣ distance between consecutive points
    double dist = distance(x1, y1, x2, y2);
    res.time_sec += dist / linear_vel;

    // 2️⃣ angle for this segment
    double current_angle = angle(x1, y1, x2, y2);

    // 3️⃣ add to total angle only if direction changed
    double angle_change = fabs(current_angle - prev_angle);
    if (angle_change > 1e-6)
    { // small threshold to ignore tiny changes
      res.angle_deg += angle_change;
    }

    prev_angle = current_angle;
  }

  return res;
}