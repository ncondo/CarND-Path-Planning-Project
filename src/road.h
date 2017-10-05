#ifndef ROAD_H
#define ROAD_H

#include <vector>

#include "spline.h"


struct Frenet {
  double s;
  double d;
};

struct Cartesian {
  double x;
  double y;
};

class Road {
public:
  static double MAX_S;
  static double LANE_WIDTH;
  static int NUM_LANES;

  static void add_waypoints(double x, double y, double s, double d_x, double d_y);
  static void init();

  static Frenet getFrenet(double x, double y, double theta);
  static Cartesian getXY(double s, double d);

private:
  static tk::spline s_x, s_y, s_dx, s_dy;
  static std::vector<double> map_waypoints_x;
  static std::vector<double> map_waypoints_y;
  static std::vector<double> map_waypoints_s;
  static std::vector<double> map_waypoints_dx;
  static std::vector<double> map_waypoints_dy;

  static int NextWaypoint(double x, double y, double theta);
  static int ClosestWaypoint(double x, double y);
  static double distance(double x1, double y1, double x2, double y2);
  static int cyclic_index(int i);
};

#endif
