#include <math.h>

#include "road.h"


double Road::MAX_S = 6945.554;
double Road::LANE_WIDTH = 4.0;
int Road::NUM_LANES = 3;
std::vector<double> Road::map_waypoints_x = std::vector<double>();
std::vector<double> Road::map_waypoints_y = std::vector<double>();
std::vector<double> Road::map_waypoints_s = std::vector<double>();
std::vector<double> Road::map_waypoints_dx = std::vector<double>();
std::vector<double> Road::map_waypoints_dy = std::vector<double>();
tk::spline Road::s_x, Road::s_y, Road::s_dx, Road::s_dy;

void Road::add_waypoints(double x, double y, double s, double d_x, double d_y) {
  map_waypoints_x.push_back(x);
  map_waypoints_y.push_back(y);
  map_waypoints_s.push_back(s);
  map_waypoints_dx.push_back(d_x);
  map_waypoints_dy.push_back(d_y);
}

void Road::init() {
  map_waypoints_x.push_back(map_waypoints_x[0]);
  map_waypoints_y.push_back(map_waypoints_y[0]);
  map_waypoints_s.push_back(MAX_S);
  map_waypoints_dx.push_back(map_waypoints_dx[0]);
  map_waypoints_dy.push_back(map_waypoints_dy[0]);

  s_x.set_points(map_waypoints_s, map_waypoints_x);
  s_y.set_points(map_waypoints_s, map_waypoints_y);
  s_dx.set_points(map_waypoints_s, map_waypoints_dx);
  s_dy.set_points(map_waypoints_s, map_waypoints_dy);
}

double Road::distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

int Road::cyclic_index(int next_wp) {
  if (next_wp == 0) {
    return map_waypoints_x.size() - 2;
  }
  return next_wp - 1;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet Road::getFrenet(double x, double y, double theta) {
  Frenet frenet;
	int next_wp = NextWaypoint(x, y, theta);
  int prev_wp = cyclic_index(next_wp);

	double n_x = map_waypoints_x[next_wp] - map_waypoints_x[prev_wp];
	double n_y = map_waypoints_y[next_wp] - map_waypoints_y[prev_wp];
	double x_x = x - map_waypoints_x[prev_wp];
	double x_y = y - map_waypoints_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	frenet.d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000 - map_waypoints_x[prev_wp];
	double center_y = 2000 - map_waypoints_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet.d *= -1;
	}

	// calculate s value
	frenet.s = 0;
	for (int i = 0; i < prev_wp; i++) {
		frenet.s += distance(map_waypoints_x[i], map_waypoints_y[i],
                         map_waypoints_x[i+1], map_waypoints_y[i+1]);
	}

	frenet.s += distance(0, 0, proj_x, proj_y);

	return frenet;
}

Cartesian Road::getXY(double s, double d) {
  Cartesian coord;
  double path_x = s_x(s);
  double path_y = s_y(s);
  double dx = s_dx(s);
  double dy = s_dy(s);
  coord.x = path_x + d * dx;
  coord.y = path_y + d * dy;

  return coord;
}

double Road::deg2rad(double angle) {
  return angle * M_PI / 180;
}

double Road::rad2deg(double angle) {
  return angle * 180 / M_PI;
}

int Road::ClosestWaypoint(double x, double y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;
  for (int i = 0; i < map_waypoints_x.size()-1; i++) {
    double map_x = map_waypoints_x[i];
    double map_y = map_waypoints_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

int Road::NextWaypoint(double x, double y, double theta) {
  int closestWaypoint = ClosestWaypoint(x, y);

  double dx = map_waypoints_dx[closestWaypoint];
  double dy = map_waypoints_dy[closestWaypoint];

  double heading = atan2(dy, dx) + M_PI / 2;
  double angle = abs(theta - heading);
  if (angle > M_PI / 4) {
    closestWaypoint++;
    if (closestWaypoint == map_waypoints_dx.size()-1) {
      closestWaypoint = 0;
    }
  }
  return closestWaypoint;
}
