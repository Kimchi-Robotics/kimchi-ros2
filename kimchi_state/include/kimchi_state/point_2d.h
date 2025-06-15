#pragma once

struct Point2D {
  double x;
  double y;

  Point2D(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}

  bool operator==(const Point2D& other) const {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const Point2D& other) const { return !(*this == other); }
};
