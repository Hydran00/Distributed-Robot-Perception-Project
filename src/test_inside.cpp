#include <iostream>
#include "project/utils.h"

using Vector = Eigen::Vector3d;
using Point = Vector;


int main(int argc, char* argv[]) {
  assert(argc == 3 + 1);
  char* end;
  Point p;
  p.x() = std::strtod(argv[1], &end);
  p.y() = std::strtod(argv[2], &end);
  p.z() = std::strtod(argv[3], &end);

  std::vector<Face> cube{
      Face{{Point(0, 0, 0), Point(0.999999, 0, 0), Point(1, 0, 1), Point(0, 0, 1)}}, // front
      Face{{Point(0, 1, 0), Point(0, 1, 1), Point(1, 1, 1), Point(1, 1, 0)}}, // back
      Face{{Point(0, 0, 0), Point(0, 0, 1), Point(0, 1, 1), Point(0, 1, 0)}}, // left
      Face{{Point(1, 0, 0), Point(1, 1, 0), Point(1, 1, 1), Point(1, 0, 1)}}, // right
      Face{{Point(0, 0, 1), Point(1, 0, 1), Point(1, 1, 1), Point(0, 1, 1)}}, // top
      Face{{Point(0, 0, 0), Point(0, 1, 0), Point(1, 1, 0), Point(1, 0, 0)}}, // bottom
  };

  std::cout << (isInConvexPoly(p, cube) ? "inside" : "outside") << std::endl;

  return 0;
}
