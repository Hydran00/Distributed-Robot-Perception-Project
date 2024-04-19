#include <project/utils.h>
// 1) integrate voro++ so that we can obtain vertices of the polyhedrons
// starting from robot pose 2) publish control input to the robot
using namespace std;
using namespace Eigen;

// Define multivariate Gaussian PDF parameters
VectorXd mean(3);           // Mean vector
MatrixXd covariance(3, 3);  // Covariance matrix


int main(int argc, char* argv[]) {
  assert(argc == 3 + 1);
  char* end;
  Vector3d p;
  p.x() = std::strtod(argv[1], &end);
  p.y() = std::strtod(argv[2], &end);
  p.z() = std::strtod(argv[3], &end);

  std::vector<Face> cube{
      Face{{Vector3d(0, 0, 0), Vector3d(0.999999, 0, 0), Vector3d(1, 0, 1), Vector3d(0, 0, 1)}}, // front
      Face{{Vector3d(0, 1, 0), Vector3d(0, 1, 1), Vector3d(1, 1, 1), Vector3d(1, 1, 0)}}, // back
      Face{{Vector3d(0, 0, 0), Vector3d(0, 0, 1), Vector3d(0, 1, 1), Vector3d(0, 1, 0)}}, // left
      Face{{Vector3d(1, 0, 0), Vector3d(1, 1, 0), Vector3d(1, 1, 1), Vector3d(1, 0, 1)}}, // right
      Face{{Vector3d(0, 0, 1), Vector3d(1, 0, 1), Vector3d(1, 1, 1), Vector3d(0, 1, 1)}}, // top
      Face{{Vector3d(0, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 1, 0), Vector3d(1, 0, 0)}}, // bottom
  };

  std::cout << (isInConvexPoly(p, cube) ? "inside" : "outside") << std::endl;

  return 0;
}
