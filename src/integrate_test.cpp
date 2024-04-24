#include "voro++.hh"
#include <unistd.h> 
using namespace voro;

const double pi = 3.1415926535897932384626433832795;

int main() {
  int i = 0;
  double x, y, z, evol, vvol;

  // Create a container with the geometry given above, and make it
  // non-periodic in each of the three coordinates. Allocate space for
  // eight particles within each computational block.
  container con(-1.2, 1.2, -1.2, 1.2, -1.1, 1, 14, 7, 7, false, false, false,
                8);

  // Add a cylindrical wall to the container
  // wall_cone cone(0, 0, 2, 0, 0, -1, atan(0.5));
  // con.add_wall(cone);
  // wall_sphere sphere(0, 0, 0, 1.02);
  // wall_sphere sphere1(0, 0, 0, 1.0);
  int step = 0;
  double step_size = 0.01;
  while (true) {
    con.clear();

    wall_sphere sphere2(0, 0, 0, 0.9);

    // con.add_wall(sphere);
    // con.add_wall(sphere1);
    con.add_wall(sphere2);

    // Place particles in a regular grid within the frustum, for points
    // which are within the wall boundaries

    // load particles from file
    con.import("sphere_points.dat");

    double x1, y1, z1, x2, y2, z2;
    x1 = 0, y1 = 0, z1 = 0.5;
    x2 = 0, y2 = 0, z2 = -0.5;

    z1 = z1 - step * step_size;
    z2 = z2 + step * step_size;
    y1 = y1 + sin(step * step_size);
    y2 = y2 - sin(step * step_size);
    if(z1 < -1) {
      break;
    }


    con.put(104, x1, y1, z1);
    con.put(105, x2, y2, z2);

    c_loop_all cla(con);
    voronoicell c;
    // compute cell
    double r;
    // con.draw_particles("PART.gnu");
    FILE *f1 = safe_fopen("PART.gnu", "w");
    // manually store particles
    fprintf(f1, "%d %g %g %g\n", 104, x1, y1, z1);
    fprintf(f1, "%d %g %g %g\n", 105, x2, y2, z2);
    fclose(f1);
    FILE *f2 = safe_fopen("WALL.gnu", "w");
    if (cla.start()) do
        if (con.compute_cell(c, cla)) {
          // Get the position and ID information for the particle
          // currently being considered by the loop. Ignore the radius
          // information.
          cla.pos(i, x, y, z, r);

          // apply wall
          // sphere.cut_cell(c, x, y, z);
          // Draw the Voronoi cell
          c.draw_gnuplot(x, y, z, f2);
        }
      while (cla.inc());
    // con.draw_cells_gnuplot("frustum_v.gnu");
    sleep(1.0);
    step++;
  }

  evol = pi * 1 * (0.5 * 0.5 + 0.5 * 1 + 1 * 1) / 3;
  vvol = con.sum_cell_volumes();
  printf(
      "Exact frustum volume : %g\n"
      "Voronoi cell volume  : %g\n"
      "Difference           : %g\n",
      evol, vvol, vvol - evol);
}
