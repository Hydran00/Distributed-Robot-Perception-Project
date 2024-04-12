// Example code demonstrating find_voronoi_cell function
//
// Author   : Chris H. Rycroft (LBL / UC Berkeley)
// Email    : chr@alum.mit.edu
// Date     : August 30th 2011

#include <iostream>
#include<unistd.h>   
#include "voro++.hh"

using namespace std;
using namespace voro;

// The sampling distance for the grids of find_voronoi_cell calls
const double h = 0.05;

// The cube of the sampling distance, corresponding the amount of volume
// associated with a sample point
const double hcube = h * h * h;

// Set the number of particles that are going to be randomly introduced
const int particles = 2;

// This function returns a random double between 0 and 1
double rnd() { return double(rand()) / RAND_MAX; }

int sign(double x) { return (x > 0) - (x < 0); }

int main() {
  // cell id
  int i = 0, j = 1;
  double x, y, z;
  // starting position
  double x1_start = 0.8, y1_start = 0.8, z1_start = 0.8;
  double x2_start =0.2, y2_start =0.2, z2_start = 0.2;
  // current pos
  double x1, y1, z1, r1;
  double x2, y2, z2, r2;
  // target
  double x1_end, y1_end, z1_end;
  double x2_end, y2_end, z2_end;
  // interpolation step
  double step = 0.001;

  container con(0, 1, 0, 1, 0, 1, 5, 5, 5, false, false, false, 8);

  // add random target pos
  x1_end = rnd();
  y1_end = rnd();
  z1_end = rnd();
  // x1_end = x2_start;
  // y1_end = y2_start;
  // z1_end = z2_start;
  cout << "Start 1: " << x1_start << " " << y1_start << " " << z1_start << endl;
  cout << "End   1: " << x1_end << " " << y1_end << " " << z2_end << endl;

  x2_end = rnd();
  y2_end = rnd();
  z2_end = rnd();

  // x2_end = x1_start;
  // y2_end = y1_start;
  // z2_end = z1_start;
  
  cout << "Start 2: " << x2_start << " " << y2_start << " " << z2_start << endl;
  cout << "End   2: " << x2_end << " " << y2_end << " " << z2_end << endl;

  x1 = x1_start;
  y1 = y1_start;
  z1 = z1_start;

  x2 = x2_start;
  y2 = y2_start;
  z2 = z2_start;

  con.put(i, x1, y1, z1);
  con.put(j, x2, y2, z2); 

  con.draw_particles("find_voro_cell_p.gnu");

  // lambda function to get the sign of a double

  while (true) {
    // clear the container
    con.clear();
    // add the particles
    con.put(i, x1, y1, z1);
    con.put(j, x2, y2, z2);
    // compute linear interpolation
    x1 = x1 + step * sign(x1_end - x1);
    y1 = y1 + step * sign(y1_end - y1);
    z1 = z1 + step * sign(z1_end - z1);

    x2 = x2 + step * sign(x2_end - x2);
    y2 = y2 + step * sign(y2_end - y2);
    z2 = z2 + step * sign(z2_end - z2);

    // Output the particle positions in gnuplot format
    con.draw_particles("find_voro_cell_p.gnu");

    // Output the Voronoi cells in gnuplot format and a file with the
    // comparisons between the Voronoi cell volumes and the sampled volumes
    FILE *f1 = safe_fopen("find_voro_cell.vol", "w");
    FILE *f2 = safe_fopen("find_voro_cell_v.gnu", "w");
    c_loop_all cla(con);
    voronoicell c;
    if (cla.start()) {
      do {
        if (con.compute_cell(c, cla)) {
          // Get the position and ID information for the particle
          // currently being considered by the loop. Ignore the radius
          // information.
          // cla.pos(j, x1, y1, z1, r1);
          // cla.pos(i, x2, y2, z2, r2);
          
          cla.pos(j, x, y, z,r1);
          
          // Save and entry to the .vol file, storing both the computed
          // Voronoi cell volume, and the sampled volume based on the
          // number of grid points that were inside the cell
          
          // fprintf(f1, "%d %g %g %g %d %g %g %g", i, x1, y1, z1, j, x2, y2, z2);
          fprintf(f1, "%d %g %g %g", j, x, y, z);

          // Draw the Voronoi cell
          // c.draw_gnuplot(x1, y1, z1, f2);
          // c.draw_gnuplot(x2, y2, z2, f2);
          c.draw_gnuplot(x, y, z, f2);
          

        }
      } while (cla.inc());
    }
    fclose(f1);
    fclose(f2);
    usleep(50000);
    cout << "X1 Y1 Z1: "<<x1<<" "<<y1<<" "<<z1 << "| X2 Y2 Z2: "<<x2<<" "<<y2<<" "<<z2<<endl;
    // if (sqrt(pow((x1 - x1_end),2) + pow((y1 - y1_end),2) + pow((z1 - z1_end),2)) <= step){
    //   break;
    // }
    // if (sqrt(pow((x2 - x2_end),2) + pow((y2 - y2_end),2) + pow((z2 - z2_end),2)) <= step){
    //   break;
    // }
  }
}
