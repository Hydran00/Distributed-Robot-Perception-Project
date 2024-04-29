#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include "voro++.hh"

using namespace voro;

// Set up constants for the container geometry
const double x_min = -1, x_max = 1;
const double y_min = -1, y_max = 1;
const double z_min = -1, z_max = 1;
const double cvol = (x_max - x_min) * (y_max - y_min) * (z_max - z_min);

// Set up the number of blocks that the container is divided into
const int n_x = 6, n_y = 6, n_z = 6;

// Set the number of particles that are going to be randomly introduced
const int particles = 20;

// Function to generate equidistant points on the surface of a sphere
std::vector<double> equidistant_sphere(int N) {
    std::vector<double> points;
    double a = 4 * M_PI / N;
    double d = sqrt(a);
    int M_theta = round(M_PI / d);
    double d_theta = M_PI / M_theta;
    
    for (int m = 0; m < M_theta; ++m) {
        double theta = M_PI * (m + 0.5) / M_theta;
        int M_phi = round(2 * M_PI * sin(theta) / d_theta);
        double d_phi = 2 * M_PI / M_phi;
        
        for (int n = 0; n < M_phi; ++n) {
            double phi = 2 * M_PI * n / M_phi;
            double x = cos(phi) * sin(theta);
            double y = sin(phi) * sin(theta);
            double z = cos(theta);
            points.push_back(x);
            points.push_back(y);
            points.push_back(z);
        }
    }

    return points;
}

int main() {
    int i;
    double x, y, z;

    // Create a container with the geometry given above, and make it
    // non-periodic in each of the three coordinates. Allocate space for
    // eight particles within each computational block
    container con(x_min, x_max, y_min, y_max, z_min, z_max, n_x, n_y, n_z, false,
                  false, false, 8);

    // wall_sphere sph(0, 0, 0, 0.98);
    // con.add_wall(sph);
    wall_sphere sph2(0, 0, 0, 1.0);
    con.add_wall(sph2);

    // Generate equidistant points on the surface of a sphere and add them into the container
    std::vector<double> points = equidistant_sphere(particles);
    for (i = 0; i < particles; ++i) {
        x = points[i * 3];
        y = points[i * 3 + 1];
        z = points[i * 3 + 2];
        con.put(i, x, y, z);
    }

    // Sum up the volumes, and check that this matches the container volume
    double vvol = con.sum_cell_volumes();
    printf(
        "Container volume : %g\n"
        "Voronoi volume   : %g\n"
        "Difference       : %g\n",
        cvol, vvol, vvol - cvol);

    // Output the particle positions in gnuplot format
    con.draw_particles("random_points_p.gnu");

    // Output the Voronoi cells in gnuplot format
    con.draw_cells_gnuplot("random_points_v.gnu");

    return 0;
}
