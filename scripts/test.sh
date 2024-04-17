#!/usr/bin/gnuplot
set xrange [0:1]
set yrange [0:1]
set zrange [0:1]
set size ratio 1
splot 'find_voro_cell_v.gnu' w l t 'Voronoi cells', \
      'find_voro_cell_p.gnu' u 2:3:4 t 'Particle_1'
pause 0.01
reread

# splot 'find_voro_cell_v.gnu' w l t 'Voronoi cells', \
# 'find_voro_cell.vec' w vec t 'Voronoi cell vectors', \
# 'find_voro_cell_p.gnu' u 6:7:8 t 'Particle_2'