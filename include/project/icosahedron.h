#ifndef ICOSAHEDRON_H
#define ICOSAHEDRON_H
#include "project/utils.h"


void getIcosahedronFaceVertices(
    double con_size_xmin, double con_size_xmax, double con_size_ymin,
    double con_size_ymax, double con_size_zmin, double con_size_zmax,
    double center_x, double center_y, double center_z,
    std::map<int, std::vector<Eigen::Vector3d>> &faces_vertices_,
    std::vector<Eigen::Vector3d> &face_centers_,
    std::vector<Eigen::Vector3d> &face_normals_,
    // std::shared_ptr<voro::container> container_,
    std::shared_ptr<voro::container> container_pdf_,
    std::vector<std::shared_ptr<voro::wall_plane>> &planes) {
  double scale = 2.0;
  container_pdf_->clear();
  utils::initIcosahedronPlanes(planes, scale);
  for (auto plane : planes) {
    container_pdf_->add_wall(*plane);
  }
  container_pdf_->put(0, center_x, center_y, center_z);

  // print container
  voro::c_loop_all clp(*container_pdf_);
  voro::voronoicell c_pdf;
  // this will do just one iteration
  if (clp.start()) do {
      container_pdf_->compute_cell(c_pdf, clp);
    } while (clp.inc());
  std::map<int, std::vector<int>> vertices_indeces;
  std::vector<int> face_vertices_indices;
  c_pdf.face_vertices(face_vertices_indices);

  // //populate map

  int length = 0;
  int j = 0, k = 0;
  for (int i = 0; i < face_vertices_indices.size(); i += length + 1) {
    length = face_vertices_indices[i];
    if (length == 0) {
      break;
    }
    for (int j = i + 1; j < i + 1 + length; j++) {
      vertices_indeces[k].push_back(face_vertices_indices[j]);
    }
    k++;
  }

  int i = 0;

  std::vector<Eigen::Vector3d> vertices;
  std::vector<double> tmp_v;

  c_pdf.vertices(tmp_v);

  for (int i = 0; i < tmp_v.size(); i += 3) {
    vertices.push_back(Eigen::Vector3d(tmp_v[i], tmp_v[i + 1], tmp_v[i + 2]));
  }

  // loop face indices
  for (auto const &face : vertices_indeces) {
    std::vector<Eigen::Vector3d> face_vertices;
    for (auto const &vertex_index : face.second) {
      face_vertices.push_back(vertices[vertex_index]);
    }
    faces_vertices_[face.first] = face_vertices;
  }

  int z = 0;
  container_pdf_->clear();
  Eigen::Vector3d center(center_x, center_y, center_z);
  for (auto const &face : faces_vertices_) {
    Eigen::Vector3d face_center = 0.5 * utils::computeCenter(face.second, center);
    if (container_pdf_->point_inside(face_center.x(), face_center.y(), face_center.z())) {
      container_pdf_->put(z, face_center.x(), face_center.y(), face_center.z());
      face_centers_.push_back(face_center);
    }
    z++;
  }
  // c_loop_all clp2(*container_pdf_);
  // voronoicell c_pdf2;
  // i = 0;

  // compute normals given face centers
  for (auto center : face_centers_) {
    // consider vector from center to origin
    Eigen::Vector3d normal = center;
    normal.normalize();
    face_normals_.push_back(normal);
  }
}

#endif