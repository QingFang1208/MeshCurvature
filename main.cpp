#include <fstream>
#include <algorithm>

#include "Math/Mrgb.h"
#include "PolyMesh/IOManager.h"

#define PI 3.14159265359

using namespace acamcad;
using namespace polymesh;

bool normalizeToZeroOne(std::vector<double>& vec,
                        double extreme_exclude = 0.1) {
  if (vec.empty()) return false;

  std::vector<double> data = vec;
  std::sort(data.begin(), data.end());
  int remove_count = static_cast<int>(data.size() * extreme_exclude);
  data.erase(data.begin(), data.begin() + remove_count);
  data.erase(data.end() - remove_count, data.end());

  double range = data.back() - data.front();
  if (range == 0) {
    for (auto& val : vec) {
      if (val > data.back())
        val = 1.0;
      else if (val < data.front())
        val = 0.0;
      else
        val = 0.5;
    }
  } else {
    for (auto& val : vec)
      val = std::min(std::max((val - data.front()) / range, 0.0), 1.0);
  }
  return true;
}

void map2mrgbf(const std::vector<double>& vec, std::vector<MRGBf>& color) {
  color.clear();
  color.reserve(vec.size());
  MRGBf a(0.999, 0.0, 0.0f), b(0.0f, 0.999f, 0.0f), c(0.0f, 0.0f, 0.999f);
  for (auto val : vec) {
    if (val < 0.5) {
      color.emplace_back(2 * val * b.R() + (1 - 2 * val) * a.R(),
                         2 * val * b.G() + (1 - 2 * val) * a.G(),
                         2 * val * b.B() + (1 - 2 * val) * a.B());
    } else {
      color.emplace_back((2 * val - 1) * c.R() + (2 - 2 * val) * b.R(),
                         (2 * val - 1) * c.G() + (2 - 2 * val) * b.G(),
                         (2 * val - 1) * c.B() + (2 - 2 * val) * b.B());
    }
  }
}

MVector3 cal_circum_enter(const MVector3& a, const MVector3& b,
                          const MVector3& c) {
  MVector3 ac = c - a, ab = b - a;
  MVector3 abXac = cross(ab, ac), abXacXab = cross(abXac, ab),
           acXabXac = cross(ac, abXac);
  return a + (abXacXab * ac.normSq() + acXabXac * ab.normSq()) /
                 (2.0 * abXac.normSq());
}

void cal_local_ave_region(PolyMesh* const mesh,
                          std::vector<double>& vertexLAR) {
  for (MPolyFace* fh : mesh->polyfaces()) {
    // judge if it's obtuse
    bool isObtuseAngle = false;
    MVert* obtuseVertexHandle;
    MHalfedge* he = fh->halfEdge();
    MHalfedge *he_next = he->next(), *he_prev = he->prev();
    MVert *v_from_he = he->fromVertex(),
          *v_from_he_next = he_next->fromVertex(),
          *v_from_he_prev = he_prev->fromVertex();
    MVector3 vec_he_nor = he->tangent(), vec_he_next_nor = he_next->tangent(),
             vec_he_prev_nor = he_prev->tangent();
    if (vectorAngle(vec_he_nor, -vec_he_prev_nor) > PI / 2.0) {
      isObtuseAngle = true;
      obtuseVertexHandle = v_from_he;
    } else if (vectorAngle(vec_he_next_nor, -vec_he_nor) > PI / 2.0) {
      isObtuseAngle = true;
      obtuseVertexHandle = v_from_he_next;
    } else if (vectorAngle(vec_he_prev_nor, -vec_he_next_nor) > PI / 2.0) {
      isObtuseAngle = true;
      obtuseVertexHandle = v_from_he_prev;
    }

    // calculate area
    if (isObtuseAngle) {
      double faceArea =
          0.5 * norm(cross(v_from_he_next->position() - v_from_he->position(),
                           v_from_he_prev->position() - v_from_he->position()));
      for (MVert* fv : mesh->polygonVertices(fh)) {
        if (fv == obtuseVertexHandle)
          vertexLAR[fv->index()] += faceArea / 2.0;
        else
          vertexLAR[fv->index()] += faceArea / 4.0;
      }
    } else {
      MVector3 cc =
          cal_circum_enter(v_from_he->position(), v_from_he_next->position(),
                           v_from_he_prev->position());
      for (MHalfedge* fhh : mesh->polygonHalfedges(fh)) {
        MVector3 edgeMidpoint =
            0.5 * (fhh->fromVertex()->position() + fhh->toVertex()->position());
        double edgeLength = fhh->edge()->length();
        double partArea = 0.5 * edgeLength * (edgeMidpoint - cc).norm();
        vertexLAR[fhh->fromVertex()->index()] += 0.5 * partArea;
        vertexLAR[fhh->toVertex()->index()] += 0.5 * partArea;
      }
    }
  }
}

void cal_mean_curvature(PolyMesh* const mesh,
                        const std::vector<double>& vertexLAR,
                        std::vector<double>& meanCurv) {
  meanCurv.clear();
  meanCurv.reserve(mesh->numVertices());

  for (MVert* vh : mesh->vertices()) {
    // Calculate vertex mean curvature
    // ...

    // ...
  }
  std::cout << "Calculate Mean Curvature Done" << std::endl;
}

void cal_gaussian_curvature(PolyMesh* const mesh,
                            const std::vector<double>& vertexLAR,
                            std::vector<double>& gaussCurv) {
  gaussCurv.clear();
  gaussCurv.reserve(mesh->numVertices());

  for (MVert* vh : mesh->vertices()) {
    // Calculate vertex Gaussian curvature
    // ...

    // ...
  }
  std::cout << "Calculate Gaussian Curvature Done" << std::endl;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "========== CurvatureHw Usage  ==========\n";
    std::cout << std::endl;
    std::cout << "Input:	MeshCurvature_HW.exe	mesh.obj\n";
    std::cout << std::endl;
    std::cout << "=================================================\n";
    return 0;
  }

  // read mesh, now only support obj
  std::string mesh_path = argv[1];
  PolyMesh* mesh = new PolyMesh();
  loadMesh(mesh_path, mesh);
  mesh->updateVerticesNormal();

  std::cout << "The curvature has area weight" << std::endl;
  std::vector<double> vertexLAR(mesh->numVertices(), 0.0);
  std::vector<double> meanCurv(mesh->numVertices(), 0.0);
  std::vector<double> gaussCurv(mesh->numVertices(), 0.0);
  cal_local_ave_region(mesh, vertexLAR);
  cal_mean_curvature(mesh, vertexLAR, meanCurv);
  cal_gaussian_curvature(mesh, vertexLAR, gaussCurv);

  std::vector<MRGBf> colors;
  IOOptions ioopt;
  ioopt.vert_have_color = true;
  if (normalizeToZeroOne(meanCurv)) {
    map2mrgbf(meanCurv, colors);
    for (int i = 0; i < mesh->numVertices(); i++) {
      mesh->vertices()[i]->setColor(colors[i]);
    }
    std::string colored_path = mesh_path;
    colored_path.replace(colored_path.rfind(".obj"), 4, "_mean.obj");
    writeMesh(colored_path, mesh, ioopt);
  }

  if (normalizeToZeroOne(gaussCurv)) {
    map2mrgbf(gaussCurv, colors);
    for (int i = 0; i < mesh->numVertices(); i++) {
      mesh->vertices()[i]->setColor(colors[i]);
    }
    std::string colored_path = mesh_path;
    colored_path.replace(colored_path.rfind(".obj"), 4, "_gauss.obj");
    writeMesh(colored_path, mesh, ioopt);
  }

  return 1;
}
