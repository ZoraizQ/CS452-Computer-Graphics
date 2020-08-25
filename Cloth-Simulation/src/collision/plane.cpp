#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.
  Vector3D lpToPlane = point - pm.last_position;
  Vector3D pToPlane = point - pm.position;

  Vector3D normaldir = normal.unit();
  // using vector to point (lp and p) in plane with the normal to check with the dot product if one of them has crossed over and are not on the same side (intersection with plane)
  bool notOnSameSide = (dot(pToPlane, normal) <= 0 && dot(lpToPlane, normal) > 0) || (dot(lpToPlane, normal) <= 0 && dot(pToPlane, normal) > 0);
  if(notOnSameSide){
    double distance = dot(pToPlane, normaldir); //distance from current pm pos to plane in direction of normal
    Vector3D tangentPoint = pm.position + distance * normaldir; //using p = o + td, get tangent point from pm's position going in direction of normal by distance calculated
    
    Vector3D correction = tangentPoint - pm.last_position; //correction vector to bring pm from last pos to tangent point
    correction += normal * SURFACE_OFFSET; //lift it slightly upwards (use normal) from the plane by the given surface offset
    pm.position = pm.last_position + correction * (1.0 - friction); //apply with friction
  }

}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 2 * (sCross + sParallel);
  positions.col(1) << sPoint + 2 * (sCross - sParallel);
  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
  positions.col(3) << sPoint + 2 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("in_color", false) != -1) {
    shader.setUniform("in_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  shader.uploadAttrib("in_normal", normals);

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
