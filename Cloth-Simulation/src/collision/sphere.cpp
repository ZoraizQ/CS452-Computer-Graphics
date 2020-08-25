#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  // compare point mass position to origin, if the pm is inside the radius of the sphere, bump it to the surface (tangent point)
  Vector3D dirPO = pm.position - origin;
  double magPO = dirPO.norm(); //distance of PM from Origin
  dirPO.normalize(); //normalize since the direction is required only
  if (magPO <= radius){
    //using a tangent from the pm's last position to the origin up to the radius
    Vector3D tangentPoint = origin + radius * dirPO; //using p=o+td, where t is the radius, d is the direction of the straight line from position to origin
    Vector3D correction = tangentPoint - pm.last_position; //correction vector to translate pm from its last position to the tangent point
    pm.position = pm.last_position + correction * (1-friction); //applied with friction
  }
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  Misc::draw_sphere(shader, origin, radius * 0.92);
}
