#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  BBox bb(p1);
  bb.expand(p2);
  bb.expand(p3);
  return bb;

}

double stp(Vector3D a, Vector3D b, Vector3D c){ //DETERMINANT == (STP) SCALAR TRIPLE PRODUCT == a . (b x c) == (a x b) . c
  return dot(a,cross(b,c));
}

bool isInRange2(double a, double low, double high){
  if (a > low && a < high)
    return true;
  else
    return false;
}

bool Triangle::intersect(const Ray& r) const {

  // Part 1, Task 3: implement ray-triangle intersection
  Vector3D A(mesh->positions[v1]), B(mesh->positions[v2]), C(mesh->positions[v3]); //A,B,C position vectors
  Vector3D AB = B-A, AC = C-A; //3D vectors for edges AB, AC of the triangle ABC
  Vector3D trans = r.o - A; // O - A == transformation to move tri to origin from original pos with first vertex at origin
  
  // express a unit triangle on the plane, distant by t from origin
  // t = distance from ray origin to P (intersection point)
  //p = r.o + t * r.d ray equation
  
  //cross product AB x AC == triangle normal -> if dot product (rd . triangle normal) == 0, ray parallel to the triangle -> no intersection
  double det = stp(AB,r.d,AC); //(AB) . (r.d x AC) is done by stp and if == 0 goes to false
  if (det != 0){ // if det is negative even, backward culling triangle
    double invDet,u,v,w,t;

    invDet = 1.0 / det;
    u = stp(trans,r.d,AC) * invDet;
    v = stp(AB,r.d,trans) * invDet;
    t = stp(AB,AC,trans) * invDet;

    // Conditions: (min_t < t < max_t) and (0 <= u <= 1) and (0 <= v <= 1) and (u + v <= 1)
    if (isInRange2(t, r.min_t, r.max_t) && u >= 0 && v >= 0 && u+v <= 1) { // if there is an intersection
      r.max_t = t; //replace this with your value of t
      return true;
    }
  }

  return false;
}


bool Triangle::intersect(const Ray& r, Intersection *isect) const {

  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  Vector3D A(mesh->positions[v1]), B(mesh->positions[v2]), C(mesh->positions[v3]); //A,B,C position vectors
  Vector3D nA(mesh->normals[v1]), nB(mesh->normals[v2]), nC(mesh->normals[v3]); //normals at vertices
  Vector3D AB = B-A, AC = C-A;
  Vector3D trans = r.o - A; 
  
  double det = stp(AB,r.d,AC); // so (AB) . (r.d x AC) is done by stp and if == 0 goes to false
  if (det != 0){ 
    double invDet,u,v,w,t;

    invDet = 1.0 / det;
    u = stp(trans,r.d,AC) * invDet;
    v = stp(AB,r.d,trans) * invDet;
    t = stp(AB,AC,trans) * invDet;
    w=(1-u-v);
    
    if (isInRange2(t, r.min_t, r.max_t) && u >= 0 && v >= 0 && u+v <= 1) { // if there is an intersection
      r.max_t = t; //replace this with your value of t
      isect->t = t; //replace this with your value of t
      isect->primitive = this;
      // parametrized intersection P in barycentric coords, for normal of intersection
      isect->n = w*nA + u*nB + v*nC; //replace this with your value of normal at the point of intersection
      isect->bsdf = get_bsdf(); // ptr to surface brdf on hit
      return true;
    }
  }

  return false;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CGL
