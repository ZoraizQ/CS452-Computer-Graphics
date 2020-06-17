#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool isInRange(double a, double low, double high){
  if (a > low && a < high)
    return true;
  else
    return false;
}

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  // Ray: P = r.o + t * r.d
  // Sphere: x^2 + y^2 + z^2 = R^2, implicit P^2 - R^2 = 0, plugin P
  // quadratic func -> ax^2 + bx + c, r.d not normalized
  Vector3D L = r.o - o; //ray origin - sphere center 
  double a = dot(r.d, r.d); // since r.d is normalized a == 1.0
  double b = 2 * dot(r.d, L); 
  double c = dot(L, L) - r2; //r2 == radius squared
  
  //  discriminant, when discriminant = 0, for ray -> one POI (t1=t2), when > 0, two POI (t1,t2), when < 0 no intersection
  double discrim = b*b-4*a*c;
  if (discrim >= 0){
    if (discrim == 0){
      t1 = t2 = -b/(2*a);  
    }
    else{
      t1 = (-b - sqrt(discrim)) / (2*a); //solutions t1, t2, where t1 is the smaller
      t2 = (-b + sqrt(discrim)) / (2*a);
    }
    // r.max_t=t2;
    
    if (!isInRange(t1, r.min_t, r.max_t) && !isInRange(t2, r.min_t, r.max_t)){ // not considerate of min_t, max_t range
      return false;
    }

    return true;
  }

  return false;
}

bool Sphere::intersect(const Ray& r) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1,t2; // not required in this intersect, so temporary vars still
  return test(r,t1,t2);
}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1,t2,t;
  if (!test(r,t1,t2)){
    return false;
  }

  // need to select from t1 and t2, the (smaller time) first one that is greater than min_t of the ray
  if (t1 > r.min_t){
    t = t1;
  }
  else{
    t = t2;
  }

  i->t = t;
  r.max_t = t;
  i->n = Vector3D(r.o + t * r.d - o); // where P = O+tD, o = C, normalize(P-C)
  (i->n).normalize(); //normalize the intersection normal as well
  
  i->bsdf = get_bsdf();

  return true;

}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
