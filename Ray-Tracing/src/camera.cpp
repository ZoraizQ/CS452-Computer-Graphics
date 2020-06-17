#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

void Camera::configure(const CameraInfo& info, size_t screenW, size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  nClip = info.nClip;
  fClip = info.fClip;
  hFov = info.hFov;
  vFov = info.vFov;

  double ar1 = tan(radians(hFov) / 2) / tan(radians(vFov) / 2);
  ar = static_cast<double>(screenW) / screenH;
  if (ar1 < ar) {
    // hFov is too small
    hFov = 2 * degrees(atan(tan(radians(vFov) / 2) * ar));
  } else if (ar1 > ar) {
    // vFov is too small
    vFov = 2 * degrees(atan(tan(radians(hFov) / 2) / ar));
  }
  screenDist = ((double) screenH) / (2.0 * tan(radians(vFov) / 2));
}

void Camera::place(const Vector3D& targetPos, const double phi,
                   const double theta, const double r, const double minR,
                   const double maxR) {
  double r_ = min(max(r, minR), maxR);
  double phi_ = (sin(phi) == 0) ? (phi + EPS_F) : phi;
  this->targetPos = targetPos;
  this->phi = phi_;
  this->theta = theta;
  this->r = r_;
  this->minR = minR;
  this->maxR = maxR;
  compute_position();
}

void Camera::copy_placement(const Camera& other) {
  pos = other.pos;
  targetPos = other.targetPos;
  phi = other.phi;
  theta = other.theta;
  minR = other.minR;
  maxR = other.maxR;
  c2w = other.c2w;
}

void Camera::set_screen_size(const size_t screenW, const size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  ar = 1.0 * screenW / screenH;
  hFov = 2 * degrees(atan(((double) screenW) / (2 * screenDist)));
  vFov = 2 * degrees(atan(((double) screenH) / (2 * screenDist)));
}

void Camera::move_by(const double dx, const double dy, const double d) {
  const double scaleFactor = d / screenDist;
  const Vector3D& displacement =
    c2w[0] * (dx * scaleFactor) + c2w[1] * (dy * scaleFactor);
  pos += displacement;
  targetPos += displacement;
}

void Camera::move_forward(const double dist) {
  double newR = min(max(r - dist, minR), maxR);
  pos = targetPos + ((pos - targetPos) * (newR / r));
  r = newR;
}

void Camera::rotate_by(const double dPhi, const double dTheta) {
  phi = clamp(phi + dPhi, 0.0, (double) PI);
  theta += dTheta;
  compute_position();
}

void Camera::compute_position() {
  double sinPhi = sin(phi);
  if (sinPhi == 0) {
    phi += EPS_F;
    sinPhi = sin(phi);
  }
  const Vector3D dirToCamera(r * sinPhi * sin(theta),
                             r * cos(phi),
                             r * sinPhi * cos(theta));
  pos = targetPos + dirToCamera;
  Vector3D upVec(0, sinPhi > 0 ? 1 : -1, 0);
  Vector3D screenXDir = cross(upVec, dirToCamera);
  screenXDir.normalize();
  Vector3D screenYDir = cross(dirToCamera, screenXDir);
  screenYDir.normalize();

  c2w[0] = screenXDir;
  c2w[1] = screenYDir;
  c2w[2] = dirToCamera.unit();   // camera's view direction is the
                                 // opposite of of dirToCamera, so
                                 // directly using dirToCamera as
                                 // column 2 of the matrix takes [0 0 -1]
                                 // to the world space view direction
}

void Camera::dump_settings(string filename) {
  ofstream file(filename);
  file << hFov << " " << vFov << " " << ar << " " << nClip << " " << fClip << endl;
  for (int i = 0; i < 3; ++i)
    file << pos[i] << " ";
  for (int i = 0; i < 3; ++i)
    file << targetPos[i] << " ";
  file << endl;
  file << phi << " " << theta << " " << r << " " << minR << " " << maxR << endl;
  for (int i = 0; i < 9; ++i)
    file << c2w(i/3, i%3) << " ";
  file << endl;
  file << screenW << " " << screenH << " " << screenDist << endl;
  cout << "[Camera] Dumped settings to " << filename << endl;
}

void Camera::load_settings(string filename) {
  ifstream file(filename);

  file >> hFov >> vFov >> ar >> nClip >> fClip;
  for (int i = 0; i < 3; ++i)
    file >> pos[i];
  for (int i = 0; i < 3; ++i)
    file >> targetPos[i];
  file >> phi >> theta >> r >> minR >> maxR;
  for (int i = 0; i < 9; ++i)
    file >> c2w(i/3, i%3);
  file >> screenW >> screenH >> screenDist;
  cout << "[Camera] Loaded settings from " << filename << endl;
}

Ray Camera::generate_ray(double x, double y) const { //ray that reaches sample point returned, input point (x,y)
  // Part 1, Task 2:
  // compute position of the input sensor sample coordinate on the
  // canonical sensor plane one unit away from the pinhole.
  // Note: hFov and vFov are in degrees.
  
  Vector3D o = pos;
  // corners of the sensor plane, vertical horizontal fields of view, (0,0 maps to bottom-left and 1,1 maps to top-right)
  // ray's direction, converted to world space by c2w = camera 2 world space transform matrix, ray direction -> r.d, normalize(),
  // r.o -> camera pos, where nClip and fClip = min_t, max_t values
  Vector3D bottom_left = Vector3D(-tan(radians(hFov)*.5), -tan(radians(vFov)*.5),-1); // the bottom left of the sensor plane
  Vector3D top_right = Vector3D( tan(radians(hFov)*.5),  tan(radians(vFov)*.5),-1); // the top right of the sensor plane

  // starting from the bottom left, interpolate between the max x (top_right's) and y values depending on passed x,y values
  //ray direction vector
  Vector3D rd = Vector3D(bottom_left.x + (top_right.x - bottom_left.x) * x, bottom_left.y + (top_right.y - bottom_left.y) * y, -1.0); 
  // camera is aligned along the negative Z-axis so -1.0
  rd = c2w * rd; // overloaded -> Vector3D Matrix3x3::operator*( const Vector3D& x ) const
  rd.normalize();
  Ray outr = Ray(Vector3D(o), Vector3D(rd), fClip, 0); //depth initially 0, set to max by raytrace
  outr.min_t = nClip;
  // normalize direction
  return outr;
}


} // namespace CGL
