#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c);
  } else {
    draw(node->l, c);
    draw(node->r, c);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c);
  } else {
    drawOutline(node->l, c);
    drawOutline(node->r, c);
  }
}

int largestDimension(Vector3D v) {
  if (v.x >= v.y && v.x >= v.z) {
    return 0;
  }
  else if (v.y >= v.x && v.y >= v.z) {
    return 1;
  }
  else if (v.z >= v.x && v.z >= v.y) {
    return 2;
  }
  else {
    return 3;
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {


  BBox centroid_box, bbox;

  for (Primitive *p : prims) {
    BBox bb = p->get_bbox();
    bbox.expand(bb);
    Vector3D c = bb.centroid();
    centroid_box.expand(c);
  }

  BVHNode *node = new BVHNode(bbox);

  if (prims.size() <= max_leaf_size) {
    // this is a leaf node
    node->prims = new vector<Primitive *>(prims);
    return node;
  }
  else {
    int axis = largestDimension(centroid_box.extent);
    vector<Primitive *> prims1 = vector<Primitive *>();
    vector<Primitive *> prims2 = vector<Primitive *>();
    for (Primitive *p : prims) {
      if (p->get_bbox().centroid()[axis] <= centroid_box.centroid()[axis]) {
        prims1.push_back(p);
      }
      else {
        prims2.push_back(p);
      }
    }
    node->l = construct_bvh(prims1, max_leaf_size);
    node->r = construct_bvh(prims2, max_leaf_size);
  }
  return node;

}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }
  else {
    if (t1 < ray.min_t || t0 > ray.max_t) {
      return false;
    }
    else {

      // node = leaf node
      if (node->isLeaf()) {
        bool hit = false;
        for (Primitive *p : *(node->prims)) {
          total_isects++;
          if (p->intersect(ray)) {
            // return true;
            hit = true;
          }
        }
        return hit;
      }
      // node != leaf node
      else {
        bool left = false;
        bool right = false;
        left = left || intersect(ray, node->l);
        right = right || intersect(ray, node->r);
        return left || right;
      }
    }
  }
}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {
  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }
  else {
    // return true;
    if (t1 < ray.min_t || t0 > ray.max_t) {
      return false;
    }
    else {
      // node = leaf node
      if (node->isLeaf()) {
        bool hit = false;
        for (Primitive *p : *(node->prims)) {
          total_isects++;
          if (p->intersect(ray, i)) {

            hit = true;
          }
        }
        return hit;
      }
      // node != leaf node
      else {
        bool left = false;
        bool right = false;
        left = left || intersect(ray, i, node->l);
        right = right || intersect(ray, i, node->r);
        return left || right;
      }
    }
  }
}

}  // namespace StaticScene
}  // namespace CGL
