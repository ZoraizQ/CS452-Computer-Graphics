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

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // Part 2, Task 1:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
  // BVHAccel -> BVHNode *root,bb,(l,r) -- NULL at leaves only,*prims (active scene primitives) -- non-NULL for leaf nodes

  BBox centroid_box, bbox;

  for (Primitive *p : prims) { // for each primitive p
    BBox bb = p->get_bbox(); //get bounding box of prim
    bbox.expand(bb); //expand bounding box to include arg (3D vec or bbox)
    Vector3D c = bb.centroid();
    centroid_box.expand(c);
  }
  //min,max,extent

  BVHNode *node = new BVHNode(bbox); // new node with bb initialized enclosing all the primitives initially

  if (prims.size() <= max_leaf_size){ //leaf node, base case
    node->prims = new vector<Primitive *>(prims); //prims added to the node's prims (since leaf node)
    return node;
  }

  // pick axis to recurse on, largest dimension of the bounding box's extent
  int recAxis;
  Vector3D bboxdim = centroid_box.extent;
  //cout << "mybboxdim:"<<bboxdim << endl;
  double maxdimval = max(bboxdim.z,max(bboxdim.x,bboxdim.y));
  if (maxdimval == bboxdim.x)
    recAxis = 0;
  else if (maxdimval == bboxdim.y)
    recAxis = 1;
  else
    recAxis = 2;
  //cout << "recaxis:" << recAxis << endl;
  
  //split point
  Vector3D splitPoint = centroid_box.centroid();
  vector<Primitive*> leftprims, rightprims;
  for (Primitive *p : prims) { // for each primitive p
    Vector3D currentPC = p->get_bbox().centroid(); //get current primitive's centroid
    if (currentPC[recAxis] <= splitPoint[recAxis]){ //current primitive's centroid is less than equal to parent centroid (splitPoint) on given axis
      leftprims.push_back(p);
    }
    else{
      rightprims.push_back(p);
    }
  }

  node->l = construct_bvh(leftprims, max_leaf_size);
  node->r = construct_bvh(rightprims, max_leaf_size);

  return node;
}

bool isInInterval(double a, double low, double high){
  if (a > low && a < high)
    return true;
  else
    return false;
}

bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {
  // Part 2, task 3: replace this.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit. (return true after first hit)

  //if node is leaf, test intersect with all prims in node, get closest intersect of ray with all children
 
  // BBox has -> bool intersect(const Ray& r, double& t0, double& t1) const, like Primitive as well
  // "If the ray intersected the bouding box within the range given by, t0, t1, update t0 and t1 with the new intersection times."
  double t0, t1; //intersection times for ray with node's bounding box
  bool rayIntersectsNode = node->bb.intersect(ray, t0, t1); //t0 and t1 are updated after this
  
  if (rayIntersectsNode){ //return false if ray does not intersect node.bbox
    //return false if ray intersects with the node.bbox but t not in interval min_t - max_t
    if (!isInInterval(t0,ray.min_t,ray.max_t) && !isInInterval(t1,ray.min_t,ray.max_t)){
      return false;
    }
    if (node->l == NULL && node->r == NULL){ // if node is leaf node
      for (Primitive *p : *(node->prims)) {
        total_isects++;
        if (p->intersect(ray)) 
          return true; //first hit, just return true, else function goes to return 0
      }
    }
    else{
      //not leaf node, then recurse left and right
      // if there's even one hit, the entire recursive function must entirely return true, for that true
      bool l = intersect(ray, node->l);
      bool r = intersect(ray, node->r);
      return l || r; // || with a  single true will give an entirely true result
    }
  }
  return false;

}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {
  // Part 2, task 3: replace this
  // must return the closest hit, check every node's bbox hit by ray
  // r.max_t is updated by primitives, so automatically closest hit recieved (smallest intersection distance)

  double t0, t1; //intersection times for ray with node's bounding box
  bool rayIntersectsNode = node->bb.intersect(ray, t0, t1); //t0 and t1 are updated after this
  
  if (rayIntersectsNode){ //return false if ray does not intersect node.bbox
    //return false if ray intersects with the node.bbox but t not in interval min_t - max_t
    if (!isInInterval(t0,ray.min_t,ray.max_t) && !isInInterval(t1,ray.min_t,ray.max_t)){
      return false;
    }
    if (node->l == NULL && node->r == NULL){ // if node is leaf node
      bool hit = false;
      for (Primitive *p : *(node->prims)) {
        total_isects++;
        if (p->intersect(ray, i)) 
        hit = true;
      }
      return hit;
    }
    //not leaf node, then recurse left and right
    bool l = intersect(ray, i, node->l);
    bool r = intersect(ray, i, node->r);
    return l || r;
  }
  return false;
  
}

}  // namespace StaticScene
}  // namespace CGL
