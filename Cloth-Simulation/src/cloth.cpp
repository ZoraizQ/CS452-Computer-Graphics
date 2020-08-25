#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

double randFromRange(double lower, double upper)
{
  return ((double(rand()) / double(RAND_MAX)) * (upper - lower)) + lower;
}

int getValidIndex(int y, int x, int nwp, int nhp){
  if (y < 0 || x < 0 || x >= nwp || y >= nhp) // to handle edge cases
    return -1; //invalid index
    
  return y * nhp + x; //(1,2) = 1*4+2 == 6
}

bool isPinned(const vector<vector<int>> &pinned, int x, int y){
  for (int i = 0; i < pinned.size(); i++){
    if (pinned[i][0] == y && pinned[i][1] == x)
      return true;
  }
  return false; 
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  // evenly spaced, num_width_points span width, height spanned by num_height_points
  srand(time(NULL));
  // cout <<"creating points" << endl;
  double xstep = width/(num_width_points-1);
  double ystep = height/(num_height_points-1); //17/(5-1) == 4.25 -> 0, 4.25, 8.5, 12.75, 17
  // number of masses =  num_width_points * num_height_points
  if (orientation == HORIZONTAL){ // orientation = HORIZONTAL, y-coords for all PMs  = 1, xz pos vary
    for (int y = 0; y < num_height_points; y++){
      for (int x = 0; x < num_width_points; x++){
        double posx = x*xstep;
        double posz = y*ystep;
        point_masses.push_back(PointMass(Vector3D(posx, 1.0, posz), isPinned(pinned, x, y)));
      }  
    } 
  }
  else if (orientation == VERTICAL){ // orientation = VERTICAL, z = random offset btw -0.001 and 0.001, xy pos vary
    for (int y = 0; y < num_height_points; y++){
      for (int x = 0; x < num_width_points; x++){
        double posx = x*xstep;
        double posy = y*ystep;
        bool pmPinned = isPinned(pinned, x, y);
        point_masses.push_back(PointMass(Vector3D(posx, posy, randFromRange(-0.001, 0.001)), isPinned(pinned, x, y)));
      }  
    }
  }
  for (int y = 0; y < num_height_points; y++){
    for (int x = 0; x < num_width_points; x++){
      int current = getValidIndex(y, x, num_width_points, num_height_points);
      int above = getValidIndex(y-1, x, num_width_points, num_height_points);
      int left = getValidIndex(y, x-1, num_width_points, num_height_points);
      int diagupleft = getValidIndex(y-1, x-1, num_width_points, num_height_points);
      int diagupright = getValidIndex(y-1, x+1, num_width_points, num_height_points);
      int righttwo = getValidIndex(y, x-2, num_width_points, num_height_points);
      int abovetwo = getValidIndex(y-2, x, num_width_points, num_height_points);

      //structural
      if (above != -1) 
        springs.push_back(Spring(&point_masses[current], &point_masses[above], STRUCTURAL));

      if (left != -1)
        springs.push_back(Spring(&point_masses[current], &point_masses[left], STRUCTURAL)); 

      //shearing
      if (diagupleft != -1) 
        springs.push_back(Spring(&point_masses[current], &point_masses[diagupleft], SHEARING));

      if (diagupright != -1) 
        springs.push_back(Spring(&point_masses[current], &point_masses[diagupright], SHEARING));

      //bending
      if (righttwo != -1) 
        springs.push_back(Spring(&point_masses[current], &point_masses[righttwo], BENDING));

      if (abovetwo != -1)
        springs.push_back(Spring(&point_masses[current], &point_masses[abovetwo], BENDING));
    } 
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D externalForces(0,0,0);
  //F = ma -> F = m(a1+a2+a3...)
  for (int i = 0; i < external_accelerations.size(); i++){
    externalForces += external_accelerations[i];
  }
  externalForces *= mass;
  // apply this to all point masses
  for(int p = 0; p < point_masses.size(); p++){
    point_masses[p].forces = externalForces;
  }

  for (int s = 0; s < springs.size(); s++){
    Vector3D forceDir = springs[s].pm_b->position - springs[s].pm_a->position; //vector from A to B pm
    double magAB = forceDir.norm(); //get magnitude
    forceDir.normalize(); //normalize since we only need the direction for the internal spring force

    Vector3D springForce = cp->ks * (magAB - springs[s].rest_length) * forceDir; // F_s = k_s * (|| p_a - p_b || - l) * F_d
    if (springs[s].spring_type == STRUCTURAL && cp->enable_structural_constraints){ //constraint type is STRUCTURAL and these are enabled by GUI then
      springs[s].pm_a->forces += springForce;
      springs[s].pm_b->forces -= springForce;
    } // add spring force to particle A and subtract from B in every case, N3L
    if (springs[s].spring_type == SHEARING && cp->enable_shearing_constraints){
      springs[s].pm_a->forces += springForce;
      springs[s].pm_b->forces -= springForce;
    }
    if (springs[s].spring_type == BENDING && cp->enable_bending_constraints){
      springs[s].pm_a->forces += springForce;
      springs[s].pm_b->forces -= springForce;
    }
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for(int p = 0; p < point_masses.size(); p++){
    if(!point_masses[p].pinned){ //if pinned, do nothing
      Vector3D a_dt = point_masses[p].forces / mass; // a = F/m by N2L
      Vector3D x_dt = point_masses[p].position;
      point_masses[p].position = x_dt + (1 - cp->damping/100) * (x_dt - point_masses[p].last_position) + a_dt * delta_t * delta_t; //new_position
      point_masses[p].last_position = x_dt;
    }
  }


  // TODO (Part 4): Handle self-collisions.
  // This won't do anything until you complete Part 4.
  build_spatial_map();
  for (PointMass &pm : point_masses) {
    self_collide(pm, simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  // This won't do anything until you complete Part 3.
  for (PointMass &pm : point_masses) {
    for (CollisionObject *co : *collision_objects) {
      co->collide(pm);
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int s = 0; s < springs.size(); s++){
    Vector3D forceDir = springs[s].pm_b->position - springs[s].pm_a->position; //vector from A to B pm
    double magAB = forceDir.norm(); //get magnitude
    forceDir.normalize(); //normalize since we only need the direction for the internal spring force
    
    bool Apinned = springs[s].pm_a->pinned;
    bool Bpinned = springs[s].pm_b->pinned;
    if (magAB > springs[s].rest_length * 1.1){   //spring's length is greater than 110% of the rest_length at any time_step
      double lengthDiff = magAB - springs[s].rest_length * 1.1;
      if(!Apinned && !Bpinned){ //both are not pinned
        forceDir *= (lengthDiff/2.0); //half correction to each point mass
        springs[s].pm_a->position += forceDir;
        springs[s].pm_b->position -= forceDir;
      }
      else if(Apinned != Bpinned){ //one of them is pinned
        forceDir *= lengthDiff;
        if (Apinned){ //apply entire force to B
          springs[s].pm_b->position -= forceDir;
        }
        else{ //apply entire force to A, since B is pinned
          springs[s].pm_a->position += forceDir;
        }
      }
    }
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (int p = 0; p < point_masses.size(); p++){
    float hkey = hash_position(point_masses[p].position);
    if (!map.count(hkey)){ //if it is already not in map using unordered map's count  
      map[hkey] = new vector<PointMass*>(); //for a new key dynamically allocate memory for a point mass* vector
    }
    else{ //if key already exists, just push back to its vector
      map[hkey]->push_back(&point_masses[p]);
    }
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // // TODO (Part 4): Handle self-collision for a given point mass.
  float hkey = hash_position(pm.position);
  if (map.count(hkey)){ //if key exists in hashtable
    Vector3D avg_correction(0, 0, 0);//apply corrections
    int num_corrections = 0;
    vector<PointMass*> hashedPMs = *map[hkey];
    for (int p = 0; p < hashedPMs.size(); p++){
      PointMass* candidatePM = hashedPMs[p]; //get every point mass in current map[hash]'s pm vector
      if (candidatePM->position == pm.position) //to prevent self collision
        continue; 
      
      Vector3D PtoC = candidatePM->position - pm.position; 
      double separation = PtoC.norm(); //magnitude of vector from PM pos to candidate PM pos
      double exceededDist = 2.0*thickness - separation; 
      if (exceededDist > 0){ //too close, add contribution for this candidate PM to correction vector
        Vector3D CtoPdir = pm.position - candidatePM->position;
        CtoPdir.normalize(); //just need direction
        avg_correction += (exceededDist * CtoPdir);
        num_corrections++;
      } 
    }
    avg_correction /= (double)num_corrections; //to get average

    if (num_corrections > 0){ //there are some corrections to be made (particles self colliding)
      pm.position += avg_correction / simulation_steps; //apply avg correction scaled down by simulation steps
    }
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents
  // membership in some uniquely identified 3D box volume.
  float h = 3 * height/num_height_points; //3 empirically proven constant for spatial hashing accuracy
  float w = 3 * width/num_width_points;
  float t = max(h, w);
  //dimensions of each 3d box w*h*t

  //truncating pos to closest 3d box
  float newx = (pos.x-fmod(pos.x, w))/w; //truncate x coordinate with remainder of pos.x/w to get to closest 3Dbox coords and divide over w (xrange dimension of box)
  float newy = (pos.y-fmod(pos.y, h))/h;
  float newz = (pos.z-fmod(pos.z, t))/t;
  
  //float unique_key = newx + (newy * h) + (newz * w * h); // for an order sensitive, unique hash
  float unique_key = newx + (newy * h) + (newz * w * h); // for an order sensitive, unique hash 
  return unique_key;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm, pm + num_width_points, pm + 1));
      triangles.push_back(new Triangle(pm + 1, pm + num_width_points,
                                       pm + num_width_points + 1));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
