#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.

    // Bezier curve n degree, n+1 control points, 0 <= t <= 1, linear interpolation
    // evaluatedLevels is already seeded with the original control points of the Bezier curve, 4 in case of cubic
    // assume we are at any level, get most recent filled level
    
    int lasti = evaluatedLevels.size()-1;
    if (evaluatedLevels[lasti].size() == 1){ //last level has only 1 CP, then it has been completely evaluated
      return;
    } 

    evaluatedLevels.push_back(std::vector<Vector2D>()); //empty level added
    for (int c = 1; c < evaluatedLevels[lasti].size(); c++){ //for every intermediate control point
      Vector2D CP1 = evaluatedLevels[lasti][c-1];
      Vector2D CP2 = evaluatedLevels[lasti][c]; //previous and current CP
      Vector2D newCP = (1-t)* CP1 + t * CP2; //ratio from ray equation to interpolate with t between every two control points in the previous level
      evaluatedLevels[lasti+1].push_back(newCP);  //add new intermediate CP calculated
    }
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)

    /*Bezier surface (n, m) degree = (n+1)(m+1) control points
      Each 4x1 cp in u define a bezier curve, corresponding points = moving curve in v, u,v space = [0,1]^2
      1D de Casteljau on 4 CP to define moving curve*/

    // For each row i:
    //  Let q(i, u) := apply de Casteljau's algorithm with parameter u to the i-th row of control points
    vector<Vector3D> q;
    for (vector<Vector3D> rowi : controlPoints){
      q.push_back(evaluate1D(rowi, u));
    }

    Vector3D p = evaluate1D(q, v); // Let p(u, v) := apply de Casteljau's algorithm with parameter v to all q(i, u)
    return p; // Return p(u, v)
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
    
    vector<Vector3D> calculatedCP;
    while (points.size() != 1){ //only a single point left
      for (int c = 1; c < points.size(); c++){ //for every intermediate control point
        //previous and current CP
        Vector3D newCP = (1-t)* points[c-1] + t * points[c]; //ratio from ray equation to interpolate with t between every two control points in the previous level
        calculatedCP.push_back(newCP); //add new intermediate CP calculated to return vector
      }
      points = calculatedCP;
      calculatedCP.clear();
    }
    return calculatedCP[0]; //final point calculated
  }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    Vector3D n = Vector3D(0, 0, 0); //since normals are going to be added to it for avg, to store normal sum
    //average surrounding face normals, HalfedgeCIter for const, halfedge(), next(), twin(), position

    HalfedgeCIter h = halfedge(); // Since we're in a Vertex, this returns a halfedge away from the vertex
    h = h->twin(); //half-edge towards a vertex, h->next() another edge on same page
    HalfedgeCIter h_orig = h; // Save a copy of h 's value in another HalfedgeCIter h_orig
    do {
      Vector3D vA = h->vertex()->position;
      h = h->next();
      Vector3D vB = h->vertex()->position;
      h = h->twin(); //Once you've added in the area-weighted normal, you should advance h to the halfedge for the next face by, using the next() and twin() functions.
      Vector3D vC = h->vertex()->position;
      n += cross(vB-vA, vC-vA); /* Accumulate area-weighted normal of the current face in the variable n . You can do this by using the cross
      product of triangle edges. Since the cross product of two vectors (AB, AC) has a norm equal to twice the area of the triangle they define, these vectors
      are already area weighted! */
    }
    while(h != h_orig); // Start a while loop that ends when h == h_orig.
    //return the re-normalized unit vector n.unit()
    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    if(e0->isBoundary()) return e0;

    //triangle 1
    HalfedgeIter h11 = e0->halfedge();
    HalfedgeIter h12 = h11->next();
    HalfedgeIter h13 = h12->next();
    VertexIter b = h11->vertex();
    VertexIter a = h13->vertex();
    FaceIter f1 = h11->face();

    //triangle 2
    HalfedgeIter h21 = h11->twin();
    HalfedgeIter h22 = h21->next();
    HalfedgeIter h23 = h22->next();
    VertexIter c = h21->vertex();
    VertexIter d = h23->vertex();
    FaceIter f2 = h21->face();

    //new half edges
    h11->edge() = e0;
    h11->next() = h23;
    h23->next() = h12;
    h12->next() = h11;

    h11->twin() = h21;

    h21->edge() = e0;
    h21->next() = h13;
    h13->next() = h22;
    h22->next() = h21;

    //new vertices
    h11->vertex() = a;
    h12->vertex() = c;
    h13->vertex() = a;
    h21->vertex() = d;
    h22->vertex() = b;
    h23->vertex() = d;

    //new faces
    h11->face() = f1;
    h12->face() = f1;
    h13->face() = f2;
    h21->face() = f2;
    h22->face() = f2;
    h23->face() = f1;

    //face half edges
    f1->halfedge() = h11;
    f2->halfedge() = h21;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    if (e0->halfedge()->isBoundary()){
        return e0->halfedge()->vertex();
    }


    //triangle 1
    HalfedgeIter h01 = e0->halfedge();
    HalfedgeIter h1 = h01->next();
    HalfedgeIter h2 = h1->next();
    VertexIter b = h01->vertex();
    VertexIter a = h2->vertex();
    FaceIter f1 = h01->face();


    //triangle 2
    HalfedgeIter h02 = h01->twin();
    HalfedgeIter h3 = h02->next();
    HalfedgeIter h4 = h3->next();
    VertexIter c = h02->vertex();
    VertexIter d = h4->vertex();
    FaceIter f2 = h02->face();

    //new things
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h21 = newHalfedge();
    HalfedgeIter h22 = newHalfedge();
    HalfedgeIter h31 = newHalfedge();
    HalfedgeIter h32 = newHalfedge();
    HalfedgeIter h41 = newHalfedge();

    EdgeIter e1 = newEdge();
    EdgeIter e2 = newEdge();
    EdgeIter e3 = newEdge();

    FaceIter new_f1 = newFace();
    FaceIter new_f2 = newFace();

    VertexIter new_v = newVertex();

    //set all new half edges
    h1->setNeighbors(h21, h1->twin(), c, h1->edge(), new_f1);
    h2->setNeighbors(h01, h2->twin(), a, h2->edge(), f1);
    h3->setNeighbors(h41, h3->twin(), b, h3->edge(), new_f2);
    h4->setNeighbors(h02, h4->twin(), d, h4->edge(), f2);

    h01->setNeighbors(h22, h31, b, e0, f1);
    h02->setNeighbors(h32, h11, c, e1, f2);
    h11->setNeighbors(h1, h02, new_v, e1, new_f1);
    h21->setNeighbors(h11, h22, a, e2, new_f1);
    h22->setNeighbors(h2, h21, new_v, e2, f1);
    h31->setNeighbors(h3, h01, new_v, e0, new_f2);
    h32->setNeighbors(h4, h41, new_v, e3, f2);
    h41->setNeighbors(h31, h32, d, e3, new_f2);

    //set new vertex
    new_v->position = (b->position + c->position)/2.0;
    new_v->halfedge() = h11;

    //set all edges
    e1->halfedge() = h11;
    e2->halfedge() = h21;
    e3->halfedge() = h32;

    //set all faces
    f1->halfedge() = h01;
    f2->halfedge() = h02;
    new_f1->halfedge() = h11;
    new_f2->halfedge() = h31;


    e0->isNew = false;
    e1->isNew = false;
    e2->isNew = true;
    e3->isNew = true;
    new_v->isNew = true;


    return new_v;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    /*
    Loop subdivision consists of two basic steps:
      1. Change the mesh topology: split each triangle into four by CONNECTING EDGE MIDPOINTS (sometimes called "4-1
      subdivision" - 1 triangle to 4).
           
      Split every edge of the mesh in any order.
      Flip any new edge that touches a new vertex and an old vertex.
 
      Edge::isNew - flag true for the 'blue' edges (not in one of the original mesh edges)
    */
   
    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.
    for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) // traversing list of vertices in mesh
    {
      // Set all Vertex::isNew to false - since it is the flag true for new vertices (not one of the original mesh vertices)
      v->isNew = false; 

      // Compute new positions for all the vertices using the weightage avg
      int n = v->degree(); // n is the number of neighboring vertices (vertex degree)
      double u = 3.0/(8*n); // constant, 3/8n otherwise
      if (n == 3)
        u = 3.0/16;

      Vector3D neighbor_position_sum(0,0,0); // neighbor_position_sum is the sum of all neighboring vertices' positions
      HalfedgeCIter hc = v->halfedge();
      do {
          neighbor_position_sum += hc->twin()->vertex()->position; //neighboring vertex position added
          hc = hc->twin()->next();
      } while(hc != v->halfedge()); // all halfedges traversed linked to vertex v

      v->newPosition = (1-n*u) * v->position + u * neighbor_position_sum; //Vertex::newPosition can be used as temporary storage for the new position
    }

    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    vector<EdgeIter> original_edges;
    for(EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) { // traversing list of edges in mesh
      e->isNew = false; //  part of original mesh
      // Edge::newPosition can be used to store the position of the vertices that will ultimately be inserted at edge midpoints, position of the vertex that will appear along the old edge after the edge is split
      Vector3D A = e->halfedge()->vertex()->position;
		  Vector3D B = e->halfedge()->twin()->vertex()->position;
		  Vector3D D = e->halfedge()->next()->twin()->vertex()->position;
      Vector3D C = e->halfedge()->twin()->next()->twin()->vertex()->position;
		  e->newPosition = (3/8.0)*(A+B) + (1/8.0)*(C+D); // position for a newly created vertex v that splits an edge AB connecting vertices A and B and is flanked by opposite
      // vertices C and D across the two faces connected to AB
      original_edges.push_back(e);
    }

    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)
    for(EdgeIter e : original_edges) { // traversing list of edges in original mesh only
      // edge split will now return an iterator to the newly inserted vertex 
      VertexIter newV = mesh.splitEdge(e);
      newV->isNew = true; //(i) flag the vertex returned by the split operation as a new vertex
      newV->newPosition = e->newPosition; // copy the newPosition field from the edge being split into the newPosition field of the newly inserted vertex
      
      // (ii) flag each outgoing edge as either being new or part of the original mesh.
      HalfedgeIter newVh = newV->halfedge(); 
      /*
        true /| 
            o-- false
        true \|
      */
      newVh->twin()->next()->edge()->isNew = true; //blue edge on left adjacent face
      newVh->edge()->isNew = false; // the halfedge of this vertex will point along the edge of the original mesh
      //so its isNew should be false
      newVh->next()->next()->edge()->isNew = true; //blue edge on right adjacent face

    }

    // TODO Now flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
      if (e->isNew){ // only for new edges
        bool v0New = e->halfedge()->vertex()->isNew; //get isNew for vertex on current halfedge end
        bool v1New = e->halfedge()->twin()->vertex()->isNew; //opposite vertex
        if (v0New != v1New){ // one of them is true (new), the other is false (old)
          mesh.flipEdge(e); //flip for this specific blue edge
        }
      }
    }

    // TODO Finally, copy the new vertex positions into final Vertex::position. (FOR ORIGINAL VERTICES ONLY)
    for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) { // traversing original list of vertices in mesh
      v->position = v->newPosition;
    }
  }

}
