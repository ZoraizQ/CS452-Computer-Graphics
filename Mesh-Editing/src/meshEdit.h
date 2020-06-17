#ifndef CGL_MESH_EDITOR_H
#define CGL_MESH_EDITOR_H

#include <string>
#include <vector>

#include "CGL/CGL.h"

#include "scene.h"
#include "camera.h"
#include "light.h"
#include "mesh.h"
#include "material.h"
#include "halfEdgeMesh.h"
#include "student_code.h"

#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>


using namespace std;

/*
 * Mesh Editor and helpful sub structures.
 *
 * Written by Bryce Summers on 9/14/2015
 *
 * 15-462 CMU Computer Graphics Assignment 2, Fall 2015.
 *
 * Purpose : This file specfies the behavior for a collada viewer.
 *
 * - The scene is rendered using OpenGl.
 * - The user mouse and keyboard controls are mapped to viewing behaviors.
 * - This class symbiotically interacts with the data structures
 *   from the collada parser and the geometric subdivision structures
 *   that the students will use in this glorious assignment!
 *
 *
 * Picking (Feature Selection) Algorithm:
 *
 *  1. Find model space triangular coordinates and MeshFeature location
 *     data.
 *
 *  2. Project the model space triangular coordinates onto the screen with
 *     the current openGl model and projection matrices.
 *
 *  3. Solve 2 dimensional point on screen - triangle inclusion test.
 *     Note: This also gives us barycentric coordinates.
 *
 *  4. Find the triangle closest to the user using the homogenous w
 *     coordinate.
 *
 *  5. Determine whether the user selects the face ABC,
 *     edge AB, BC, CA, or vertice A, B, or C though the
 *     barycentric coordinates.
 *
 *  6. Select feature, show visual cues, and display debugging text.
 */

namespace CGL {

  // This structure is used to store a color pallete for this application.
  struct DrawStyle
  {
     Color halfedgeColor;
     Color vertexColor;
     Color edgeColor;
     Color faceColor;

     float strokeWidth;
     float vertexRadius;
  };

  class MeshNode;

  // A MeshFeature is used to represent an element of the surface selected
  // by the user (e.g., edge, vertex, face).  No matter what kind of feature
  // is selected, the feature is specified relative to some polygon in the
  // mesh.  For instance, if an edge is selected, the MeshFeature will store
  // a pointer to a face containing that edge, as well as the local index of
  // the first (of two) vertices in the polygon corresponding to the edge.
  class MeshFeature
  {
     public:
        // By default, a mesh feature points nowhere!
        MeshFeature( void )
        : element( NULL ), node( NULL ), w( 0. )
        {}

        bool isValid( void ) const
        // Returns true if and only if this feature points
        // to some valid element of some valid mesh.
        {
           return element != NULL &&
                     node != NULL;
        }

        void invalidate( void )
        // Marks this feature as not pointing to anything.
        {
           element = NULL;
              node = NULL;
        }

        HalfedgeElement* element; // which element is selected?
        MeshNode* node; // which mesh node does this element come from?
        double w; // what's the depth value for this selection?
  };


  /* MeshNode class, for use with the MeshEdit class.
   * Intended for Assignment 2 of 15-462 at Carnegie Mellon University.
   *
   * Written by Bryce Summers on 9/14/2015.
   *
   * Computes useful stuff for meshes and renders them to the screen.
   * Also provides functionality for applying visual styles to the mesh
   * that match up with the MeshEdit picking system.
   */
   class MeshNode
   {
      public:
         // Constructor.
         MeshNode( Polymesh& polyMesh )
         {

            // Construct a new array of index lists for the halfedgemesh structure.
            vector< vector<size_t> > polygons;

            // Currently, the halfedge data structure only stores the connectivity of
            // the mesh and the vertex positions; here we just want to copy the
            // connectivity into our local array ("polygons").
            for( PolyListIter p  = polyMesh.polygons.begin();
                              p != polyMesh.polygons.end();
                              p ++ )
            {
               polygons.push_back( p->vertex_indices );
            }

            mesh.build( polygons, polyMesh.vertices );
         }

         // Destructor --- this destructor shouldn't be needed according to the
         // C++ spec, though some compilers seem to complain if there isn't a
         // default destructor explicitly defined.  (May be worth checking up on
         // later!)
         ~MeshNode() {}


         /* Returns the lower and upper corners of the axis aligned
          * bounding box for the mesh
          */
         void getBounds( Vector3D& low, Vector3D& high );

         // Centroid / weighted average point.
         void getCentroid( Vector3D& centroid );


         /* The following functions will be used for extracting model
          * space triangluar data.
          * These functions assume that all polygons are triangles.
          * If a polygon has more than 3 vertices,
          * only the first three vertices will be used.
          * FIXME : Triangulate degenerate n > 3 gons.
          */

         /*
          * populates the given feature structure with data corresponding to
          * mesh feature on the face corresponding to the given lookup structure
          * and bary_centric_coordinates.
          * OUT : feature.
          */
         void fillFeatureStructure(MeshFeature & lookup,
                                   MeshFeature & feature,
                                   Vector3D    & barycentric_coords,
                                   float w);


         // representation of the mesh geometry itself
         HalfedgeMesh mesh;

         // This vector gives us indexed hooks into the half edge structure,
         // which can be used to query information for the debugging messages.
         std::vector<Vertex*> half_edge_vertices;

      private:
         // These thresholds define when a mouse click on given
         // triangle corresponds to selection of a vertex, edge,
         // or face; they are expressed as percent relative to
         // barycentric coordinates. (Note that .3 is about halfway
         // through the triangle.)
         const double low_threshold  = .1;
         const double mid_threshold  = .2;
         const double high_threshold = 1.0 - low_threshold;

   };// class MeshNode.


// The viewer class the manages the viewing and rendering of Collada Files.
class MeshEdit : public Renderer {
 public:

  // --  Inherited public interface functions.
  ~MeshEdit() { }

  virtual void init();
  virtual void render();
  virtual void resize( size_t w, size_t h);

  virtual std::string name();
  virtual std::string info();

  virtual void key_event( char key );
  virtual void cursor_event( float x, float y, unsigned char keys );
  virtual void scroll_event( float offset_x, float offset_y );
  virtual void mouse_button_event( int button, int event );

  void load( Scene* scene );

 private:

  void initializeStyle( void );

  // --  Private Variables.
  Scene* scene;

  vector<MeshNode> meshNodes;

  // View Frustrum Variables.
  float hfov; // FIXME : I would like to specify the view frustrum
              // In terms of the the view_distance.
  float vfov;
  float nearClip;
  float farClip;

  size_t screen_w;
  size_t screen_h;

  GLuint shaderProgID;

  bool shadingMode;
  bool smoothShading;

  // Specify the location of eye and what it is pointing at.
  Vector3D view_focus;

  enum e_up{X_UP, Y_UP, Z_UP};
  e_up up;
  Vector3D camera_angles;

  // The current view distance used in the camera on screen.
  double view_distance;
  // Length of diagonal of bounding box for the mesh.
  // Guranteed to not have the camera occlude with the mes.
  double canonical_view_distance;

  // Minnimum and maximum bounds.
  double min_view_distance;
  double max_view_distance;

  int light_num;


  // -- Helper functions.

  // Initialization functions to get the opengl cooking with oil.
  void init_camera   (Camera& camera     );
  void init_light    (Light& light       );
  void init_polymesh (Polymesh& polymesh );
  void init_material (Material& material );

  // Control functions.
  void update_camera();
  void draw_meshes();
  void draw_controlPoints();
  void renderlights();

  // Resets the camera to the canonical initial view position.
  void reset_camera();

  // Rendering functions.
  void renderMesh   ( HalfedgeMesh& mesh );
  void drawFaces    ( HalfedgeMesh& mesh );
  void drawEdges    ( HalfedgeMesh& mesh );
  void drawVertices ( HalfedgeMesh& mesh );
  void drawHalfedges( HalfedgeMesh& mesh );
  void drawHalfedgeArrow( Halfedge* h );

  // Sets the draw style (colors, edge widths, etc.) for the specified
  // mesh element according to whether it is hovered or selected, using
  // the styles below.
  void setElementStyle( HalfedgeElement* element );

  /*
   * A pleasing Brycian, Kavonian, Keenian, Skyian,
   * and/or Michaelvelian colorstyle pallet.
   */
  DrawStyle defaultStyle; // style for elements that are neither hovered nor selected
  DrawStyle hoverStyle;   // style for element currently under the cursor
  DrawStyle selectStyle;  // style for currently selected element
  Color text_color;

  // -- Mouse input.
  // -- User Input variables.
  bool mouse_rotate;
  float mouse_x, mouse_y;
  Vector3D selected;

  enum e_mouse_button
  {
    LEFT   = MOUSE_BUTTON_LEFT,
    RIGHT  = MOUSE_BUTTON_RIGHT,
    MIDDLE = MOUSE_BUTTON_MIDDLE
  };

  bool left_down;
  bool right_down;
  bool middle_down;

  void mouseP(e_mouse_button b);// Mouse pressed.
  void mouseR(e_mouse_button b);// Mouse Released.
  void mouseD(float x, float y);// Mouse Dragged.
  void mouseM(float x, float y);// Mouse Moved.
  void updateMouseCoordinates(float x, float y);// updates stored mouse_x/y


  // -- Feature Selection.
  MeshFeature hoveredFeature;  // feature currently under the cursor
  MeshFeature selectedFeature; // feature last clicked on by the user

  // Executes the picking algorithm.
  // result stored in 'hover_selection'.
  void findMouseSelection(float x, float y);
  // Copies 'hover_selection' to 'current_selection' on mouse release.
  void enactPotentialSelection();

  /**
   * IN: screen_x_offset -- offset in screen space in x direction.
   *     screen_y_offset -- offset in screen space in y direction.
   * IN / OUT : position -- Position vector in world space.
   */
  void dragPosition(float screen_x_offset,
					float screen_y_offset,
					Vector3D & position);


  inline bool triangleSelectionTest
	(const Vector2D & selectionPoint,
	 Vector4D & A, Vector4D & B, Vector4D & C,
	 float & w,
         Vector3D& barycentricCoordinates );

  inline Vector2D unitCubeToScreenSpace(Vector4D & in);



  // -- Geometric Operations
  // Local operations on current element.
  void flipSelectedEdge( void );
  void splitSelectedEdge( void );
  // Sets up and calls the MeshResampler with the appropiate operation.
  void mesh_up_sample();

  // If a halfedge is selected, advances to the next or twin halfedge.
  void selectNextHalfedge( void );
  void selectTwinHalfedge( void );

  // The canonical resampler used to perform operations on meshes.
  MeshResampler resampler;


  // OSD text manager
  OSDText text_mgr;

  vector<int> messages;

  // -- Debugging strings.
  bool showHUD;
  bool showControlPoints;
  void drawHUD();
  inline void drawString(float x, float y, string str, size_t size, Color c);





double colorpalette[64][3] = {
    { 0.208100000000000 , 0.166300000000000 , 0.529200000000000 },
{ 0.211623809523810 , 0.189780952380952 , 0.577676190476191 },
{ 0.212252380952381 , 0.213771428571429 , 0.626971428571429 },
{ 0.208100000000000 , 0.238600000000000 , 0.677085714285714 },
{ 0.195904761904762 , 0.264457142857143 , 0.727900000000000 },
{ 0.170728571428571 , 0.291938095238095 , 0.779247619047619 },
{ 0.125271428571429 , 0.324242857142857 , 0.830271428571429 },
{ 0.0591333333333334  , 0.359833333333333 , 0.868333333333333 },
{ 0.0116952380952381  , 0.387509523809524 , 0.881957142857143 },
{ 0.00595714285714286 , 0.408614285714286 , 0.882842857142857 },
{ 0.0165142857142857  , 0.426600000000000 , 0.878633333333333 },
{ 0.0328523809523810  , 0.443042857142857 , 0.871957142857143 },
{ 0.0498142857142857  , 0.458571428571429 , 0.864057142857143 },
{ 0.0629333333333333  , 0.473690476190476 , 0.855438095238095 },
{ 0.0722666666666667  , 0.488666666666667 , 0.846700000000000 },
{ 0.0779428571428572  , 0.503985714285714 , 0.838371428571429 },
{ 0.0793476190476190  , 0.520023809523810 , 0.831180952380952 },
{ 0.0749428571428571  , 0.537542857142857 , 0.826271428571429 },
{ 0.0640571428571428  , 0.556985714285714 , 0.823957142857143 },
{ 0.0487714285714286  , 0.577223809523810 , 0.822828571428572 },
{ 0.0343428571428572  , 0.596580952380952 , 0.819852380952381 },
{ 0.0265000000000000  , 0.613700000000000 , 0.813500000000000 },
{ 0.0238904761904762  , 0.628661904761905 , 0.803761904761905 },
{ 0.0230904761904762  , 0.641785714285714 , 0.791266666666667 },
{ 0.0227714285714286  , 0.653485714285714 , 0.776757142857143 },
{ 0.0266619047619048  , 0.664195238095238 , 0.760719047619048 },
{ 0.0383714285714286  , 0.674271428571429 , 0.743552380952381 },
{ 0.0589714285714286  , 0.683757142857143 , 0.725385714285714 },
{ 0.0843000000000000  , 0.692833333333333 , 0.706166666666667 },
{ 0.113295238095238 , 0.701500000000000 , 0.685857142857143 },
{ 0.145271428571429 , 0.709757142857143 , 0.664628571428572 },
{ 0.180133333333333 , 0.717657142857143 , 0.642433333333333 },
{ 0.217828571428571 , 0.725042857142857 , 0.619261904761905 },
{ 0.258642857142857 , 0.731714285714286 , 0.595428571428571 },
{ 0.302171428571429 , 0.737604761904762 , 0.571185714285714 },
{ 0.348166666666667 , 0.742433333333333 , 0.547266666666667 },
{ 0.395257142857143 , 0.745900000000000 , 0.524442857142857 },
{ 0.442009523809524 , 0.748080952380952 , 0.503314285714286 },
{ 0.487123809523809 , 0.749061904761905 , 0.483976190476191 },
{ 0.530028571428571 , 0.749114285714286 , 0.466114285714286 },
{ 0.570857142857143 , 0.748519047619048 , 0.449390476190476 },
{ 0.609852380952381 , 0.747314285714286 , 0.433685714285714 },
{ 0.647300000000000 , 0.745600000000000 , 0.418800000000000 },
{ 0.683419047619048 , 0.743476190476191 , 0.404433333333333 },
{ 0.718409523809524 , 0.741133333333333 , 0.390476190476190 },
{ 0.752485714285714 , 0.738400000000000 , 0.376814285714286 },
{ 0.785842857142857 , 0.735566666666667 , 0.363271428571429 },
{ 0.818504761904762 , 0.732733333333333 , 0.349790476190476 },
{ 0.850657142857143 , 0.729900000000000 , 0.336028571428571 },
{ 0.882433333333333 , 0.727433333333333 , 0.321700000000000 },
{ 0.913933333333333 , 0.725785714285714 , 0.306276190476191 },
{ 0.944957142857143 , 0.726114285714286 , 0.288642857142857 },
{ 0.973895238095238 , 0.731395238095238 , 0.266647619047619 },
{ 0.993771428571429 , 0.745457142857143 , 0.240347619047619 },
{ 0.999042857142857 , 0.765314285714286 , 0.216414285714286 },
{ 0.995533333333333 , 0.786057142857143 , 0.196652380952381 },
{ 0.988000000000000 , 0.806600000000000 , 0.179366666666667 },
{ 0.978857142857143 , 0.827142857142857 , 0.163314285714286 },
{ 0.969700000000000 , 0.848138095238095 , 0.147452380952381 },
{ 0.962585714285714 , 0.870514285714286 , 0.130900000000000 },
{ 0.958871428571429 , 0.894900000000000 , 0.113242857142857 },
{ 0.959823809523810 , 0.921833333333333 , 0.0948380952380953  },
{ 0.966100000000000 , 0.951442857142857 , 0.0755333333333333  },
{ 0.976300000000000 , 0.983100000000000 , 0.0538000000000000  }};

}; // class MeshEdit
} // namespace CGL

#endif // CGL_MESH_EDITOR_H
