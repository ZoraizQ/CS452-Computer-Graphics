#include "drawrend.h"
#include "svg.h"
#include "transforms.h"
#include "CGL/misc.h"
#include <iostream>
#include <sstream>
#include "CGL/lodepng.h"
#include "texture.h"
#include <ctime>
#include <time.h>
#include <cstdio>
#include <ctime>
using namespace std;

namespace CGL {

struct SVG;


DrawRend::~DrawRend( void ) {}

/**
* Initialize the renderer.
* Set default parameters and initialize the viewing transforms for each tab.
*/
void DrawRend::init() {
  gl = true;

  sample_rate = 1;
  left_clicked = false;
  show_zoom = 0;

  svg_to_ndc.resize(svgs.size());
  for (int i = 0; i < svgs.size(); ++i) {
    current_svg = i;
    view_init();
  }
  current_svg = 0;
  psm = P_NEAREST;
  lsm = L_ZERO;

}

/**
* Draw content.
* Simply reposts the framebuffer and the zoom window, if applicable.
*/
void DrawRend::render() {
  draw_pixels();
  if (show_zoom)
    draw_zoom();
}

/**
 * Respond to buffer resize.
 * Resizes the buffers and resets the
 * normalized device coords -> screen coords transform.
 * \param w The new width of the context
 * \param h The new height of the context
 */
void DrawRend::resize( size_t w, size_t h ) {
  width = w; height = h;

  framebuffer.resize(4 * w * h);

  samplebuffer.clear();
  vector<SampleBuffer> samplebuffer_row(width, SampleBuffer(sqrt(sample_rate)));
  for (int i = 0; i < height; ++i)
    samplebuffer.push_back(samplebuffer_row);

  float scale = min(width, height);
  ndc_to_screen(0,0) = scale; ndc_to_screen(0,2) = (width  - scale) / 2;
  ndc_to_screen(1,1) = scale; ndc_to_screen(1,2) = (height - scale) / 2;

  redraw();
}

/**
 * Return a brief description of the renderer.
 * Displays current buffer resolution, sampling method, sampling rate.
 */
static const string level_strings[] = { "level zero", "nearest level", "bilinear level interpolation"};
static const string pixel_strings[] = { "nearest pixel", "bilinear pixel interpolation"};
std::string DrawRend::info() {
  stringstream ss;
  stringstream sample_method;
  sample_method  << pixel_strings[psm];
  ss << "Resolution " << width << " x " << height << ". ";
  ss << "Using " << sample_method.str() << " sampling. ";
  ss << "Supersample rate " << sample_rate << " per pixel. ";
  if(toggle_scanline) ss << "Scan line";
  return ss.str();
}

/**
 * Respond to cursor events.
 * The viewer itself does not really care about the cursor but it will take
 * the GLFW cursor events and forward the ones that matter to  the renderer.
 * The arguments are defined in screen space coordinates ( (0,0) at top
 * left corner of the window and (w,h) at the bottom right corner.
 * \param x the x coordinate of the cursor
 * \param y the y coordinate of the cursor
 */
void DrawRend::cursor_event( float x, float y ) {
  // translate when left mouse button is held down
  if (left_clicked) {
    float dx = (x - cursor_x) / width  * svgs[current_svg]->width;
    float dy = (y - cursor_y) / height * svgs[current_svg]->height;
    move_view(dx,dy,1);
    redraw();
  }

  // register new cursor location
  cursor_x = x;
  cursor_y = y;
}

/**
 * Respond to zoom event.
 * Like cursor events, the viewer itself does not care about the mouse wheel
 * either, but it will take the GLFW wheel events and forward them directly
 * to the renderer.
 * \param offset_x Scroll offset in x direction
 * \param offset_y Scroll offset in y direction
 */
void DrawRend::scroll_event( float offset_x, float offset_y ) {
  if (offset_x || offset_y) {
    float scale = 1 + 0.05 * (offset_x + offset_y);
    scale = std::min(1.5f,std::max(0.5f,scale));
    move_view(0,0,scale);
    redraw();
  }
}

/**
 * Respond to mouse click event.
 * The viewer will always forward mouse click events to the renderer.
 * \param key The key that spawned the event. The mapping between the
 *        key values and the mouse buttons are given by the macros defined
 *        at the top of this file.
 * \param event The type of event. Possible values are 0, 1 and 2, which
 *        corresponds to the events defined in macros.
 * \param mods if any modifier keys are held down at the time of the event
 *        modifiers are defined in macros.
 */
void DrawRend::mouse_event( int key, int event, unsigned char mods ) {
  if (key == MOUSE_LEFT) {
    if (event == EVENT_PRESS)
      left_clicked = true;
    if (event == EVENT_RELEASE)
      left_clicked = false;
  }
}

/**
 * Respond to keyboard event.
 * The viewer will always forward mouse key events to the renderer.
 * \param key The key that spawned the event. ASCII numbers are used for
 *        letter characters. Non-letter keys are selectively supported
 *        and are defined in macros.
 * \param event The type of event. Possible values are 0, 1 and 2, which
 *        corresponds to the events defined in macros.
 * \param mods if any modifier keys are held down at the time of the event
 *        modifiers are defined in macros.
 */
void DrawRend::keyboard_event( int key, int event, unsigned char mods ) {
  if (event != EVENT_PRESS)
    return;

  // tab through the loaded files
  if (key >= '1' && key <= '9' && key-'1' < svgs.size()) {
    current_svg = key - '1';
    redraw();
    return;
  }

  switch( key ) {

    // reset view transformation
    case ' ':
      view_init();
      redraw();
      break;

    // set the sampling rate to 1, 4, 9, or 16
    case '=':
      if (sample_rate < 16) {
        sample_rate = (int)(sqrt(sample_rate)+1)*(sqrt(sample_rate)+1);

        samplebuffer.clear();
        vector<SampleBuffer> samplebuffer_row(width, SampleBuffer(sqrt(sample_rate)));
        for (int i = 0; i < height; ++i)
          samplebuffer.push_back(samplebuffer_row);
        redraw();
      }
      break;
    case '-':
      if (sample_rate > 1) {
        sample_rate = (int)(sqrt(sample_rate)-1)*(sqrt(sample_rate)-1);

        samplebuffer.clear();
        vector<SampleBuffer> samplebuffer_row(width, SampleBuffer(sqrt(sample_rate)));
        for (int i = 0; i < height; ++i)
          samplebuffer.push_back(samplebuffer_row);
        redraw();
      }
      break;

    // save the current buffer to disk
    case 'S':
      write_screenshot();
      break;

    // toggle pixel sampling scheme
    case 'P':
      psm = (PixelSampleMethod)((psm+1)%2);
      redraw();
      break;

    // // toggle level sampling scheme
    // case 'L':
    //   lsm = (LevelSampleMethod)((lsm+1)%3);
    //   redraw();
    //   break;

      // toggle scanline
    case 'L':
      toggle_scanline = !toggle_scanline;
      redraw();
      break;
    // toggle zoom
    case 'Z':
      show_zoom = (show_zoom+1)%2;
      break;

    default:
      return;
  }
}

/**
 * Writes the contents of the pixel buffer to disk as a .png file.
 * The image filename contains the month, date, hour, minute, and second
 * to make sure it is unique and identifiable.
 */
void DrawRend::write_screenshot() {
    redraw();
    if (show_zoom) draw_zoom();

    vector<unsigned char> windowPixels( 4*width*height );
    glReadPixels(0, 0,
                width,
                height,
                GL_RGBA,
                GL_UNSIGNED_BYTE,
                &windowPixels[0] );

    vector<unsigned char> flippedPixels( 4*width*height );
    for (int row = 0; row < height; ++row)
      memcpy(&flippedPixels[row * width * 4], &windowPixels[(height - row - 1) * width * 4], 4*width);

    time_t t = time(nullptr);
    tm *lt = localtime(&t);
    stringstream ss;
    ss << "screenshot_" << lt->tm_mon+1 << "-" << lt->tm_mday << "_"
      << lt->tm_hour << "-" << lt->tm_min << "-" << lt->tm_sec << ".png";
    string file = ss.str();
    cout << "Writing file " << file << "...";
    if (lodepng::encode(file, flippedPixels, width, height))
      cerr << "Could not be written" << endl;
    else
      cout << "Success!" << endl;
}

/**
 * Writes the contents of the framebuffer to disk as a .png file.
 *
 */
void DrawRend::write_framebuffer() {
  if (lodepng::encode("test.png", &framebuffer[0], width, height))
    cerr << "Could not write framebuffer" << endl;
  else
    cerr << "Succesfully wrote framebuffer" << endl;
}


/**
 * Draws the current SVG tab to the screen. Also draws a
 * border around the SVG canvas. Resolves the supersample buffers
 * into the framebuffer before posting the framebuffer pixels to the screen.
 */
void DrawRend::redraw() {


  for (int i = 0; i < samplebuffer.size(); ++i)
    for (int j = 0; j < samplebuffer[i].size(); ++j)
      samplebuffer[i][j].clear();

  // clock time
  std::clock_t start;
  double duration;
  start = std::clock();

  SVG &svg = *svgs[current_svg];
  svg.draw(this, ndc_to_screen*svg_to_ndc[current_svg]);

  // draw canvas outline
  Vector2D a = ndc_to_screen*svg_to_ndc[current_svg]*(Vector2D(    0    ,     0    )); a.x--; a.y++;
  Vector2D b = ndc_to_screen*svg_to_ndc[current_svg]*(Vector2D(svg.width,     0    )); b.x++; b.y++;
  Vector2D c = ndc_to_screen*svg_to_ndc[current_svg]*(Vector2D(    0    ,svg.height)); c.x--; c.y--;
  Vector2D d = ndc_to_screen*svg_to_ndc[current_svg]*(Vector2D(svg.width,svg.height)); d.x++; d.y--;

  rasterize_line(a.x, a.y, b.x, b.y, Color::Black);
  rasterize_line(a.x, a.y, c.x, c.y, Color::Black);
  rasterize_line(d.x, d.y, b.x, b.y, Color::Black);
  rasterize_line(d.x, d.y, c.x, c.y, Color::Black);

  resolve();
  // end clock time
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  std::cout<<"\ntime: "<< duration;

  if (gl)
    draw_pixels();
}

/**
 * OpenGL boilerplate to put an array of RGBA pixels on the screen.
 */
void DrawRend::draw_pixels() {
  const unsigned char *pixels = &framebuffer[0];
  // copy pixels to the screen
  glPushAttrib( GL_VIEWPORT_BIT );
  glViewport(0, 0, width, height);

  glMatrixMode( GL_PROJECTION );
  glPushMatrix();
  glLoadIdentity();
  glOrtho( 0, width, 0, height, 0, 0 );

  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glLoadIdentity();
  glTranslatef( -1, 1, 0 );

  glRasterPos2f(0, 0);
  glPixelZoom( 1.0, -1.0 );
  glDrawPixels( width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels );
  glPixelZoom( 1.0, 1.0 );

  glPopAttrib();
  glMatrixMode( GL_PROJECTION ); glPopMatrix();
  glMatrixMode( GL_MODELVIEW  ); glPopMatrix();
}

/**
 * Reads off the pixels that should be in the zoom window, and
 * generates a pixel array with the zoomed view.
 */
void DrawRend::draw_zoom() {

  // size (in pixels) of region of interest
  size_t regionSize = 32;

  // relative size of zoom window
  size_t zoomFactor = 16;

  // compute zoom factor---the zoom window should never cover
  // more than 40% of the framebuffer, horizontally or vertically
  size_t bufferSize = min( width, height );
  if( regionSize*zoomFactor > bufferSize * 0.4) {
    zoomFactor = (bufferSize * 0.4 )/regionSize;
  }
  size_t zoomSize = regionSize * zoomFactor;

  // adjust the cursor coordinates so that the region of
  // interest never goes outside the bounds of the framebuffer
  size_t cX = max( regionSize/2, min( width-regionSize/2-1, (size_t) cursor_x ));
  size_t cY = max( regionSize/2, min( height-regionSize/2-1, height - (size_t) cursor_y ));

  // grab pixels from the region of interest
  vector<unsigned char> windowPixels( 3*regionSize*regionSize );
  glReadPixels( cX - regionSize/2,
                cY - regionSize/2 + 1, // meh
                regionSize,
                regionSize,
                GL_RGB,
                GL_UNSIGNED_BYTE,
                &windowPixels[0] );

  // upsample by the zoom factor, highlighting pixel boundaries
  vector<unsigned char> zoomPixels( 3*zoomSize*zoomSize );
  unsigned char* wp = &windowPixels[0];
  // outer loop over pixels in region of interest
  for( int y = 0; y < regionSize; y++ ) {
   int y0 = y*zoomFactor;
   for( int x = 0; x < regionSize; x++ ) {
      int x0 = x*zoomFactor;
      unsigned char* zp = &zoomPixels[ ( x0 + y0*zoomSize )*3 ];
      // inner loop over upsampled block
      for( int j = 0; j < zoomFactor; j++ ) {
        for( int i = 0; i < zoomFactor; i++ ) {
          for( int k = 0; k < 3; k++ ) {
            // highlight pixel boundaries
            if( i == 0 || j == 0 ) {
              const float s = .3;
              zp[k] = (int)( (1.-2.*s)*wp[k] + s*255. );
            } else {
              zp[k] = wp[k];
            }
          }
          zp += 3;
        }
        zp += 3*( zoomSize - zoomFactor );
      }
      wp += 3;
    }
  }

  // copy pixels to the screen using OpenGL
  glMatrixMode( GL_PROJECTION ); glPushMatrix(); glLoadIdentity(); glOrtho( 0, width, 0, height, 0.01, 1000. );
  glMatrixMode( GL_MODELVIEW  ); glPushMatrix(); glLoadIdentity(); glTranslated( 0., 0., -1. );

  glRasterPos2i( width-zoomSize, height-zoomSize );
  glDrawPixels( zoomSize, zoomSize, GL_RGB, GL_UNSIGNED_BYTE, &zoomPixels[0] );
  glMatrixMode( GL_PROJECTION ); glPopMatrix();
  glMatrixMode( GL_MODELVIEW ); glPopMatrix();

}

/**
 * Initializes the default viewport to center and reasonably zoom the SVG
 * with a bit of margin.
 */
void DrawRend::view_init() {
  float w = svgs[current_svg]->width, h = svgs[current_svg]->height;
  set_view(w/2, h/2, 1.2 * std::max(w,h) / 2);
}

/**
 * Sets the viewing transform matrix corresponding to a view centered at
 * (x,y) in SVG space, extending 'span' units in all four directions.
 * This transform maps to 'normalized device coordinates' (ndc), where the window
 * corresponds to the [0,1]^2 rectangle.
 */
void DrawRend::set_view(float x, float y, float span) {
  svg_to_ndc[current_svg] = Matrix3x3(1,0,-x+span,  0,1,-y+span,  0,0,2*span);
}

/**
 * Recovers the previous viewing center and span from the viewing matrix,
 * then shifts and zooms the viewing window by setting a new view matrix.
 */
void DrawRend::move_view(float dx, float dy, float zoom) {
  Matrix3x3& m = svg_to_ndc[current_svg];
  float span = m(2,2)/2.;
  float x = span - m(0,2), y = span - m(1,2);
  set_view(x - dx, y - dy, span * zoom);
}

// Rasterize a point.
void DrawRend::rasterize_point( float x, float y, Color color ) {
  // fill in the nearest pixel
  int sx = (int) floor(x);
  int sy = (int) floor(y);

  // check bounds
  if ( sx < 0 || sx >= width ) return;
  if ( sy < 0 || sy >= height ) return;

  samplebuffer[sy][sx].fill_pixel(color);
  return;

}

// Rasterize a line.
void DrawRend::rasterize_line( float x0, float y0,
                     float x1, float y1,
                     Color color) {
  if (x0 > x1) {
    swap(x0,x1); swap(y0,y1);
  }

  float pt[] = {x0,y0};
  float m = (y1-y0)/(x1-x0);
  float dpt[] = {1,m};
  int steep = abs(m) > 1;
  if (steep) {
    dpt[0] = x1==x0 ? 0 : 1/abs(m);
    dpt[1] = x1==x0 ? (y1-y0)/abs(y1-y0) : m/abs(m);
  }

  while (floor(pt[0]) <= floor(x1) && abs(pt[1]-y0) <= abs(y1-y0)) {
    rasterize_point(pt[0],pt[1],color);
    pt[0]+=dpt[0]; pt[1]+=dpt[1];
  }
}

// toggle between scanline and classical rasterization
void DrawRend::rasterize_fig( float x0, float y0,
                         float x1, float y1,
                         float x2, float y2,
                         Color color, Triangle *tri) {
                           if(!toggle_scanline) rasterize_triangle(x0,y0,x1,y1,x2,y2,color,tri);
                           else rasterize_scanline(x0,y0,x1,y1,x2,y2,color,tri);
                         }

bool isInside(float Px, float Py, float Ax, float Ay, float Bx, float By, float Cx, float Cy) { //had to rename this since inside function already existed in triangulation.cpp
  float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;  
  //x,y vectors for edge AB, BC, CA or revese ordering AC, CB, BA
  ax = Cx - Bx;  ay = Cy - By;
  bx = Ax - Cx;  by = Ay - Cy;
  cx = Bx - Ax;  cy = By - Ay;
  apx= Px - Ax;  apy= Py - Ay;
  bpx= Px - Bx;  bpy= Py - By;
  cpx= Px - Cx;  cpy= Py - Cy;
  
  //result edge function on point being positive on the second edge (common in both tests for orderings)
  bool ep2 = (bpx * ay - bpy * ax >= 0.0f);
  
  // considering both clockwise and anticlockwise ordering/winding of vertices
  return (apx * cy - apy * cx >= 0.0f) == ep2 && ep2 == (cpx * by - cpy * bx >= 0.0f); //either all three are false or all three are true
  // return 
  // |
  // ((cpx * -ay - cpy * -ax >= 0.0f) & 
  // (bpx * -cy - bpy * -cx >= 0.0f) &
  // (apx * -by - apy * -bx >= 0.0f));
};

// Rasterize a triangle.
void DrawRend::rasterize_triangle( float x0, float y0,
                        float x1, float y1,
                        float x2, float y2,
                        Color color, Triangle *tri) {  
  // Part 1: Fill in this function with basic triangle rasterization code.
  //         Hint: Implement fill_color() function first so that you can see 
  //         rasterized points and lines, then start rasterizing triangles. -- Done
  //         Use something like this:
  //             samplebuffer[row][column].fill_pixel(color);
  // Part 2: Add supersampling.
  //         You need to write color to each sub-pixel by yourself,
  //         instead of using the fill_pixel() function.
  //         Hint: Use the fill_color() function like this:
  //             samplebuffer[row][column].fill_color(sub_row, sub_column, color);
  //         You also need to implement get_pixel_color() function to support supersampling.
  
  // given the 3 vertices of the triangle calculate the 2D minimum bounding box
  uint x_min, x_max, y_min, y_max, y, x, by, bx; //Floating Point Unit in modern CPUs
  x_min = min(x0, min(x1, x2));
  x_max = max(x0, max(x1, x2));
  y_min = min(y0, min(y1, y2));
  y_max = max(y0, max(y1, y2));

  //cout << x0 << " " << x1 << " " << x2 << " " << y0 << " " << y1 << " " << y2 << '\n';
  //cout << "xmin: " << x_min << " xmax:" << x_max << " ymin:" << y_min << " ymax: " << y_max << "\n\n";
  
  uint max_side = sqrt(sample_rate); 
  float splen = 1.0f/max_side; // length of sub-pixel
  float halfsplen = splen/2.0f; // length of sub-pixel
  float spx, spy; //coordinates of center of subpixel 
  uint bsize = 8; //block size, can be varied       
  //ATTEMPT TO SPEED UP RASTERIZATION, NOT CHECKING EVERY SAMPLE IN THE BOUNDING BOX NOW

  for (by = y_min; by <= y_max; by += bsize){ // testing in blocks of 8
    for (bx = x_min; bx <= x_max; bx += bsize){ //bx,by block coordinates (top-left)
      // test all 4 corners of the block
      bool c1Inside = isInside(bx,by, x2, y2, x1, y1, x0, y0),
        c2Inside = isInside(bx,by+bsize-1, x2, y2, x1, y1, x0, y0),
        c3Inside = isInside(bx+bsize-1,by, x2, y2, x1, y1, x0, y0),
        c4Inside = isInside(bx+bsize-1,by+bsize-1, x2, y2, x1, y1, x0, y0);
      
      if (c1Inside && c2Inside && c3Inside && c4Inside){  // if all 4 corners of the block are inside
        // fill the whole thing without point-in-triangle checks or supersampling, fill_pixel used
        for (uint j = by; j < by+bsize; j++){
          for (uint i = bx; i < bx+bsize; i++){
            samplebuffer[j][i].fill_pixel(color); //no need to check for these samples
          }
        }
      }
      else{
        for (y = by; y < by+bsize; y++){
          for (x = bx; x < bx+bsize; x++){
              //assuming y and x are at the top-left of the pixel, 1x1 unit
              for (uint sub_row = 0; sub_row < max_side; sub_row++){ // e.g. if max_side = 2, then sub_row 0,1
                for (uint sub_column = 0; sub_column < max_side; sub_column++){ // so 4 pixels at indices [0][0], [0][1], [1][0], [1][1]
                  spy = y + sub_row * splen + halfsplen; // if x = 0, then spx = 0.25, 0.75 for sample_rate = 4
                  spx = x + sub_column * splen + halfsplen;
                  
                  // check if the sub-pixel is inside the triangle
                  if (isInside(spx,spy, x2, y2, x1, y1, x0, y0)) //Each samplebuffer instance stores one pixel
                    samplebuffer[y][x].fill_color(sub_row, sub_column, color); //only fill color (store pixel color in sample buffer) if the sub-pixel is inside the triangle
                }  
              }
          
          }  
        }
      }
    }  
  }
  
  // Part 4: Add barycentric coordinates and use tri->color() for shading when available.
  //
  // Part 5: Fill in the SampleParams struct and pass it to the tri->color function.
  //         Hint: you may use the tri->color() function, and declare SampleParams as so:
  //              SampleParams sp = SampleParams();
  //              sp.psm = psm;
  //              samplebuffer[row][column].fill_color(row, column, tri->color(x0,y0,x1,y1,x2,y2,row,column,sp));
}


void DrawRend::rasterize_scanline( float x0, float y0,
                        float x11, float y1,
                        float x22, float y2,
                        Color color, Triangle *tri) {
  //Part 6: Implement scan line. The process of passing colors should be the same.
  vector<vector<float>> activeEdgeTable;
  vector<vector<float>> edgeTable;

}
}
