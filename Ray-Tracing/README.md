![Screenshot](https://drive.google.com/uc?id=1EH15q8AJiSHUhh85o-MNRoZMk9Jnon1C)
 
<h1>Assignment 2: Ray Tracing</h1>
<h2>[Deadline: Monday, 14th Oct 2019]</h2>
 
|Flag and parameters | Description|
|:-----:|------|
|`-s <INT>`  | Number of camera rays per pixel (default=1, should be a power of 2)|
|`-l <INT>`  | Number of samples per area light (default=1)|
|`-t <INT>` | Number of render threads (default=1)|
|`-m <INT>` | Maximum ray depth (default=1)|
|`-f <FILENAME>` | Image (.png) file to save output to in windowless mode|
|`-r <INT> <INT>` | Width and height in pixels of output image (if windowless) or of GUI window|
|`-p <x> <y> <dx> <dy>	` | Used with the -f flag to render a cell|
|`-c <FILENAME>` | Load camera settings file (mainly to set camera position when windowless)|
|`-a <INT> <FLOAT>` | Samples per batch and tolerance for adaptive sampling|
|`-H` | Enable hemisphere sampling for direct lighting|
|`-h` | Print command line help message|


<h3><b>Moving the camera (in edit and BVH mode)</b></h3>

|Command | Action|
|:-----:|------|
|`Rotate`  | Left-click and drag|
|`Translate`  | Right-click and drag|
|`Zoom in and out`  | Scroll|
|`Reset view`  | Spacebar|

<h3><b>Keyboard commands</b></h3>

|Command | Keys|
|:-----:|------|
|`Mesh-edit mode (default)`  | E|
|`BVH visualizer mode`  | V|
|`Descend to left/right child (BVH viz)`  | LEFT/RIGHT|
|`Move up to parent node (BVH viz)`  | UP|
|`Start rendering`  | R|
|`Save a screenshot`  | S|
|`Decrease/increase area light samples`  | - +|
|`Decrease/increase camera rays per pixel`  | [ ]|
|`Decrease/increase maximum ray depth`  | < >|
|`Toggle cell render mode`  | C|
|`Toggle uniform hemisphere sampling`  | H|
|`Dump camera settings to file`  | D|

Cell render mode lets you use your mouse to highlight a region of interest so that you can see quick results in that area when fiddling with per pixel ray count, per light ray count, or ray depth.

<h1>Ray Generation and Scene Intersection</h1>

<h2><b>Task 1: Filling in the sample loop</b></h2>

Fill in PathTracer::raytrace_pixel() in pathtracer.cpp. This function returns a Spectrum corresponding to the integral of the irradiance over this pixel, which you will estimate by averaging over ns_aa samples.

The inputs to this function are integer coordinates in pixel space. You should generate ns_aa random rays through this pixel using camera->generate_ray() (which you will implement in Task 2) and evaluate their radiance with trace_ray().

Notes:

<ul><li>PathTracer owns a gridSampler, which has a method you can use to get random samples in $[0,1]^2$ (see sampler.h/cpp for details).</li>
<li>The width and height of the pixel buffer are stored in sampleBuffer.w and sampleBuffer.h.</li>
<li>You will most likely want to pass a location to the camera that has been scaled down to $[0,1]^2$ coordinates.</li>
<li>When ns_aa == 1, you should generate your ray through the center of the pixel, i.e., $(x+.5,y+.5)$.</li>
<li>Remember to be careful about mixing int and double types here, since the input variables have integer types.</li>

</ul>


<h2><b>Task 2: Generating camera rays</b></h2>

Fill in Camera::generate_ray() in camera.cpp. The input is a 2D point you calculated in Task 1. Generate the corresponding world space ray as depicted in this slide.

The camera has its own coordinate system. In camera space, the camera is positioned at the origin, looks along the $-z$ axis, has the $+y$ axis as image space "up". Given the two field of view angles hFov and vFov, we can define a sensor plane one unit along the view direction with its bottom left and top right corners at
<pre class="prettyprint"><code><span class="typ">Vector3D</span><span class="pun">(-</span><span class="pln">tan</span><span class="pun">(</span><span class="pln">radians</span><span class="pun">(</span><span class="pln">hFov</span><span class="pun">)*.</span><span class="lit">5</span><span class="pun">),</span><span class="pln"> </span><span class="pun">-</span><span class="pln">tan</span><span class="pun">(</span><span class="pln">radians</span><span class="pun">(</span><span class="pln">vFov</span><span class="pun">)*.</span><span class="lit">5</span><span class="pun">),-</span><span class="lit">1</span><span class="pun">)</span><span class="pln">
</span><span class="typ">Vector3D</span><span class="pun">(</span><span class="pln"> tan</span><span class="pun">(</span><span class="pln">radians</span><span class="pun">(</span><span class="pln">hFov</span><span class="pun">)*.</span><span class="lit">5</span><span class="pun">),</span><span class="pln">  tan</span><span class="pun">(</span><span class="pln">radians</span><span class="pun">(</span><span class="pln">vFov</span><span class="pun">)*.</span><span class="lit">5</span><span class="pun">),-</span><span class="lit">1</span><span class="pun">)</span></code></pre>
respectively. Convert the input point to a point on this sensor so that $(0,0)$ maps to the bottom left and $(1,1)$ maps to the top right. This is your ray's direction in camera space. We can convert this vector to world space by applying the transform c2w. This vector becomes our r.d (don't forget to normalize it!!). The r.o parameter is simply the camera's position pos. The ray's minimum and maximum $t$ values should be the nClip and fClip camera parameters.

<b><i>Tasks 1 and 2 will be tricky to debug before implementing part 3, since nothing will show up on your screen!</b></i>

<H2><B>Task 3: Intersecting Triangles</B></H2>

Fill in both Triangle::intersect() methods in triangle.cpp. You are free to use any method you choose, but we recommend using the Moller Trumbore algorithm. Make sure you understand the derivation of the algorithm: here is one reference.

Remember that not every intersection is valid -- the ray has min_t and max_t fields defining the valid range of t values. If t lies outside this range, you should return false. Else, update max_t to be equal to t so that future intersections with farther away primitives will be discarded.

Once you get the ray's $t$-value at the intersection point, you should populate the Intersection *isect structure in the second version of the function as follows:

<ul>
<li><code>t</code> is the ray's <span class="MathJax_Preview" style="color: inherit; display: none;"></span><span class="MathJax" id="MathJax-Element-10-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot;><mi>t</mi></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true"><span class="math" id="MathJax-Span-58" style="width: 0.498em; display: inline-block;"><span style="display: inline-block; position: relative; width: 0.388em; height: 0px; font-size: 124%;"><span style="position: absolute; clip: rect(1.542em 1000.35em 2.477em -1000.02em); top: -2.307em; left: 0em;"><span class="mrow" id="MathJax-Span-59"><span class="mi" id="MathJax-Span-60" style="font-family: MathJax_Math-italic;">t</span></span><span style="display: inline-block; width: 0px; height: 2.312em;"></span></span></span><span style="display: inline-block; overflow: hidden; vertical-align: -0.065em; border-left: 0px solid; width: 0px; height: 0.958em;"></span></span></nobr><span class="MJX_Assistive_MathML" role="presentation"><math xmlns="http://www.w3.org/1998/Math/MathML"><mi>t</mi></math></span></span><script type="math/tex" id="MathJax-Element-10">t</script>-value at the hit point.</li>
<li><code>n</code> is the surface normal at the hit point. Use barycentric coordinates to interpolate between <code>n1, n2, n3</code>, the per-vertex mesh normals.</li>
<li><code>primitive</code> points to the primitive that was hit (use the <code>this</code> pointer).</li>
<li><code>bsdf</code> points to the surface bsdf at the hit point (use <code>get_bsdf()</code>).</li>
</ul>


![11_](https://user-images.githubusercontent.com/31760909/31054385-06c56852-a6cb-11e7-9fe7-c54c7f8f6205.jpg)


<h2><b>Task 4: Intersecting Spheres</b></h2>

Fill in both Sphere::intersect() methods in sphere.cpp. Use the quadratic formula. There is also a helper function Sphere::test() that you might want to implement and use. As with Triangle::intersect(), set r.max_t in both routines and fill in the isect parameters for the second version of the function. For a sphere, the surface normal is a scaled version of the vector pointing from the sphere's center to the hit point.

Now the spheres should appear in dae/sky/CBspheres_lambertian.dae:
![11_1](https://user-images.githubusercontent.com/31760909/31054384-00ae22ce-a6cb-11e7-9245-2cdf06f6afbd.jpg)
Reference: Berkeley course - https://cs184.org/article/12


<h1> Additional info</h1>
<img width="478" alt="screen shot 2017-10-06 at 3 22 10 pm-1" src="https://user-images.githubusercontent.com/31760909/31302636-bdf391fc-ab1c-11e7-845d-3fb4129a60cd.png">


This result was generated using the following command and took 2194.49 seconds to render.

./pathtracer -t 8 -r 480 320 ../dae/sky/CBdragon.dae 

***********************************************************************************************************

<h1>Bounding Volume Hierarchy</h1>


<div class="markdown article-content">
<h2>Task 5: Constructing the BVH</h2>

<p>Implement the function <code>BVHAccel:construct_bvh()</code> inside <em>bvh.cpp</em>. The <code>BVHAccel</code> class itself only contains a <code>BVHNode *root</code>. Each node contains a bounding box <code>bb</code>, left and right children <code>l</code> and <code>r</code>, and a pointer <code>vector&lt;Primitive *&gt; *prims</code> to a list of actual scene primitives. For interior nodes, <code>l</code> and <code>r</code> are non-<code>NULL</code>, and for leaf nodes, <code>prim</code> is non-<code>NULL</code>.</p>

<p>The starter code creates a one-node BVH by storing all nodes directly into a leaf node. You may notice that any <em>.dae</em> files with even mildly complicated geometry take a very long time to "render," even with only the simple normal shading. Even a BVH constructed with simple heuristics will perform much better (taking ray intersection complexity from linear to log, for those who care).</p>

<p>Some important utility functions for you to note:</p>

<ol>
<li><code>Primitive::get_bbox()</code> returns the bounding box of a primitive.</li>
<li><code>BBox::expand()</code> expands a bounding box to include the function argument, which can either be a <code>Vector3D</code> or another <code>BBox</code>.</li>
<li>Check out the <code>Vector3D</code> members variables inside a <code>BBox</code>: <code>min, max, extent</code>, where <code>extent = max-min</code>.</li>
</ol>

<p>We recommend that you first attempt BVH construction with some the following simple (but slightly inefficient) recursive function:</p>

<ol>
<li>Compute the bounding box of the primitives in <code>prims</code> in a loop. </li>
<li>Initialize a new <code>BVHNode</code> with that bounding box.</li>
<li>If there are at most <code>max_leaf_size</code> primitives in the list, this is a leaf node. Allocate a new <code>Vector&lt;Primitive *&gt;</code> for node's primitive list (initialize this with <code>prims</code>) and return the node.</li>
<li>If not, we need to recurse left and right. Pick the axis to recurse on (perhaps the largest dimension of the bounding box's extent).</li>
<li>Calculate the split point you are using on this axis (perhaps the midpoint of the bounding box).</li>
<li>Split all primitives in <code>prims</code> into two new vectors based on whether their bounding box's centroid's coordinate in the chosen axis is less than or greater than the split point. (<code>p-&gt;get_bbox().centroid()</code> is a quick way to get a bounding box centroid for <code>Primitive *p</code>.)</li>
<li>Recurse, assigning the left and right children of this node to be two new calls to <code>construct_bvh()</code> with the two primitive lists you just generated.</li>
<li>Return the node.</li>
</ol>

<p><del>One potential problem to consider: what happens if all the primitives lie on one side of the split point? We will get a segfault because of infinite attempted recursive calls. You need some logic to handle the case where either the left or right primitive vector is empty.</del></p>

<p>We've provided a helpful BVH visualization mode which you can use to debug your implementation. To enter this mode, press <code>V</code>. You can then navigate around the BVH levels using the left, right, and up keys. The BVH viz view looks like this:</p>

<p></p><center>
<img src="http://cs184.eecs.berkeley.edu/cs184_sp16_content/article_images/12_2.jpg" width="600px">
</center><p></p>

<h2><del>Task 6: Intersecting <code>BBox</code> </del></h2>

<p><del>Implement the function <code>BBox::intersect()</code> inside <em>bbox.cpp</em>, using the simple ray-aligned-plane intersection equation <a href="https://cs184.org/lecture/rt/slide_029">here</a> and the ray-aligned-box intersection method <a href="https://cs184.org/lecture/rt/slide_028">here</a>. Note that this function returns an <em>interval</em> of <code>t</code> values for which the ray lies inside the box.</del></p>

<h2>Task 7: Intersecting <code>BVHAccel</code></h2>

<p>Using the previous two parts, implement the two <code>BVHAccel::intersect()</code> routines inside <em>bvh.cpp</em>. The starter code assumes that <em>root</em> is a leaf node and tests the ray against every single primitive in the tree. Your improved method should implement <a href="https://cs184.org/lecture/acceleration/slide_041">this</a> recursive traversal algorithm.</p>

<p>Notes</p>

<ul>
<li>You can safely return <code>false</code> if a ray intersects with a <code>BBox</code> but its <code>t</code> interval has an empty intersection with the ray's interval from <code>min_t</code> to <code>max_t</code>. </li>
<li>In the version with no <code>isect</code> parameter, you can safely return <code>true</code> after a single hit. However, the other version must return the <em>closest</em> hit along the ray, so it needs to check every <code>BBox</code> touched by the ray. </li>
<li>If all primitives update <code>r.max_t</code> correctly in their own intersection routines, you don't need to worry about making sure the ray stores the closer hit in <code>BVHAccel::intersect()</code> since it will be taken care of automatically.</li>
</ul>

<p>Once this part is complete, your intersection routines should be fast enough to render any of our scene files in a matter of seconds (with normal shading only), even ones like <em>dae/meshedit/maxplanck.dae</em> with tens of thousands of triangles:</p>

<p></p><center>
<img src="http://cs184.eecs.berkeley.edu/cs184_sp16_content/article_images/12_.jpg" width="480px">
</center><p></p>

<p>Or <em>dae/sky/CBlucy.dae</em>, with hundreds of thousands of triangles:</p>

<p></p><center>
<img src="http://cs184.eecs.berkeley.edu/cs184_sp16_content/article_images/12_1.jpg" width="480px">
</center><p></p></div>




</div>

</div>  <!-- end of content_container (defined in header.php) -->
</div>  <!-- end main_container (defined in header.php) -->

<hr size="1">
<div class="footer">
<div class="footer_right" style="padding-bottom: 1em;">
Copyright 2017 UC Berkeley ref: https://cs184.org/article/14
</div>
</div>


</body>



