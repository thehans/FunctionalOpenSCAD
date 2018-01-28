# Functional OpenSCAD
Implementing OpenSCAD in OpenSCAD

## Introduction

This library is an attempt to re-implement OpenSCAD's builtin modules 
as functions which operate directly on vertex data.

Having access to vertex, path, and face geometry data makes it possible
to extend the functionality of OpenSCAD from within a user script.

For example, this library includes function `bounds(poly)` which returns `[[minX,minY,minZ],[maxX,maxY,maxZ]]` 
representing the max and min coordinates of all points that make up a shape or list of shapes
This sort of calculation is not possible when using builtin modules because their vertex data is not accessible.

## Functions vs Modules

To understand what this library is doing, its important to note the distinction between **functions** and **modules** in OpenSCAD.  See the relevant [OpenSCAD documentation](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#Chapter_7_--_User-Defined_Functions_and_Modules)
if you need a introduction / refresher to these concepts.

The **functions** in this library have the same names as the builtin OpenSCAD **modules** that they emulate, 
but that doesn't mean the builtins are overriden.  The OpenSCAD language keeps separate namespaces for **functions** and **modules**, and distinguishes between the two based on syntax and context.

**Functions** always return a value result:
```
poly = square([1,2]);  // result assigned to variable
poly2d(square([1,2])); // result passed as parameter to a module
```
In the code above, calls to `square` are expected to return a value, so OpenSCAD knows to call the `square` **function** from our library

**Modules** can't be passed as parameters, and can't be assigned to variables.  **Module** calls are valid statements by themselves:
```
square();
```
Since the call to `square` above is not stored in a variable or passed as parameter, OpenSCAD knows to use the builtin `square` **module**.

## The `poly` "datatype"

The functions in this library operate on geometric data in a special format which we call a `poly`.  It is called this because it can represent either a *poly*gon or *poly*hedron.  I use quotations around "datatype" because OpenSCAD does not have user-defined types.  It's just some nested lists(or "[vectors](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#Vectors)") that we can interpret as shapes or vertices.
Functions for the geometric primitives always return a `poly`.  Functions that operation on existing shapes (eg transformations) always take `poly` as the last parameter, *and* return a `poly`.  

The formats supported are:
 * `poly=[points, paths]` for 2D shapes. Same format as used by 
   [`polygon`](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#polygon)
 * `poly=[points, faces]` for 3D shapes.  Same format as used by 
   [`polyhedron`](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#polyhedron)
 * `poly=points` just a list of points.  Depending on how it is used, it may represent a 2D polygon, or some partial path that will later be concatenated with other paths to form a whole polygon, etc. 

Additionally, a poly may represent a "poly vector" (list of polys), which most functions will also accept:
 * `poly = [ [points, faces], [points2, faces2], ...]`
 * `poly = [ [points, paths], [points2, paths2], ...]`

In OpenSCAD, **functions** can not have or interact with 
[`children`](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/User-Defined_Functions_and_Modules#Children) 
in the way that OpenSCAD **modules** do.  Therefore we must pass our `poly` data as the last parameter for all Transformation **functions**, etc.

## API Reference 

### Library Files

<dl>

<dt><a href="#functional">functional.scad</a></dt>
<dd>The core of FunctionalOpenSCAD.  All functions that implement OpenSCAD builtins are contained in this file, plus a few utilities and extras</dt>

<dt><a href="#alternative_spheres">alternative_spheres.scad</a></dt>
<dd>Alternative implementations of spherical geometries, using different methods of tesselation, (eg. subdivided icosahedron).</dd>

<dt><a href="#double_fillet">double_fillet.scad</a></dt>
<dd>Provides double_fillet function which generates a smooth transition between two parallel surfaces.</dd>

<dt><a href="#planes">planes.scad</a></dt>
<dd>Functions relating to geometric planes.</dd>

<dt><a href="#subdivision">subdivision.scad</a></dt>
<dd>Provides subdivision function which splits each triangle in a 3D poly into 4 smaller triangles by adding midpoints.  Included by alternatives_spheres.scad</dd>

<dt><a href="#webbing">webbing.scad</a></dt>
<dd>Implements "webbing" function which is a way to connect two separate circles with a smooth transition to a thin section between them.</dd>

</dl>

<a name="functional"></a>
## functional.scad

All **functions** here are intended to behave identically to their OpenSCAD builtin counterparts, however some may have additional parameters.  The already mentioned `poly` parameter is **always required** when present.  Any other parameters have been added as convenience features to enhance default functionality, and are completely optional. 
All parameters not part of OpenSCAD builtins are marked in **bold** to distinguish them.

### OpenSCAD Builtin Modules Implemented as Functions
 
 #### 2D Primitives
   * square(size=1, center=false, **r=0**)
   * circle(r=1, **c=[0,0]**, **internal=false**, d)
 #### 3D Primitives
   * cube(size=1, center=false)
   * sphere(r=1, d)
   * cylinder(h=1, r1, r2, center=false, r, d, d1, d2)
 #### 2D to 3D
   * linear_extrude(height=100, center=false, convexity=1, twist=0, slices, scale=1.0, **poly**)
   * rotate_extrude(angle=360, **offsetAngle=0**, **center=false**, **v_offset=0**, **i_offset=0**, **poly**)
 #### Transforms
   * scale(v, **poly**)
   * resize(newsize, **poly**) 
   * rotate(a, v, **poly**)
   * translate(v, **poly**)
   * mirror(normal, **poly**) 
   * multmatrix(M, **poly**)
   
### Functional Extras (no equivalent OpenSCAD Builtin)
 #### Extra Functions
   * arc(r=1, angle=360, offsetAngle=0, c=[0,0], center=false, internal=false)
   * bounds(poly)
   * invert(poly)
   * signed_area(points)
   * signed_volume(poly)
 #### Extra Modules 
   * poly3d(poly)
   * poly2d(poly)
   * showPoints(poly, r=0.1)

### OpenSCAD Builtin Modules NOT (yet) Implemented as Functions
 #### 3D to 2D
   * [projection](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#projection)
 #### Transform
   * [offset](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#offset)
   * [minkowski](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#minkowski)
   * [hull](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#hull)
 #### Booleans
   * [union](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#union)
   * [difference](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#difference)
   * [intersection](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#intersection)

<a name="alternative_spheres"></a>
## alternative_spheres.scad
<dl>
  <dt>sphere2(r=1, d)</dt>
  <dd>Simplest sphere implementation, where poles come to points(very slightly different geometry from builtin)</dd>
  <dt>normalized_cube(r=1,div_count=12,d)</dt>
  <dd>Sphere from a simple cube mapping</dd>
  <dt>spherified_cube(r=1,origin=[0,0,1],div_count=12,d)</dt>
  <dd>Sphere from a more balanced cube mapping</dd>
  <dt>icosahedron(r=1,d,n=0)</dt>
  <dd>Subdivided icosahedron mapping (geodesic sphere).

* n = subdivision iterations. (number of faces = 20 * 4^n)
  </dd>
</dl>

<a name="double_fillet"></a>
## double_fillet.scad
<dl>
  <dt>double_fillet(h=1, r1=1, r2=1, xoffset1=0, xoffset2=0, closed=true)</dt>
  <dd>Double Fillet generates a path that is a smooth transition between two parallel surfaces

* h is the vertical distance between surfaces, negative height will mirror about the vertical axis
* r1 and r2 are the first and second fillet radius traversing away from origin
* xoffset1 distance from origin where first radius begins ( should be >= 0 )
* xoffset2 distance from edge of first radius to the start of second radius.  0 value makes straight wall, < 0 makes overhang
* closed = true will return a closed polygon ready for extrusion, while cloesd == false returns a just the curved vertex path that can be use as part of a larger path
  </dd>
</dl>


<a name="planes"></a>
## planes.scad
<dl>   
<dt>function planeFromPoints(a, b, c)</dt>
<dd>Create a plane from any 3 points.</dd>

<dt>function planeFromFace(poly, iface, lasti=2)</dt>
<dd>Find the plane tangent to a specific polyhedron face.</dd>

<dt>module showPlane(plane, size=[100,100,0.05])</dt>
<dd>Visualize a plane as a finite thin cube.</dd>

<dt>function splitPolygonByPlane(plane, polyh, iface)</dt>
<dd>Atttempted port of OpenJSCAD / csg.js "splitPolygonByPlane" function, prerequisit for further development of boolean operations, projection, etc.

returns [front, coplanarFront, coplanarBack, back] each containing a list of points, or nan

**INCORRECT IMPLEMENTATION, NEEDS DEBUGGING**
</dd>
  
</dl>


<a name="subdivision"></a>
## subdivision.scad
<dl>
  <dt>subdivide_faces(n=1, poly)</dt>
  <dd>Subdivide each triangle in `poly` into 4 smaller triangles.  Additional points are created but the volume remains unchanged by this function.  

* n = subdivision iterations. (number of faces = 20 * 4^n)   
   </dd>
</dl>


<a name="webbing"></a>
## webbing.scad
<dl>
  <dt>webbing(d1, d2, c2, th, r3)</dt>
  <dd>Draw two circles connected by a continuous curve that necks down to a minimum thickness th.

* d1 = diameter of 1st circle
* d2 = diameter of 2nd circle
* c2 = an [x,y] translation pair for 2nd circle
* th = minimum thickness of webbing between circles
* r3 = optionally override the radius calculated for th 
</dl>
