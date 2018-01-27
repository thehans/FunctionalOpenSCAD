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

The functions in this library have the same names as the builtin modules, 
but that doesn't mean they are overriden.  The OpenSCAD language distinguishes between 
function calls and module calls based on syntax and context.
Functions always return a result:
```
poly = square([1,2]);  // result assigned to variable
poly2d(square([1,2])); // result passed as parameter to a module
```
In the code above, calls to `square` are expected to return a value, so OpenSCAD knows to call the `square` function from our library

Modules can't be passed as parameters, and can't be assigned to variables.  Module calls are valid statements by themselves:
```
square();
```
Since the call to `square` above is not stored in a variable or passed as parameter, OpenSCAD knows to use the builtin `square` module.

## The `poly` "datatype"

The functions in this library operate on geometric data in the form of a `poly` (which can represent either a polygon or polyhedron).
Functions for geometric primitives always return a `poly`, and functions that operation on existing shapes (eg transformations) take `poly` as the last parameter.
 * `poly=[points, paths]` for 2D shapes. Same format as used by 
   [`polygon`](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#polygon)
 * `poly=[points, faces]` for 3D shapes.  Same format as used by 
   [`polyhedron`](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#polyhedron)
 * In some cases poly may also be simply a list of points in `poly=points` (often for representing a "path" or part of one)

Additionally, a poly may represent a "poly vector" (list of polys), which most functions will also accept:
 * `poly = [ [points, faces], [points2, faces2], ...]`
 * `poly = [ [points, paths], [points2, paths2], ...]`

In OpenSCAD, functions can not have or interact with 
[`children`](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/User-Defined_Functions_and_Modules#Children) 
in the way that OpenSCAD modules do.  Therefore we must pass our `poly` data as the last parameter for all Transformation functions, etc.

## API Reference for functional.scad

### Copies of Builtin Modules as Functions
All parameters are intended to behave identical to OpenSCAD builtins, however some have added features as *optional* parameters.  Any parameters not part of OpenSCAD builtins are marked in **bold**.
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
 #### Extra Functions
   * arc(r=1, angle=360, offsetAngle=0, c=[0,0], center=false, internal=false)
   * bounds(poly)
   * invert(poly)
   * signed_area(points)
   * signed_volume(poly)
 #### Additonal Modules 
   * poly3d(poly)
   * poly2d(poly)
   * showPoints(poly, r=0.1)
   
### OpenSCAD Modules not (yet) implemented as Functions
 #### 3D to 2D
   * projection
 #### Transform
   * offset
   * minkowski
   * hull
 #### Booleans
   * union
   * difference
   * intersection
