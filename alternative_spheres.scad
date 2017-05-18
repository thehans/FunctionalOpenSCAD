use <functional.scad>
use <subdivision.scad>

translate([-40,0,0]) poly3d(sphere2(r=9,$fn=80));
translate([-20,0,0]) poly3d(sphere(r=9,$fn=80));
translate([0,0,0]) poly3d(normalized_cube(r=9,div_count=20));
translate([20,0,0]) poly3d(spherified_cube(9,div_count=20));
translate([40,0,0]) poly3d(icosahedron(9,n=4));


// This sphere is simpler to code but slightly different geometry from default OpenSCAD style (the poles come to a point)
function sphere2(r=1, d) = 
  let(R = d == undef ? r : d/2)
  rotate_extrude(poly=arc(r=R, angle=180,offsetAngle=-90));

function normalized_cube(r=1,div_count=12,d) = 
  let(
    R = d == undef ? r : d/2,
    origin=[0,0,1],
    right = [1,0,0],
    up = [0,1,0],
    div2 = div_count/2,
    face_points = [
      for (j = [0:div_count], 
           i = [0:div_count])
        let(
          face_point = origin + 2.0 * (right * (i-div2) + up * (div2-j)) / div_count
        )
        R*unit(face_point)
    ],
    lface = len(face_points),
    points = concat(
      face_points,
      rotate([90,0,0],poly=face_points),
      rotate([-90,0,0],poly=face_points),
      rotate([180,0,0],poly=face_points),
      rotate([0,-90,0],poly=face_points),
      rotate([0,90,0],poly=face_points)
    ), 
    faces = [
      for (f = [0:6-1],
           j = [0:div_count-1], 
           i = [0:div_count-1]) 
        let (
          i0=f*lface+j*(div_count+1)+i,
          i1=i0+1,
          i2=i0+(div_count+1),
          i3=i2+1
          //tri1 = 
        )
        [[i0,i1,i3],
         [i0,i3,i2]]
    ] // groups of two need to be concat'd
  )
  [points,flatten(faces)];


function spherified_cube(r=1,origin=[0,0,1],div_count=12,d) = 
  let(
    R = d == undef ? r : d/2,
    right = [1,0,0],
    up = [0,1,0],
    div2 = div_count/2,
    face_points = [
      for (j = [0:div_count], 
           i = [0:div_count])
        let(
          p = origin + 2.0 * (right * (i-div2) + up * (div2 - j)) / div_count,
          p2 = [p.x*p.x,p.y*p.y,p.z*p.z],
          rx = p.x * sqrt(1.0 - 0.5 * (p2.y + p2.z) + p2.y*p2.z/3.0),
          ry = p.y * sqrt(1.0 - 0.5 * (p2.z + p2.x) + p2.z*p2.x/3.0),
          rz = p.z * sqrt(1.0 - 0.5 * (p2.x + p2.y) + p2.x*p2.y/3.0)
        )
        R*[rx,ry,rz]
    ],
    lface = len(face_points),
    points = concat(
      face_points,
      rotate([90,0,0],poly=face_points),
      rotate([-90,0,0],poly=face_points),
      rotate([180,0,0],poly=face_points),
      rotate([0,-90,0],poly=face_points),
      rotate([0,90,0],poly=face_points)
    ), 
    faces = [
      for (f = [0:6-1],
           j = [0:div_count-1], 
           i = [0:div_count-1]) 
        let (
          i0=f*lface+j*(div_count+1)+i,
          i1=i0+1,
          i2=i0+(div_count+1),
          i3=i2+1
          //tri1 = 
        )
        [[i0,i1,i3],
         [i0,i3,i2]]
    ] // groups of two need to be concat'd
  )
  [points,flatten(faces)];


// n = subdivision iterations (number of faces = 20 * 4^n)
// points constructed from golden rectangles as shown here https://commons.wikimedia.org/wiki/File:Icosahedron-golden-rectangles.svg
function icosahedron(r=1,d,n=0) =
  let(
    R = d == undef ? r : d/2,
    phi = (1+sqrt(5))/2,
    magnitude = norm([1,phi]),
    a = 1/magnitude,
    b = phi/magnitude,
    points = [
      [-a,b,0],[a,b,0],[a,-b,0],[-a,-b,0], // golden rectangle in XY plane
      [-b,0,a],[b,0,a],[b,0,-a],[-b,0,-a], // golden rectangle in XZ plane
      [0,-a,b],[0,a,b],[0,a,-b],[0,-a,-b]  // golden rectangle in YZ plane
    ],
    faces = [
      [0,9,4],[4,9,8],[8,9,5],[5,9,1],[1,9,0],
      [2,11,3],[3,11,7],[7,11,10],[10,11,6],[6,11,2],
      [8,5,2],[2,5,6],[6,5,1],[6,1,10],[10,1,0],[10,0,7],[7,0,4],[7,4,3],[3,4,8],[8,2,3]
    ],
    poly = n==0 ? [points,faces] : subdivide_faces(n=n, poly=[points,faces])
  )
  [[for (p = poly[0]) R*unit(p)],poly[1]];
