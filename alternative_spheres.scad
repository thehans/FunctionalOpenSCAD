use <functional.scad>
use <subdivision.scad>


//*
// Visual sampler of various sphere approximations from subdivided platonic solids
// Different geometries are spaced to be roughly similar in complexity.
for(n=[2:1:50]) {
  translate([20*n, 0,40]) poly3d(sphere(r=9,$fn=n));  // OpenSCAD-equivalent UV sphere
  translate([20*n, 0, 0]) poly3d(sphere2(r=9,$fn=n)); // UV sphere with vertex at poles
  if (n%2==0) translate([20*n, 40, 0]) poly3d(sphere_subdiv(r=9,divs=ceil(n/2), poly=TETRAHEDRON()));
  if (n%4==0) translate([20*n, 80, 0]) poly3d(sphere_subdiv(r=9,divs=ceil(n/4), poly=CUBE()));
  if (n%4==0) translate([20*n, 80,40]) poly3d(spherified_cube(r=9,div_count=ceil(n/4)));
  if (n%4==0) translate([20*n,120, 0]) poly3d(sphere_subdiv(r=9,divs=ceil(n/4), poly=OCTAHEDRON()));
  if (n%5==0) translate([20*n,160, 0]) poly3d(sphere_subdiv(r=9,divs=ceil(n/5), poly=ICOSAHEDRON()));
  if (n%10==0) translate([20*n,200, 0]) poly3d(sphere_subdiv(r=9,divs=ceil(n/10), poly=DODECAHEDRON()));
}
//*/


/*
// Visualize volumetric accuracy (triangle efficiency) for different sphere types.
// Chart Axes [x, y] = [triangle_count, 10 / error]
//   (error = 1 - poly_volume / actual_volume)

V = 4/3*PI; // Actual unit sphere volume
N = 100;    // max complexity ($fn) for sphere calculations, roughly estimated for non standard spheres
fn = 16; // chart display circle resolution
R = 25; // chart display circle radius

color(1/2*[1,1,1]) for(p=[for(n=[3:1:N]) let(poly=sphere(r=1, $fn=n))
  [n*(2*ceil(n/2)-2)+2*(n-2), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);
color(2/2*[1,1,1]) for(p=[for(n=[3:1:N]) let(poly=sphere2(r=1, $fn=n))
  [n*(2*ceil(n/2)-4)+2*n, 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);

color(1/1*[1,0,0]) for(p=[for(n=[1:1:N/2]) let(poly=sphere_subdiv(r=1, divs=n, poly=TETRAHEDRON()))
  [len(poly[1]), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);

color(1/3*[0,1,0]) for(p=[for(n=[1:1:ceil(N/3.5)]) let(poly=normalized_cube(r=1,div_count=n))
  [2*len(poly[1]), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);
color(2/3*[0,1,0]) for(p=[for(n=[1:1:ceil(N/3.5)]) let(poly=spherified_cube(r=1,div_count=n))
  [2*len(poly[1]), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);
color(3/3*[0,1,0]) for(p=[for(n=[1:1:ceil(N/3.5)]) let(poly=sphere_subdiv(r=1, divs=n, poly=CUBE()))
  [len(poly[1]), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);

color(1/2*[0,0,1]) for(p=[for(n=[1:1:5]) let(poly=octahedron_old(r=1, n=n))
  [len(poly[1]), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);
color(2/2*[0,0,1]) for(p=[for(n=[1:1:ceil(N/3)]) let(poly=sphere_subdiv(r=1, divs=n, poly=OCTAHEDRON()))
  [len(poly[1]), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);

color(1/3*[1,1,0]) for(p=[for(n=[0:1:4]) let(poly=icosahedron_old(r=1,n=n))
  [len(poly[1]), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);
color(3/3*[1,1,0]) for(p=[for(n=[1:1:ceil(N/4.5)]) let(poly=sphere_subdiv(r=1, divs=n, poly=ICOSAHEDRON()))
  [len(poly[1]), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);

color(3/3*[0,1,1]) for(p=[for(n=[1:1:ceil(N/8)]) let(poly=sphere_subdiv(r=1, divs=n, poly=DODECAHEDRON()))
  [len(poly[1]), 1-signed_volume(poly)/V]])
  translate([p.x, 10/p.y]) circle(R, $fn=fn);
//*/



// This sphere is simpler to code but slightly different geometry from
// default OpenSCAD style. (the poles come to a point)
function sphere2(r=1, d) = let(R = d == undef ? r : d/2)
  rotate_extrude(poly=arc(r=R, angle=180,offsetAngle=-90));


// Platonic solids, centered with unit radius, and triangle-only faces.
function TETRAHEDRON() = [
  [ [-1,-1,1],[1,-1,-1],[-1,1,-1],[1,1,1] ]/sqrt(3),
  [ [0,1,2],[1,3,2],[2,3,0],[3,1,0] ] ];
function CUBE() = [
  [ [ 1, 1, 1],[-1, 1, 1],[-1,-1, 1],[ 1,-1, 1],
    [-1,-1,-1],[ 1,-1,-1],[ 1, 1,-1],[-1, 1,-1] ]/sqrt(3),
  [ [1,3,2],[0,3,1],[1,2,4],[1,4,7],[0,1,6],[1,7,6],
    [4,5,6],[4,6,7],[3,6,5],[0,6,3],[4,3,5],[4,2,3] ] ];
function OCTAHEDRON() = [
  [ [0,0,-1],[1,0,0],[0,1,0],[-1,0,0],[0,-1,0],[0,0,1] ],
  [ [0,3,4],[0,1,2],[0,2,3],[0,4,1],
    [5,2,1],[5,3,2],[5,4,3],[5,1,4] ] ];
function ICOSAHEDRON() = [ let(a = 1, b = (1+sqrt(5))/2, x = norm([a,b]))
  [ [-a,b,0],[a,b,0],[a,-b,0],[-a,-b,0], /* golden rectangle in XY plane */
    [-b,0,a],[b,0,a],[b,0,-a],[-b,0,-a], /* golden rectangle in XZ plane */
    [0,-a,b],[0,a,b],[0,a,-b],[0,-a,-b]  /* golden rectangle in YZ plane */ ] / x,
  [ [ 0, 9, 4],[ 4, 9, 8],[ 8, 9, 5],[ 5, 9, 1],[ 1, 9, 0],
    [ 2,11, 3],[ 3,11, 7],[ 7,11,10],[10,11, 6],[ 6,11, 2],
    [ 8, 5, 2],[ 2, 5, 6],[ 6, 5, 1],[ 6, 1,10],[10, 1, 0],
    [10, 0, 7],[ 7, 0, 4],[ 7, 4, 3],[ 3, 4, 8],[ 8, 2, 3] ] ];
function DODECAHEDRON() = let(a = (1+sqrt(5))/4, b=(3+sqrt(5))/4,
  p0 = [
    [0.0, 0.5,   b],[0.0, 0.5,  -b],[ 0.0,-0.5,   b],[ 0.0,-0.5,  -b],
    [  b, 0.0, 0.5],[  b, 0.0,-0.5],[  -b, 0.0, 0.5],[  -b, 0.0,-0.5],
    [0.5,   b, 0.0],[0.5,  -b, 0.0],[-0.5,   b, 0.0],[-0.5,  -b, 0.0],
    [  a,   a,   a],[  a,   a,  -a],[   a,  -a,   a],[   a,  -a,  -a],
    [ -a,   a,   a],[ -a,   a,  -a],[  -a,  -a,   a],[  -a,  -a,  -a]
  ],
  f0 = [ 
    [ 12 ,  4, 14,  2,  0],[ 16 , 10,  8, 12,  0],[  2 , 18,  6, 16,  0],
    [ 17 , 10, 16,  6,  7],[ 19 ,  3,  1, 17,  7],[  6 , 18, 11, 19,  7],
    [ 15 ,  3, 19, 11,  9],[ 14 ,  4,  5, 15,  9],[ 11 , 18,  2, 14,  9],
    [  8 , 10, 17,  1, 13],[  5 ,  4, 12,  8, 13],[  1 ,  3, 15,  5, 13]
  ],
  p1 = [for(f = f0) vsum([for(pi = f) p0[pi] ]) ],
  points = [for(p=concat(p0, p1)) unit(p)],
  // split pentagonal faces into triangles for subdivision
  faces = [for(fi=[0:11]) let(f=f0[fi]) 
    for(pi=[0:4]) [ len(p0)+fi, f[pi], f[(pi+1)%5] ]
  ]
) [points, faces];


// subdivide faces, splitting edges into integer number of divisions
// input faces must be triangles with vertices on the unit sphere
function sphere_subdiv(r,d,divs=1, poly) = 
  let(
    R = d == undef ? r : d/2, // optional radius or diameter
    d = divs, // shorthand
    pv = poly[0], // points vector
    tv = poly[1], // triangle index vector
    newpoints = [for (t = tv) let(p = [pv[t[0]], pv[t[1]], pv[t[2]]])
      for (i=[0:1:d], j=[0:1:d-i]) if (i+j!=0 && i!=d && j!=d) // skip original corner points
        let(subv=[for (vi=[0:2]) let(k=d-i-j, ii=[i,j,k],
            j1=ii[(vi+1)%3], n=ii[vi]+j1,
            p0=p[vi], p1=p[(vi+1)%3], p2=p[(vi+2)%3],
            p_i=slerp(p0,p1,n/d), p_j=slerp(p0,p2,n/d)
          ) slerp(p_i,p_j,j1/n)
        ])
        unit(vsum(subv))
    ],
    Tn = function(n) n*(n+1)/2, // triangular numbers
    Td = Tn(d+1), // total points for subdivided face
    np = Td - 3, // new points per original face
    lp = len(pv),
    allpoints = concat(pv, newpoints),
    // Given original triangle point indices t, 
    // and indices i,j for subdivided basis vectors, { i => (tri[0],tri[1]), j => (tri[0],tri[2]) }
    // convert to absolute point index of resulting full point set.
    pij = function(n,t,i,j) i+j==0 ? t[0] : i==d ? t[1] : j==d ? t[2] :
      lp + n*np + Td - Tn(d+1-i) + j - (i==0 ? 1 : 2),
    faces = flatten([for (n = [0:1:len(tv)-1]) let(t = tv[n]) [
      for (i=[0:1:d-1], j=[0:1:d-1-i]) [ pij(n,t,i,j), pij(n,t,i+1,j), pij(n,t,i  ,j+1) ],
      for (i=[1:1:d-1], j=[0:1:d-1-i]) [ pij(n,t,i,j), pij(n,t,i,j+1), pij(n,t,i-1,j+1) ] 
    ] ])
  ) d > 1 ? [R*allpoints, faces] : [R*poly[0],poly[1]];




/****************************************************************************
 ** Less effective, obsolete sphere alternatives kept for reference below. **
 ****************************************************************************/      

function normalized_cube(r=1,div_count=12,d) = let(
  R = d == undef ? r : d/2,
  origin=[0,0,1],
  right = [1,0,0],
  up = [0,1,0],
  div2 = div_count/2,
  face_points = [
    for (j = [0:1:div_count], i = [0:1:div_count]) let(
      face_point = origin + 2.0 * (right * (i-div2) + up * (div2-j)) / div_count
    ) R*unit(face_point)
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
    for (f = [0:5], j = [0:1:div_count-1], i = [0:1:div_count-1]) let (
      i0=f*lface+j*(div_count+1)+i,
      i1=i0+1,
      i2=i0+(div_count+1),
      i3=i2+1
    ) [i0,i1,i3,i2]
  ]
) [points,faces];


function spherified_cube(r=1,origin=[0,0,1],div_count=12,d) = let(
  R = d == undef ? r : d/2,
  right = [1,0,0],
  up = [0,1,0],
  div2 = div_count/2,
  face_points = [
    for (j = [0:1:div_count], i = [0:1:div_count]) let(
      p = origin + 2.0 * (right * (i-div2) + up * (div2 - j)) / div_count,
      p2 = [p.x*p.x,p.y*p.y,p.z*p.z],
      rx = p.x * sqrt(1.0 - 0.5 * (p2.y + p2.z) + p2.y*p2.z/3.0),
      ry = p.y * sqrt(1.0 - 0.5 * (p2.z + p2.x) + p2.z*p2.x/3.0),
      rz = p.z * sqrt(1.0 - 0.5 * (p2.x + p2.y) + p2.x*p2.y/3.0)
    ) R*[rx,ry,rz]
  ],
  lface = len(face_points),
  points = concat(
    face_points,
    rotate([ 90,  0,0], poly=face_points),
    rotate([-90,  0,0], poly=face_points),
    rotate([180,  0,0], poly=face_points),
    rotate([  0,-90,0], poly=face_points),
    rotate([  0, 90,0], poly=face_points)
  ),
  faces = [
    for (f = [0:5], j = [0:1:div_count-1], i = [0:1:div_count-1]) let (
      i0=f*lface+j*(div_count+1)+i,
      i1=i0+1,
      i2=i0+(div_count+1),
      i3=i2+1
    ) [i0,i1,i3,i2]
  ]
) [points,faces];

// simple normalized octahedron
function octahedron_old(r=1,d,n=0) =
  let(
    R = d == undef ? r : d/2,
    points = [
      [1,0,0],[0,1,0],[-1,0,0],[0,-1,0],[0,0,1],[0,0,-1]
    ],
    faces = [
      [0,4,1],[1,4,2],[2,4,3],[3,4,0],
      [0,1,5],[1,2,5],[2,3,5],[3,0,5]
    ],
    poly = n==0 ? [points,faces] : subdivide_faces(n=n, poly=[points,faces])
  )
  [[for(p = poly[0]) R*unit(p)], poly[1]];

// n = subdivision iterations (number of faces = 20 * 4^n)
// points constructed from golden rectangles as shown here https://commons.wikimedia.org/wiki/File:Icosahedron-golden-rectangles.svg
function icosahedron_old(r=1,d,n=0) = let(
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
) [[for (p = poly[0]) R*unit(p)],poly[1]];
