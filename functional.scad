// "Functional OpenSCAD" library by Hans Loeblich
//
// https://github.com/thehans/FunctionalOpenSCAD
//
// This library is an attempt to re-implement OpenSCAD's builtin modules
// as functions which operate directly on vertex data.

// TODO support multiple paths for polygons in linear_extrude and rotate_extrude
// TODO better document utility functions

functional_examples();

// Some basic examples of how library functions can be used
module functional_examples() {
  // multiple nested function calls, with the results of a function passed as poly parameter for another
  color("yellow") poly3d(
    translate([10,0,0],
      poly=scale([1,2,4],
        poly=sphere(1,$fn=30)
      )
    )
  );

  r = 2;

  // assigning function results to intermediate variables
  shape = sphere(r=r,$fn=20);
  moved_shape = translate([-10,0,0], poly=shape);
  color("blue") poly3d(moved_shape);

  // calculate properties of the geometry
  echo(str("volume of blue: ", signed_volume(moved_shape), " mm^3" ) );
  echo(str("volume of perfect sphere: V = 4/3*PI*r^3 = ", (4/3)*PI*pow(r,3), " mm^3" ));

  // make a vector containing multiple shapes
  shape_vector = [ for (i = [-1:1]) translate([0,i*10,0], poly=cube(i*2+4,center=true)) ];
  color("green") poly3d(shape_vector);

  // compute properties on lists
  b = bounds(shape_vector);
  echo(bounds=b);
  volumes = [for (shape = shape_vector) signed_volume(shape) ];
  echo(volumes=volumes);
  volumesum = signed_volume(shape_vector);
  echo(volumesum=volumesum);

  // show corner points of a bounding volume
  color("red") showPoints(b, r=0.5, $fn=40);

  // debug point locations
  showPoints(shape_vector);
  //echo(shape_vector=shape_vector);

  // display the bounding volume
  //color([1,1,1,0.1]) translate(b[0]) cube(b[1]-b[0]);
}

// basic utility functions
function is_vec(a) = version_num()<20190500 ?
  (len(a) ? true : false) :
  is_list(a);
function unit(v) = v / norm(v); // convert vector to unit vector
function flatten(l) = [ for (a = l) for (b = a) b ];
function reverse(v) = [ for (i = [0:len(v)-1])  v[len(v) -1 - i] ];
// integer based range, inclusive
function irange(a,b) = let (step = a > b ? -1 : 1) [ for (i = [a:step:b]) i ];

// sum a vector of numbers.  sum([]) == 0
function sum(v, i=0, s=0) = len(v) > i ? sum(v, i+1, s+v[i]) : s;
// sum a vector of vectors.  vsum([]) == undef
function vsum(v,i=0) = len(v)-1 > i ? v[i] + vsum(v, i+1) : v[i];

// depth of first elements, not necessarily max depth of a structure
function depth(a,n=0) = is_vec(a) ? depth(a[0],n+1) : n;
function default(x,default) = x == undef ? default : x;
// "polyhole" conversion, typically for internal holes
function to_internal(r_in, r_max) = r_in / cos (180 / fragments(r_max == undef ? r_in : r_max));
// angle between two vectors (2D or 3D)
function anglev(v1,v2) = acos( (v1*v2) / (norm(v1)*norm(v2) ) );
// spherical linear interpolation
function slerp(p0,p1,t) = let(a = anglev(p0,p1)) (sin((1-t)*a)*p0 + sin(t*a)*p1) / sin(a);

function sinh(x) = (1 - exp( -2 * x) )/ (2 * exp(-x));
function cosh(x) = (1 + exp( -2 * x)) / (2 * exp(-x));
function tanh(x) = sinh(x) / cosh(x);
function cot(x) = 1 / tan(x);
function mod(a,m) = a - m*floor(a/m);

// get/set point array from 3 possible input types:
//    poly=[points, faces] (as used by polyhedron),
//    poly=[points, paths] (as used by polygon),
// or poly=points (used by polygon or for intermediate point data processing)
function get_points(poly) = depth(poly) <= 2 ? poly : poly[0];
function set_points(poly, points) = depth(poly) <= 2 ? points : [points,poly[1]];

function is_poly_vector(poly) = depth(poly) > 3;
function is_3d_poly(poly) = is_poly_vector(poly) ?
  len(get_points(poly[0])[0]) == 3 :
  len(get_points(poly)[0]) == 3;

// Helper functions mainly used within other functions:

// based on get_fragments_from_r documented on wiki
// https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language#$fa,_$fs_and_$fn
function fragments(r=1) = ($fn > 0) ?
  ($fn >= 3 ? $fn : 3) :
  ceil(max(min(360.0 / $fa, r*2*PI / $fs), 5));

// Calculate fragments for a linear dimension.
// (don't factor in radius-based calculations)
function linear_fragments(l=1) = ($fn > 0) ?
  ($fn >= 3 ? $fn : 3) :
  ceil(max(l / $fs),5);

// Generate a list of points for a circular arc with center c, radius r, etc.
// "center" parameter centers the sweep of the arc about the offsetAngle (half to each side of it)
// "internal" parameter enables polyhole radius correction
// optional "d" parameter overrides r
// optional "fragments" parameter overrides calculations from $fn,$fs,$fa with a direct input
// optional "endpoint" parameter specifies whether or not to include the last point in the arc,
//    by default the endpoint is included as long as the angle is not 360
//    setting endpoint=false can be useful to avoid duplicating points if you are concatenating arc paths together
function arc(r=1, angle=360, offsetAngle=0, c=[0,0], center=false, internal=false, d, fragments, endpoint) =
  let (
    r1 = d==undef ? r : d/2,
    fragments = fragments==undef ? ceil((abs(angle) / 360) * fragments(r1)) : fragments,
    step = angle / fragments,
    a = offsetAngle-(center ? angle/2 : 0),
    R = internal ? to_internal(r1) : r1,
    last = endpoint==undef ? (abs(angle) == 360 ? 1 : 0) : (endpoint ? 0 : 1)
  )
  [ for (i = [0:fragments-last] ) let(a2=i*step+a) c+R*[cos(a2), sin(a2)] ];

// **2D Primitives**
function square(size=1, center=false,r=0) =
  let(
    x = is_vec(size) ? size.x : size,
    y = is_vec(size) ? size.y : size,
    o = center ? [-x/2,-y/2] : [0,0],
    d = r*2
  )
  //assert(d <= x && d <= y)
  translate(o,
    (r > 0 ?
      concat(
        arc(r=r, angle=-90, offsetAngle=0,   c=[x-r,  r]),
        arc(r=r, angle=-90, offsetAngle=270, c=[  r,  r]),
        arc(r=r, angle=-90, offsetAngle=180, c=[  r,y-r]),
        arc(r=r, angle=-90, offsetAngle=90,  c=[x-r,y-r])
      ) :
      [[[0,0],[0,y],[x,y],[x,0]],[[0,1,2,3]]]
    )
  );

function circle(r=1, c=[0,0], internal=false, offsetAngle=0, d) =
  let(
    r1 = d==undef ? r : d/2,
    points = arc(r=r1,c=c,angle=-360,offsetAngle=offsetAngle,internal=internal)
  )
  [points,[irange(0,len(points)-1)]];


// **3D Primitives**
function cube(size=1, center=false) =
  let(
    s = is_vec(size) ? size : [size,size,size],
    points = [
      [0,0,0],[s.x,0,0],[0,s.y,0],[s.x,s.y,0],
      [0,0,s.z],[s.x,0,s.z],[0,s.y,s.z],[s.x,s.y,s.z]],
    faces = [[3,1,5,7],[0,1,3,2],[1,0,4,5],[2,3,7,6],[0,2,6,4],[5,4,6,7]],
    c = is_vec(center) ? [center.x ? s.x : 0, center.y ? s.y : 0, center.z ? s.z : 0]/2 : s/2
  )
  center ? [translate(-c,points),faces] : [points,faces];

function sphere(r=1, d) =
  let(
    R = d == undef ? r : d/2,
    fragments = fragments(R),
    rings = floor((fragments+1)/2),
    c1 = get_points(circle(r=R)),
    points = flatten([for (i = [0:rings-1])
      let(angle =  (180 * (i + 0.5)) / rings)
      translate([0,0,R*cos(angle)], poly=sin(angle)*c1)
    ]),
    lp = len(points),
    faces = concat([for (i = [0:rings-2], j = [0:fragments-1])
      let (
        il = i * fragments,
        il1 = il + fragments,
        j1 = (j == fragments-1) ? 0 : j+1,
        i0 = il + j,
        i1 = il + j1,
        i2 = il1 + j,
        i3 = il1 + j1
      )
      [i0,i2,i3,i1]
    ], [irange(0,fragments-1),     // top ring face
        irange(lp-1,lp-fragments)] // bottom ring face
    )
  )
  [points,faces];

// cylinder
function cylinder(h=1,r1,r2,center=false,r,d,d1,d2) =
  let(
    R1 = (d1 == undef ? (d == undef ? (r1 == undef ? (r == undef ? 1 : r) : r1) : d/2) : d1/2),
    R2 = (d2 == undef ? (d == undef ? (r2 == undef ? (r == undef ? 1 : r) : r2) : d/2) : d2/2),
    scale = R2/R1
  )
  linear_extrude(height=h, scale=scale, center=center, poly=circle(r=R1));



// **3D to 2D**
// TODO projection



// **2D to 3D**
function linear_extrude(height=100, center=false, convexity=1, twist=0, slices, scale=1.0, poly) =
  is_poly_vector(poly) ?
    [for (p = poly) _linear_extrude(height=height,center=center,convexity=convexity,twist=twist,slices=slices,scale=scale,poly=poly)] :
    _linear_extrude(height=height,center=center,convexity=convexity,twist=twist,slices=slices,scale=scale,poly=poly);

function _linear_extrude(height, center, convexity, twist, slices, scale, poly) =
  let(
    points = get_points(poly),
    sl = slices == undef ? (twist == 0 ? 1 : 7) : slices,
    hstep = height/sl,
    astep = -twist/sl,
    sstep = (scale-1)/sl,
    hoffset = center ? -height/2 : 0,
    newPoints = flatten([for (i = [0:sl])
      rotate(a=astep*i, poly=
        translate([0,0,hstep*i+hoffset], poly=
          scale(1+sstep*i, poly=
            points
          )
        )
      )
    ]),
    l = len(points),
    lp = len(newPoints),
    faces = concat(
      flatten(
        [for (i = [0:sl-1], j = [0:l-1] )
          let(
            il = i*l,
            j1 = j + 1 == l ? 0 : j+1,
            i0 = il + j,
            i1 = il + j1,
            i2 = il+l + j,
            i3 = il+l + j1
          )
          [[i0,i1,i3],[i0,i3,i2]]
        ]),
        [irange(l-1,0),irange(lp-l,lp-1)]
    )
  )
  [newPoints, faces];


// generate points/paths for a polyhedron, given a vector of 2d point data
function rotate_extrude(angle=360, offsetAngle=0, center=false, i_offset=0, poly) =
  let(
    points = get_points(poly),
    a = ((angle != angle/*nan check*/ || angle==undef || angle > 360) ? 360 :
      (angle <= -360 ? 360 : angle)
    ),
    full_rev = a==360 ? 1 : 0,
    l = len(points),
    xs = [for (p = points) p.x],
    min_x = min(xs),
    max_x = max(xs),
    fragments = ceil((abs(a) / 360) * fragments(max_x)),
    steps = fragments - full_rev,
    step = a / fragments,
    a1 = offsetAngle-(center ? a/2 : 0),
    ps = signed_area(points) < 0 ? reverse(points) : points,
    out_points = flatten([
      for (i = [0:steps] )
        let(a2=i*step+a1)
        rotate([90,0,a2], poly=ps)
    ]),
    lp = len(out_points),
    out_paths = [
      for (i = [0:fragments-1], j = [0:l-1])
        let(
          il = i*l,
          il1 = (i == steps) ? 0 : (i+1)*l,
          j1 = (j == l-1) ? 0 : j+1,
          a=il+j,
          b=il+j1,
          c=il1+j,
          d=il1+j1,
          ax=ps[j].x,
          bx=ps[j1].x
        )
        if (ax != 0 || bx != 0) // filter zero area tri
          (ax == 0) ?           // ax == cx == 0
            [c,b,d] :           // triangle fan
          (bx == 0 ?            // bx == dx == 0
            [a,b,c] :           // triangle fan
            [a,b,d,c]           // full quad
          )
    ],
    faces = full_rev ?
      out_paths :
      concat(out_paths, [irange(l-1,0),irange(lp-l,lp-1)]) // include end caps
  )
  //assert(min_x >= 0)
  [out_points, faces];


// **Transform**

// scale
function scale(v=1, poly) = is_poly_vector(poly) ?
  [for (p=poly) _scale(v=v,poly=p)] :
  _scale(v=v,poly=poly);

// scale for single poly, no vectors of polys
function _scale(v, poly) =
  let(
    points = get_points(poly),
    s = is_vec(v) ? v : [v,v,v],
    newPoints = len(points[0]) == 3 ?
      [for (p = points) [s.x*p.x,s.y*p.y,s.z*p.z]] :
      [for (p = points) [s.x*p.x,s.y*p.y]]
  )
  set_points(poly, newPoints);


// resize
function resize(newsize,poly) =
  let(
    b = bounds(poly),
    minB = b[0],
    maxB = b[1],
    sX = newsize.x ? newsize.x/(maxB.x-minB.x) : 1,
    sY = newsize.y ? newsize.y/(maxB.y-minB.y) : 1,
    v1 = len(minB) == 3 ?
      [sX, sY, newsize.z ? newsize.z/(maxB.z-minB.z) : 1] :
      [sX, sY]
  )
  scale(v1,poly);

function rotate(a=0, v, poly) = is_poly_vector(poly) ?
  [for (p=poly) _rotate(a=a,v=v,poly=p)] :
  _rotate(a=a,v=v,poly=poly);

function _rotate(a, v, poly) =
  let(
    points = get_points(poly),
    newPoints = is_3d_poly(points) || (is_vec(a) && len(a) == 3) ?
      _rotate3d(a=a,v=v,points=points) :
      _rotate2d(a=a,points=points)
  )
  set_points(poly, newPoints);

function _rotate3d(a,v,points) =
  let(A = is_vec(a) ? to3d(a) : [0,0,a])
  (!is_vec(a) && is_vec(v)) ?
    _rotate3d_v(a,unit(to3d(v)),points) :
    _rotate3d_xyz(A,points);

// rotate a list of 3d points by angle vector a
// a = [pitch,roll,yaw] in degrees
function _rotate3d_xyz(a, points) =
  let(
    cosa = cos(a.z), sina = sin(a.z),
    cosb = cos(a.y), sinb = sin(a.y),
    cosc = cos(a.x), sinc = sin(a.x),
    Axx = cosa*cosb,
    Axy = cosa*sinb*sinc - sina*cosc,
    Axz = cosa*sinb*cosc + sina*sinc,
    Ayx = sina*cosb,
    Ayy = sina*sinb*sinc + cosa*cosc,
    Ayz = sina*sinb*cosc - cosa*sinc,
    Azx = -sinb,
    Azy = cosb*sinc,
    Azz = cosb*cosc
  )
  [for (p = points)
    let( pz = (p.z == undef) ? 0 : p.z )
    [Axx*p.x + Axy*p.y + Axz*pz,
     Ayx*p.x + Ayy*p.y + Ayz*pz,
     Azx*p.x + Azy*p.y + Azz*pz] ];

function _rotate3d_v(a, v, points) =
  let(
    cosa = cos(a), sina = sin(a)
  )
  [for (p = points)
    let( P=to3d(p) )
    P*cosa+(cross(v,P))*sina+v*(v*P)*(1-cosa) // Rodrigues' rotation formula
  ];

// rotate about z axis without adding 3rd dimension to points
function _rotate2d(a, points) =
  let(cosa = cos(a), sina = sin(a))
  [for (p = points) [p.x * cosa - p.y * sina,p.x * sina + p.y * cosa]];

function translate(v, poly) = is_poly_vector(poly) ?
  [for (p=poly) _translate(v=v, poly=p)] :
  _translate(v=v, poly=poly);

function _translate(v, poly) =
  let(
    points = get_points(poly),
    lp = len(points[0]), // 2d or 3d point data?
    lv = len(v),         // 2d or 3d translation vector?
    V = lp > lv ? [v.x,v.y,0] : v, // allow adding a 2d vector to 3d points
    newPoints = (lv > lp) ?
        [for (p = points) [p.x,p.y,0]+V] : // allow adding a 3d vector to 2d point data
        [for (p = points) p+V] // allow adding 2d or 3d vectors
  )
  set_points(poly, newPoints);

function mirror(normal, poly) = is_poly_vector(poly) ?
  [for (p=poly) _mirror(normal=normal, poly=p)] :
  _mirror(normal=normal, poly=poly);

function _mirror(normal=[1,0], poly) =
  let(
    points = get_points(poly),
    newPoints = [for (p = points)
      let(
        n = normal*normal,
        t = n == 0 ? 0 : (-p*normal) / n
      )
      p + 2*t*normal
    ]
  )
  set_points(poly, newPoints);

function multmatrix(M, poly) = is_poly_vector(poly) ?
  [for (p = poly) _multmatrix(M=M,poly=p)] :
  _multmatrix(M=M,poly=poly);

function _multmatrix(M, poly) =
  let(
    points = get_points(poly),
    newPoints = is_3d_poly(poly) ?
      [for (p = points) to3d(M*[p.x,p.y,p.z,1])] :
      [for (p = points) to2d(M*[p.x,p.y,0,1])]
  )
  set_points(poly, newPoints);


function bounds(poly) = is_3d_poly(poly) ?
  (is_poly_vector(poly) ?
    _bounds_multi_3d(poly) :
    _bounds_3d(poly)) :
  (is_poly_vector(poly) ?
    _bounds_multi_2d(poly) :
    _bounds_2d(poly));

function _bounds_2d(poly) =
    let(
      points = get_points(poly),
      minX = min([for (p = points) p.x]),
      maxX = max([for (p = points) p.x]),
      minY = min([for (p = points) p.y]),
      maxY = max([for (p = points) p.y])
    )
    [[minX,minY],[maxX,maxY]];

function _bounds_3d(poly) =
    let(
      points = get_points(poly),
      minX = min([for (p = points) p.x]),
      maxX = max([for (p = points) p.x]),
      minY = min([for (p = points) p.y]),
      maxY = max([for (p = points) p.y]),
      minZ = min([for (p = points) p.z]),
      maxZ = max([for (p = points) p.z])
    )
    [[minX,minY,minZ],[maxX,maxY,maxZ]];

function _bounds_multi_2d(polys) =
    let(
      minX = min([for (poly=polys, p = get_points(poly)) p.x]),
      maxX = max([for (poly=polys, p = get_points(poly)) p.x]),
      minY = min([for (poly=polys, p = get_points(poly)) p.y]),
      maxY = max([for (poly=polys, p = get_points(poly)) p.y])
    )
    [[minX,minY],[maxX,maxY]];

function _bounds_multi_3d(polys) =
    let(
      minX = min([for (poly=polys, p = get_points(poly)) p.x]),
      maxX = max([for (poly=polys, p = get_points(poly)) p.x]),
      minY = min([for (poly=polys, p = get_points(poly)) p.y]),
      maxY = max([for (poly=polys, p = get_points(poly)) p.y]),
      minZ = min([for (poly=polys, p = get_points(poly)) p.z]),
      maxZ = max([for (poly=polys, p = get_points(poly)) p.z])
    )
    [[minX,minY,minZ],[maxX,maxY,maxZ]];

// reverse direction of all faces in polyhedron.
function invert(poly) =
  let(faces = poly[1])
  [ poly[0], [for (face = faces) reverse(face)] ];

function to3d(p) = let(l = len(p)) (l>2 ? [p.x,p.y,p.z] : (l>1 ? [p.x,p.y,0]:(l>0 ? [p.x,0,0] : [0,0,0])));
function to2d(p) = let(l = len(p)) (l>1 ? [p.x,p.y]:(l>0 ? [p.x,0] : [0,0]));

// shoelace formula, returns negative value for clockwise wound polygons
function signed_area(points) =
  let( l = len(points) )
  sum([
    for (i = [0:l-1])
      let(
        p_i = points[i],
        i1 = i+1,
        p_i1 = points[i1 >= l ? i1-l : i1]
      )
      p_i.x * p_i1.y - p_i1.x * p_i.y
  ])/2;

function signed_volume(poly) = is_poly_vector(poly) ?
  sum([for (p=poly) _signed_volume(p)]) :
  _signed_volume(poly);

function _signed_volume(poly) = let(points = poly[0])
  sum([for(face = poly[1], i = [1:len(face)-2]) assert(len(face) > 2)
    cross(points[face[i+1]], points[face[i]]) * points[face[0]]
  ])/6;

/* Visualizations */

// visualize a vector of points
module showPoints(poly, r=0.1, $fn=8) {
  if (is_poly_vector(poly))
    for (p = poly) _showPoints(p, r);
  else
    _showPoints(poly, r);
}

module _showPoints(poly, r) {
  points = get_points(poly);
  for (c = points) translate(c) sphere(r=r);
}

module poly3d(poly, convexity=1) {
  if (is_poly_vector(poly))
    for (p = poly) polyhedron(points=p[0],faces=p[1], convexity=convexity);
  else
    polyhedron(points=poly[0],faces=poly[1], convexity=convexity);
}

module poly2d(poly) {
  if (is_poly_vector(poly))
    for (p = poly) polygon(points=p[0], paths=p[1]);
  else {
    if (depth(poly) == 2)
      polygon(poly);
    else
      polygon(points=poly[0], paths=poly[1]);
  }
}
