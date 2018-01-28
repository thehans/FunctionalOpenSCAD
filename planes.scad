use <functional.scad>

planes_demo();

module planes_demo() {
  a = [1,4,3];
  b = [2,3,4];
  c = [9,0,5];
  pl1 = planeFromPoints(a,b,c);
  /*
  showPoints([a,b,c,pl1[0]*pl1[1]]);
  echo(pl1);
  color([1,1,1,0.2]) showPlane(pl1);
  */

  polyg1 = circle(r=4, $fn=8)[0];
  polyg2 = [for (p = polyg1) to3d(p)];
  
  polyh = [polyg2, [irange(0,len(polyg2)-1)]];
  pl2 = planeFromPoints([1,3,1],[0,0,0],[1,10,-1]);
  //echo(pl2);

  spbp = splitPolygonByPlane(pl2, polyh, 0);
  color("red") scale([1,1,0.05]) polygon([for (p = spbp[0]) to2d(p)]);
  color("blue") scale([1,1,0.05]) polygon([for (p = spbp[3]) to2d(p)]);
  color([1,1,1,0.2]) showPlane(pl2);

  echo("spbp", [for (p = spbp[0]) to2d(p)]);
}


function planeFromPoints(a, b, c) = 
  let(
    a3 = to3d(a), b3 = to3d(b), c3 = to3d(c),
    n = unit(cross(b3 - a3, c3 - a3)),
    w = n*a3
  )
  [n,w];

function planeFromFace(poly, iface, lasti=2) = 
  let(
    points = poly[0],
    face = poly[1][iface],
    plane = planeFromPoints(points[face[0]],points[face[1]],points[face[lasti]])
  )
  plane;

module showPlane(plane, size=[100,100,0.05]) {
    n = plane[0];
    w = plane[1];
    up = [0, 0, 1];
    a = acos(n * up);
    v = cross(n, up);
    //echo("nup", n,up);
    rotate(a=-a, v=v) translate([0,0,w])
      cube(size, center=true);
}  


function filter_nan(v) = [for (x = v) if (x==x) x];



// *** INCORRECT IMPLEMENTATION, NEEDS DEBUGGING ***
// Atttempted port of OpenJSCAD / csg.js "splitPolygonByPlane" function
// returns [front, coplanarFront, coplanarBack, back]
// each containing a list of points, or nan
// *** INCORRECT IMPLEMENTATION, NEEDS DEBUGGING ***
function splitPolygonByPlane(plane, polyh, iface) = 
  let(
    eps = 1e-5,
    COPLANAR = 0,
    FRONT = 1,
    BACK = 2,
    SPANNING = 3,
    NaN = 0/0,
    allpoints = polyh[0],
    face = polyh[1][iface],
    points = [for (pi = face) allpoints[pi] ],
    n = plane[0], w = plane[1],
    l = len(points),
    polyPlane = planeFromPoints(points[0], points[1], points[2]),
    nPoly = polyPlane[0],
    types = [for (i = [0:l-1]) 
      let( t = n*points[i] - w )
      (t < -eps) ? BACK : (t > eps) ? FRONT : COPLANAR
    ],
    // search returns list of length 0 or 1 by default
    polygonType = FRONT * len(search( FRONT, types )) + 
                  BACK * len(search( BACK, types )),
    front = polygonType == SPANNING ? 
      filter_nan(flatten([for (i = [0:l-1])   
        let(
          j = (i + 1) % l,
          ti = types[i], tj = types[j],
          vi = points[i], vj = points[j]
        ) ti+tj == SPANNING ? 
          [ ti != BACK ? i : NaN, vi+(vj-vi)*((w-n*vi)/(n*(vj-vi))) ] :
          [ti != BACK ? vi : NaN]
      ])) : [],
    back = polygonType == SPANNING ? 
      filter_nan(flatten([for (i = [0:l-1])   
        let(
          j = (i + 1) % l,
          ti = types[i], tj = types[j],
          vi = points[i], vj = points[j]
        ) ti+tj == SPANNING ? 
          [ ti != FRONT ? vi : NaN, vi+(vj-vi)*((w-n*vi)/(n*(vj-vi))) ] :
          [ti != FRONT ? vi : NaN]
      ])) : [],
      lf = len(front), lb = len(back)
  )
  (polygonType == FRONT) ? 
    [points, NaN, NaN, NaN] : 
    (polygonType == BACK ? 
      [NaN, NaN, NaN, points] :
      (polygonType == COPLANAR ?
        (n*nPoly > 0 ? 
          [NaN, points, NaN, NaN] : 
          [NaN, NaN, points, NaN]
        ) : 
        // polygonType == SPANNING   
        [lf >= 3 ? front : NaN, NaN, NaN, lb >= 3 ? back : NaN]
      )
    );
