use <functional.scad>

webbing_demo($t=$t+0.125);

// show different examples of how webbing might be applied to generate 2d shapes which can then be extruded with linear_extrude or rotate_extrude
// If Animation is enabled, you can see the shapes iterate over various combinations of parameters
module webbing_demo() {
  d1 = 20 * (1+sin(360*2*$t + 0))/2+1;
  d2 = 20 * (1+sin(360*3*$t))/2+1;
  c2 = [0,50*(sin(360*$t))];
  th = 10*(1+sin(360*4*$t-90))/2+1;

  $fs=0.5;
  $fa=0.1;

  steps = 10;
  spacing = 40;

  web = webbing(d1, d2, c2, th);

  // show circles and minimum thickness that webbing are based on
  color([1,1,1]) linear_extrude(10) circle(d=d1);
  color([1,1,1]) linear_extrude(10.01) translate(c2) circle(d=d2);
  color(0.75*[1,1,1]) linear_extrude(10.02) translate([-th/2,c2.y < 0 ? c2.y : 0]) square([th, norm(c2)]);

  color([1,1,1]) { 

    // solid webbing
    translate([spacing,0]) linear_extrude(10)
      poly2d(web);

    // larger webbing with padding around holes of the original specificed diameter, but keeping original thickness
    translate([spacing*2,0]) linear_extrude(10, convexity=2) difference() {
      poly2d( webbing(d1+th, d2+th, c2, th) );
      circle(d=d1);
      translate(c2) circle(d=d2);
    }

    // larger padded webbing subtracted by original size webbing
    translate([spacing*3,0]) linear_extrude(10, convexity=2) 
      difference() {
        poly2d( webbing(d1+th, d2+th, c2, th*2) );
        poly2d(web);
      }

    // using positive offset instead of larger webbing (should be the same i think)
    translate([spacing*4,0]) linear_extrude(10, convexity=5) 
      difference() {
        offset(th/2) poly2d(web);
        poly2d(web);
      }

    // using negative offset to sutract a smaller webbing from the original sized one
    translate([spacing*5,0]) linear_extrude(10, convexity=5) difference() {
      poly2d(web);
      offset(r=-th/3) poly2d(web);
    }

    // using webbing with thickness subtracted from diamters, can create self intersecting shape when diameters become negative
    translate([spacing*6,0]) linear_extrude(10, convexity=5) difference() {
      poly2d(web);
      poly2d( webbing((d1-2*th/3), (d2-2*th/3), c2, th/3) );
    }

  }

}


// possible situations
// circles do not intersect
// circles are tangent, outside each other 
// circles intersect, and point of intersection is horizontally between circle centers
// circles intersect, and point of intersection is horizontally to the right of both circle centers
// circles are coincident (same size and offset): draw the outer circle
// one circle is inside the other (possibly tangent):  draw the outer circle

// draw two circles connected by a continuous curve that necks down to a minimum thickness th, 
// c2 is expected to be an [x,y] translation pair
// use r3 parameter to override the radius calculated for th 
function webbing(d1, d2, c2, th, r3) = 
  webbing_shape(d1/2, d2/2, r3 == undef ? webbing_r3(d1/2, d2/2, th, norm(c2)): r3, c2);

  
// solve the following for r3:  
// c = sqrt( (r1+r3)^2 - (r3+w)^2 ) + sqrt( (r2+r3)^2 - (r3+w)^2 )
// c is the distance between centers
// w is the minimum webbing width
function webbing_r3(r1, r2, w, c) = 
  let(
    eps = 1e-9,
    x1 = 4*c*c*r1 + 4*c*c*r2 - 4*c*c*w - 4*r1*r1*r1 + 4*r1*r1*r2 + 4*r1*r2*r2 - 4*r2*r2*r2 ,
    x2 = sqrt(pow(-4*c*c*r1-4*c*c*r2+4*c*c*w+4*r1*r1*r1-4*r1*r1*r2-4*r1*r2*r2+4*r2*r2*r2,2) - 4*(4*r1*r1-8*r1*r2+4*r2*r2)*(c*c*c*c-2*c*c*r1*r1-2*c*c*r2*r2+c*c*w*w+pow(r1,4)-2*r1*r1*r2*r2+pow(r2,4))),
    x3 = 2*(4*r1*r1-8*r1*r2+4*r2*r2)
  )
  abs(r1 - r2) < eps ? // radii considered same below some threshold
    (w/2 > r1) ? 1/0 : // r3 = infinity to represent flat sides
    max(0,(c*c/4 + w*w/4 - r1*r1) / (2*r1 - w)) : // symmetrical case
    (((w/2 >= min(r1,r2)) ? // width larger than smaller diameter
      ((r2 > r1) ?
        max(0,(c*c + r1*r1 - r2*r2) / (2 * (r2 - r1))) : 
        max(0,(c*c - r1*r1 + r2*r2) / (2 * (r1 - r2)))
      ) :
      max(0, (x1 - x2) / x3) // general case
      )
    );

// Law of cosines, get angles based on knowing 3 sides of a non-right triangle
function getLawCosA(a,b,c) = acos((b*b + c*c - a*a) / (2*b*c));
function getLawCosB(a,b,c) = acos((c*c + a*a - b*b) / (2*c*a));
function getLawCosC(a,b,c) = acos((a*a + b*b - c*c) / (2*a*b));

// draw the webbing shape once all radii are known
// r1 center is at origin
// r2 center is at c2
// r3 center is at c3 (deduced from other data, knowing that it must be tangent to r1 and r2)
function webbing_shape(r1,r2,r3,c2) = 
  let(
    inf = 1e9,
    d = norm(c2),
    a2 = atan2(c2.y, c2.x),
    a = r1+r3,
    b = r2+r3,
    B = getLawCosB(a, b, d),
    C = getLawCosC(a, b, d),
    c3 = [a*cos(B), a*sin(B)],
    a1 = atan2(c3.y, c3.x-d), // distnace between centers of r3 and r2
    minr = min(r1,r2), maxr = max(r1,r2),
    points = d+minr <= maxr ? // check if one circle is within another
      circle(r=max(r1,r2), c=[maxr==r2 ? d : 0, 0]) :
      (r3 >= inf) ? 
        concat(
          arc(r=r1, angle=180, offsetAngle=180, c=[0,0], center=true),
          arc(r=r2, angle=180, offsetAngle=0, c=[d,0], center=true)
        ) :
        concat(
          arc(r=r1, angle=2*(180-B), offsetAngle=180, c=[0,0], center=true),
          arc(r=r3, angle=-C, offsetAngle=180-B, c=[c3.x,-c3.y]),
          arc(r=r2, angle=2*a1, offsetAngle=0, c=[d,0], center=true),
          arc(r=r3, angle=-C, offsetAngle=180+a1, c=c3)
        )
  )
  rotate(a=a2, poly=points);
