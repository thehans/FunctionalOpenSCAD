use <functional.scad>

// Set View -> Animate, and set FPS to 60 and Steps to 200 or for example morphing through some possible combinations

// sine function for animating oscillating parameters
function sine(minY=-1,maxY=1,freq=1,phase=0,x=$t) = minY + (maxY - minY) * (sin(360*freq*x+360*phase)+1)/2;

profile = double_fillet(h=10, r1=sine(1,4,3), r2=sine(1,4,2,0.25), xoffset1=12, xoffset2=sine(-15,5,phase=0.66), $fn=100);
rotate_extrude(angle=270, convexity=3,$fn=200) polygon(profile);

// Double Fillet generates a path that is a smooth transition between two parallel surfaces
// h is the vertical distance between surfaces, negative height will mirror about the vertical axis
// r1 and r2 are the first and second fillet radius traversing away from origin
// xoffset1 distance from origin where first radius begins ( should be >= 0 )
// xoffset2 distance from edge of first radius to the start of second radius.  0 value makes straight wall, < 0 makes overhang
// closed = true will return a closed polygon ready for extrusion, 
//    while cloesd == false returns a just the curved vertex path that can be use as part of a larger path
function double_fillet(h=1, r1=1, r2=1, xoffset1=0, xoffset2=0, closed=true) = 
  let(
    ah = abs(h),
    xL = r1 + r2 + xoffset2,
    yL = ah - r1 - r2,
    L = max(sqrt(xL * xL + yL * yL), r1 + r2),
    a1 = 90-acos( (r1 + r2) / L ),
    a2 = atan2( yL, xL ),
    a = a1 + a2,
    c1 = [xoffset1,ah-r1],
    c2 = c1 + [xL,-yL],
    arc1 = arc(r=r1, angle=-a, offsetAngle=90,c=c1),
    arc2 = arc(r=r2, angle=-a, offsetAngle=-90,c=c2)
  )
  mirror([0,h < 0 ? 1 : 0],
  concat(
    closed ? [[0,0],[0,ah]] : [],
    arc1,
    reverse(arc2)
  ));
