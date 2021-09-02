use <functional.scad>

// Divide faces (triangles only) into 4 sub-triangles, recursively
// "Resolution" is limited to (4^n * input_faces)
function subdivide_faces(n=1, poly) = 
  let(
    points = poly[0],
    tris = poly[1],
    newpoints = flatten([for (t = tris) 
      let(
        p0 = points[t[0]],
        p1 = points[t[1]],
        p2 = points[t[2]]
      )
      [(p0+p1)/2, (p1+p2)/2, (p2+p0)/2]
    ]),
    lp = len(points),
    allpoints = concat(points, newpoints),
    faces = flatten([for (i = [0:len(tris) - 1])
      let(
        f = tris[i],
        p0 = f[0], p1 = f[1], p2 = f[2],
        m0 = lp + i * 3, m1 = m0 + 1, m2 = m1 + 1
      )
      [[p0,m0,m2],
       [m0,p1,m1],
       [m0,m1,m2],
       [m2,m1,p2]]
    ])
  )
  n > 1 ? subdivide_faces(n-1, [allpoints, faces]) : n > 0 ? [allpoints,faces] : poly;
