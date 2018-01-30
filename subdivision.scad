use <functional.scad>

// faces better be triangles... or else!

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

//Utility function for splitting circular vectors.
function circle_cut (list,start,end)=
   (end<start)?
      concat([for (i=[start:len(list)-1]) list[i]],[for (i=[0:end]) list[i]])
   :
      [for (i=[start:end]) list[i]]
;

//Ineffictient (Order N^2) naive face triangulation function that works
//by ear clipping:
//Finds a convex vertex (order N)
//Checks if the 'ear' with that vertex contains any points.
//If not, the ear can be clipped, otherwise there's a diagonal in the ear,

function triangulate_poly3d(poly)=
   [poly[0],triangulate_faces(poly[0],poly[1])]
;

function triangulate_faces(points,faces)=
   flatten([for(i=[0:len(faces)-1]) triangulate_face(points,faces[i])])
;

function triangulate_face(points,face)=
   let(count=len(face))
   (3==count)?
      [face]
   :
      let(wd=face_winding(points,face),cv=convex_vertex(wd,points,face),
          pv=(count+cv-1)%count,nv=(cv+1)%count,
          p0=points[face[pv]],p1=points[face[cv]],p2=points[face[nv]],
          tests=[
             [cross(wd,p0-p2),cross(wd,p0-p2)*p0],
             [cross(wd,p1-p0),cross(wd,p1-p0)*p1],
             [cross(wd,p2-p1),cross(wd,p2-p1)*p2]
          ],
          eartest=point_in_ear(points,face,tests),
          clipableear=(eartest[0]<0),
          diagonalpoint=eartest[1]
       )
       (clipableear)? //There is no point inside the ear.
          flatten([
             [circle_cut(face,pv,nv)],
             triangulate_face(points,circle_cut(face,nv,pv))
          ])
       : //If there is a point inside the ear, make a diagonal and clip along that.
          flatten([
             triangulate_face(points,circle_cut(face,cv,diagonalpoint)),
             triangulate_face(points,circle_cut(face,diagonalpoint,cv))
          ])
;

function vsum(v_list,i=0)=
  (i<len(v_list)-1)?
     v_list[i]+vsum(v_list,i+1)
  :
     [0,0,0]
;

function face_winding(points,face)=
   let(count=len(face))
   vsum([
      for(i=[0:count-3]) cross(
         points[face[(i+1)]]-points[face[0]],
         points[face[(i+2)]]-points[face[(i+1)]]
      )
   ])
;

function convex_vertex(wd,points,face,i=0)=
   let(count=len(face)-1,
       p0=points[face[i]],p1=points[face[(i+1)%count]],p2=points[face[(i+2)%count]]
   )
   (len(face)>i)?
      (cross(p1-p0,p2-p1)*wd>0)? 
         (i+1)%count
      :
         convex_vertex(wd,points,face,i+1)
   ://This should never happen since there is at least 1 convex vertex.
      undef
;

//The order of tests matters - This assumes Test 0 indicates which point is furthest in 
//the ear - and that point will form a diagonal with the tip of the ear.

function point_in_ear(points,face,tests,i=0)=
    (i<len(face)-1)?
       let(
           prev=point_in_ear(points,face,tests,i+1),
           test=check_point_in_ear(points[face[i]],tests)
       )
       (test>prev[0])?
          [test,i]
       :
          prev
    :
       [check_point_in_ear(points[face[i]],tests),i]
;

function check_point_in_ear (point,tests)=
   let(
      result=[
         (point*tests[0][0])-tests[0][1],
         (point*tests[1][0])-tests[1][1],
         (point*tests[2][0])-tests[2][1]
      ]
   )
   (result[0]>0 && result[1]>0 && result[2]>0)?
      result[0]
   :
      -1
;







