use<functional.scad>

/* Rounded cube using the cube/octahedron duality. */

function rounded_cube(size, r=0, center=false, fn=12)=
   (!is_array(size))?
     rounded_cube(size=[size,size,size],r=r,center=center,fn=fn)
   :(r>=min(size[0],size[1],size[2])/2)?
      rounded_cube(size=size,r=min(size[0],size[1],size[2])/2-.0001,center=center,fn=fn)
   :(r<=0)?
      cube(size,center=center)
   :
      let(
         steps=(fn>0)?ceil(fn/4):1,
         sx=size[0]/2-r,
         sy=size[1]/2-r,
         sz=size[2]/2-r,
         da=90/steps,
         count=4*(steps+1)*(steps+2)-1,
         rcube=[
              flatten(concat(
                  [for (i=[0:steps])
                      let(
                         z=r*(-cos(da*i))-sz,
                         l=r*sin(da*i),
                         dp=(i>0)?(90/i):0
                      )
                      concat(
                         [for(j=[0:i]) [sx+l*cos(dp*j),sy+l*sin(dp*j),z]],
                         [for(j=[0:i]) [-sx-l*sin(dp*j),sy+l*cos(dp*j),z]],
                         [for(j=[0:i]) [-sx-l*cos(dp*j),-sy-l*sin(dp*j),z]],
                         [for(j=[0:i]) [sx+l*sin(dp*j),-sy-l*cos(dp*j),z]]
                      )
                  ],
                  [for (i=[steps:-1:0])
                      let(
                         z=sz+r*cos(da*i),
                         l=r*sin(da*i),
                         dp=(i>0)?90/i:0
                      )
                      concat(
                         [for(j=[0:i]) [sx+l*cos(dp*j),sy+l*sin(dp*j),z]],
                         [for(j=[0:i]) [-sx-l*sin(dp*j),sy+l*cos(dp*j),z]],
                         [for(j=[0:i]) [-sx-l*cos(dp*j),-sy-l*sin(dp*j),z]],
                         [for(j=[0:i]) [sx+l*sin(dp*j),-sy-l*cos(dp*j),z]]
                      )
                  ]
            )),
//Each side on the i'th row has i+1 points
//The rows start with 0,4,8,12 ... 2*i*(i+1)
//Each side on the ith row has i+1 points
                 concat(
                    [[0,1,2,3]],
                    [for(i=[1:steps]) let(start=2*i*(i+1),laststart=2*i*(i-1),
                                          ppl=4*(i+1),lppl=4*i,
                                          pps=(i+1),lpps=i
                                          )
                       for(side=[0:3])
                       [
                          laststart+lpps*side,
                          laststart+(lpps*side+lppl-1)%lppl,
                          start+(pps*side+ppl-1)%ppl,
                          start+pps*side
                       ]
                    ],
                    [for(i=[1:steps]) let(start=2*i*(i+1),laststart=2*i*(i-1),
                                          ppl=4*(i+1),lppl=4*i,
                                          pps=(i+1),lpps=i
                                          )
                       for(side=[0:3]) for(j=[0:i-1])
                       [
                          laststart+lpps*side+j,
                          start+pps*side+j,
                          start+pps*side+1+j
                       ]
                    ],
                    (steps<2)?[]: [for(i=[2:steps]) let(start=2*i*(i+1),laststart=2*i*(i-1),
                                          ppl=4*(i+1),lppl=4*i,
                                          pps=(i+1),lpps=i
                                          )
                       for(side=[0:3]) for(j=[0:i-2])
                       [
                          laststart+lpps*side+j,
                          start+pps*side+j+1,
                          laststart+lpps*side+j+1
                       ]
                    ],
//These faces fuse the top & bottom
//The steps'th row starts with 2*steps*(steps+1)
//The (steps+1)'th row starts with 2*steps*(steps+1)+4*steps
                    [for(side=[0:3]) let(start=2*steps*(steps+1)+4*(steps+1),
                                          laststart=2*steps*(steps+1),
                                          ppl=4*(steps+1),lppl=4*(steps+1),
                                          pps=(steps+1),lpps=(steps+1)
                                          ) for(j=[0:steps])
                       
                       [
                          laststart+lpps*side+j,
                          laststart+(lpps*side+lppl-1+j)%lppl,
                          start+(pps*side+ppl-1+j)%ppl,
                          start+pps*side+j
                       ]
                    ],
//And the top is just the bottom wound in reverse.
                    [[count-0,count-1,count-2,count-3]],
                    [for(i=[1:steps]) let(start=2*i*(i+1),laststart=2*i*(i-1),
                                          ppl=4*(i+1),lppl=4*i,
                                          pps=(i+1),lpps=i
                                          )
                       for(side=[0:3])
                       [
                          count-(laststart+lpps*side),
                          count-(laststart+(lpps*side+lppl-1)%lppl),
                          count-(start+(pps*side+ppl-1)%ppl),
                          count-(start+pps*side)
                       ]
                    ],
                    [for(i=[1:steps]) let(start=2*i*(i+1),laststart=2*i*(i-1),
                                          ppl=4*(i+1),lppl=4*i,
                                          pps=(i+1),lpps=i
                                          )
                       for(side=[0:3]) for(j=[0:i-1])
                       [
                          count-(laststart+lpps*side+j),
                          count-(start+pps*side+j),
                          count-(start+pps*side+1+j)
                       ]
                    ],
                    (steps<2)?[]: [for(i=[2:steps]) let(start=2*i*(i+1),laststart=2*i*(i-1),
                                          ppl=4*(i+1),lppl=4*i,
                                          pps=(i+1),lpps=i
                                          )
                       for(side=[0:3]) for(j=[0:i-2])
                       [
                          count-(laststart+lpps*side+j),
                          count-(start+pps*side+j+1),
                          count-(laststart+lpps*side+j+1)
                       ]
                    ]
                 )
          ]
   )
   center?rcube:translate(size/2,rcube)
;

rcube=(rounded_cube([3,2,1],1/3,fn=24,center=false));
poly3d(rcube);

