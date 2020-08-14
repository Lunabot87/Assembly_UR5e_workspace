from math import pi

P1hole_offset =[]

P2hole1={'trans':[0.1585,0.016,0.01],'rot':[0,pi/2,0]}
P2hole2={'trans':[0.1585,-0.016,0.01],'rot':[0,pi/2,0]}
P2hole3={'trans':[0.1585,0,0.01],'rot':[0,pi/2,0]}
P2hole4={'trans':[-0.1585,0.016,0.01],'rot':[0,pi/2,0]}
P2hole5={'trans':[-0.1585,-0.016,0.01],'rot':[0,pi/2,0]}
P2hole6={'trans':[-0.1585,-0.0,0.01],'rot':[0,pi/2,0]}
P2hole7={'trans':[0.112,-0.0145,0.0135],'rot':[0,0,0]}
P2hole8={'trans':[-0.112,-0.0145,0.0135],'rot':[0,0,0]}

P2hole_offset=[P2hole1,P2hole2,P2hole3,P2hole4,
				P2hole5,P2hole6,P2hole7,P2hole8]

P3hole1={'trans':[0.1235,0.016,0.01],'rot':[0,pi/2,0]}
P3hole2={'trans':[0.1235,-0.016,0.01],'rot':[0,pi/2,0]}
P3hole3={'trans':[0.1235,-0,0.01],'rot':[0,pi/2,0]}
P3hole4={'trans':[-0.1235,0.016,0.01],'rot':[0,pi/2,0]}
P3hole5={'trans':[-0.1235,-0.016,0.01],'rot':[0,pi/2,0]}
P3hole6={'trans':[-0.1235,-0.0,0.01],'rot':[0,pi/2,0]}
P3hole7={'trans':[0.08,-0.0145,0.0135],'rot':[0,0,0]}
P3hole8={'trans':[-0.08,-0.0145,0.0135],'rot':[0,0,0]}

P3hole_offset=[P3hole1,P3hole2,P3hole3,P3hole4,
				P3hole5,P3hole6,P3hole7,P3hole8]

P4hole1={'trans':[0.13,0.0,0.01],'rot':[0,pi/2,0]}
P4hole2={'trans':[0.13,0.016,0.01],'rot':[0,pi/2,0]}
P4hole3={'trans':[0.13,-0.016,0.01],'rot':[0,pi/2,0]}
P4hole4={'trans':[-0.13,0.0,0.01],'rot':[0,pi/2,0]}
P4hole5={'trans':[-0.13,0.016,0.01],'rot':[0,pi/2,0]}
P4hole6={'trans':[-0.13,-0.016,0.01],'rot':[0,pi/2,0]}
P4hole7={'trans':[-0.13,-0.295,0.005],'rot':[0,pi/2,0]}
P4hole8={'trans':[0.13,-0.295,0.005],'rot':[0,pi/2,0]}

P4hole_offset=[P4hole1,P4hole2,P4hole3,P4hole4,
				P4hole5,P4hole6,P4hole7,P4hole8]

P5hole1={'trans':[0.002166,0.42666,0.0115],'rot':[0,0,0]}
P5hole2={'trans':[0.001329,0.410682,0.0115],'rot':[0,0,0]}
P5hole3={'trans':[0.000492,0.394704,0.0115],'rot':[0,0,0]}
P5hole4={'trans':[0.369042,0.407433,0.06],'rot':[0,0,0]}
P5hole5={'trans':[0.368205,0.391455,0.06],'rot':[0,0,0]}
P5hole6={'trans':[0.367368,0.375477,0.06],'rot':[0,0,0]}
P5hole7={'trans':[0.393244,0.567391,0.06],'rot':[0,0,0]}
P5hole8={'trans':[0.427122,0.844371,0.06],'rot':[0,0,0]}
P5hole9={'trans':[0.431575,0.87606,0.019],'rot':[0,0,0]}
P5hole10={'trans':[0.429349,0.860216,0.0365],'rot':[0,0,0]}

P5hole_offset=[P5hole1,P5hole2,P5hole3,P5hole4,P5hole5,
				P5hole6,P5hole7,P5hole8,P5hole9,P5hole10]

P6hole1={'trans':[0.002166,0.42666,-0.0115],'rot':[-3.14,0,1.57]}
P6hole2={'trans':[0.001329,0.410682,-0.0115],'rot':[-3.14,0,1.57]}
P6hole3={'trans':[0.000492,0.394704,-0.0115],'rot':[-3.14,0,1.57]}
P6hole4={'trans':[0.369042,0.407433,-0.045],'rot':[-3.14,0,1.57]}
P6hole5={'trans':[0.368205,0.391455,-0.045],'rot':[-3.14,0,1.57]}
P6hole6={'trans':[0.367368,0.375477,-0.045],'rot':[-3.14,0,1.57]}
P6hole7={'trans':[0.393244,0.567391,-0.045],'rot':[-3.14,0,1.57]}
P6hole8={'trans':[0.427122,0.844371,-0.045],'rot':[-3.14,0,1.57]}
P6hole9={'trans':[0.431575,0.87606,-0.045],'rot':[-3.14,0,1.57]}
P6hole10={'trans':[0.429349,0.860216,-0.045],'rot':[-3.14,0,1.57]}

P6hole_offset=[P6hole1,P6hole2,P6hole3,P6hole4,P6hole5,
				P6hole6,P6hole7,P6hole8,P6hole9,P6hole10]

hole_offset = [P1hole_offset,P2hole_offset,P3hole_offset,
				P4hole_offset,P5hole_offset,P6hole_offset]


pin122620_hole = {'trans':[0.0,0.0,0.00075],'rot':[0,0,0]}

pin_hole_offset = [{},{},pin122620_hole,{}]