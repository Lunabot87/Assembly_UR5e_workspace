from math import pi, acos
from numpy import dot, array
from numpy.linalg import norm
P1hole_offset =[]

P2hole1={'trans':[0.17,0.016,0.01],'rot':[0,-pi/2,0]}
P2hole2={'trans':[0.17,-0.016,0.01],'rot':[0,-pi/2,0]}
P2hole3={'trans':[-0.17,0.016,0.01],'rot':[0,-pi/2,pi]}
P2hole4={'trans':[-0.17,-0.016,0.01],'rot':[0,-pi/2,pi]}
P2hole5={'trans':[0.112,-0.0145,0.02],'rot':[pi,0,pi]}
P2hole6={'trans':[-0.112,-0.0145,0.02],'rot':[pi,0,pi]}
P2hole7={'trans':[0.17,0,0.01],'rot':[0,-pi/2,0]}
P2hole8={'trans':[-0.17,-0.0,0.01],'rot':[0,pi/2,0]}

P2hole_offset=[P2hole1,P2hole2,P2hole3,P2hole4,
				P2hole5,P2hole6,P2hole7,P2hole8]

P3hole1={'trans':[ 0.125+0.0065, 0.0165, 0.01],'rot':[pi,pi/2,0]}
P3hole2={'trans':[ 0.125+0.0065,-0.0165, 0.01],'rot':[pi,pi/2,0]}
P3hole3={'trans':[-0.125-0.0065, 0.0165, 0.01],'rot':[0,pi/2,0]}
P3hole4={'trans':[-0.125-0.0065,-0.0165, 0.01],'rot':[0,pi/2,0]}
P3hole5={'trans':[ 0.080	   ,-0.013, 0.02],'rot':[pi,0,pi]}
P3hole6={'trans':[-0.080	   ,-0.013, 0.02],'rot':[pi,0,pi]}
P3hole7={'trans':[-0.115-0.02  ,-0.0000, 0.01],'rot':[0,pi/2,0]}
P3hole8={'trans':[ 0.115+0.02  ,-0.0000, 0.01],'rot':[0,-pi/2,0]}

P3hole_offset=[P3hole1,P3hole2,P3hole3,P3hole4,
				P3hole5,P3hole6,P3hole7,P3hole8]

P4hole1={'trans':[ 0.1275+0.0065, 0.0165, 0.01],'rot':[-pi/2,pi/2,pi/2]}
P4hole2={'trans':[ 0.1275+0.0065,-0.0165, 0.01],'rot':[-pi/2,pi/2,pi/2]}
P4hole3={'trans':[-0.1275-0.0065, 0.0165, 0.01],'rot':[0,pi/2,0]}
P4hole4={'trans':[-0.1275-0.0065,-0.0165, 0.01],'rot':[0,pi/2,0]}
P4hole5={'trans':[-0.1275-0.0065,-0.294, 0.01],'rot':[0,pi/2,0]}
P4hole6={'trans':[ 0.1275+0.0065,-0.294, 0.003],'rot':[-pi/2,pi/2,pi/2]}
P4hole7={'trans':[-0.1150-0.0200, 0.000, 0.005],'rot':[0,pi/2,0]}
P4hole8={'trans':[ 0.1150+0.0200, 0.000, 0.005],'rot':[-pi/2,pi/2,pi/2]}

P4hole_offset=[P4hole1,P4hole2,P4hole3,P4hole4,
				P4hole5,P4hole6,P4hole7,P4hole8]

# P5hole1={'trans':[0.002166, 0.426660, 0.005],'rot':[0,pi,pi/2]}
# P5hole2={'trans':[0.000492, 0.394704, 0.005],'rot':[0,pi,pi/2]}
# P5hole3={'trans':[0.369042, 0.407433, 0.040],'rot':[0,pi,pi/2]}
# P5hole4={'trans':[0.367368, 0.375477, 0.040],'rot':[0,pi,pi/2]}
# P5hole5={'trans':[0.393244, 0.567391, 0.035],'rot':[0,pi,pi/2]}
# P5hole6={'trans':[0.427122, 0.844371, 0.035],'rot':[0,pi,pi/2]}
# P5hole7={'trans':[0.431575, 0.876060, 0.035],'rot':[0,pi,pi/2]}
# P5hole8={'trans':[0.001329, 0.410682,-0.015],'rot':[0,0,pi/2]}
# P5hole9={'trans':[0.369205, 0.391455, 0.020],'rot':[0,0,pi/2]}
# P5hole10={'trans':[0.429349, 0.860216, 0.020],'rot':[0,0,pi/2]}

P5hole1={'trans':[0.002166, 0.425, 0.005],'rot':[0,-pi,pi/2]}
P5hole2={'trans':[0.000492, 0.392, 0.005],'rot':[0,-pi,pi/2]}
P5hole3={'trans':[0.370, 0.41203, 0.040],'rot':[-pi,0,0]}
P5hole4={'trans':[0.370, 0.37903, 0.040],'rot':[-pi,0,0]}
P5hole5={'trans':[0.37947, 0.57127, 0.035],'rot':[0,pi,pi/2]}
P5hole6={'trans':[0.43931, 0.84512, 0.035],'rot':[0,pi,pi/2]}
P5hole7={'trans':[0.44464, 0.87769, 0.035],'rot':[0,pi,pi/2]}
P5hole8={'trans':[0.001329, 0.4085,-0.015],'rot':[0,0,0]}
P5hole9={'trans':[0.370, 0.39553, 0.020],'rot':[0,0,pi/2]}
P5hole10={'trans':[0.44198, 0.8614, 0.020],'rot':[0,0,pi/2]}

P5hole_offset=[P5hole1,P5hole2,P5hole3,P5hole4,P5hole5,
				P5hole6,P5hole7,P5hole8,P5hole9,P5hole10]

# P6hole1={'trans':[0.002166, 0.426660,-0.015],'rot':[0,0,0]}
# P6hole2={'trans':[0.000492, 0.394704,-0.015],'rot':[0,0,0]}
# P6hole3={'trans':[0.369042, 0.407433,-0.050],'rot':[0,0,0]}
# P6hole4={'trans':[0.367368, 0.375477,-0.050],'rot':[0,0,0]}
# P6hole5={'trans':[0.393244, 0.567391,-0.050],'rot':[0,0,0]}
# P6hole6={'trans':[0.427122, 0.844371,-0.050],'rot':[0,0,0]}
# P6hole7={'trans':[0.431575, 0.876060,-0.050],'rot':[0,0,0]}
# P6hole8={'trans':[0.001329, 0.410682,-0.015],'rot':[0,pi,0]}
# P6hole9={'trans':[0.369205, 0.391455,-0.050],'rot':[0,pi,0]}
# P6hole10={'trans':[0.429349, 0.860216,-0.050],'rot':[0,pi,0]}

P6hole1={'trans':[0.002166, 0.425,-0.015],'rot':[0,0,0]}
P6hole2={'trans':[0.000492, 0.392,-0.015],'rot':[0,0,0]}
P6hole3={'trans':[0.370, 0.41203,-0.050],'rot':[0,0,0]}
P6hole4={'trans':[0.370, 0.37903,-0.050],'rot':[0,0,0]}
P6hole5={'trans':[0.37947, 0.57127,-0.050],'rot':[0,0,0]}
P6hole6={'trans':[0.43931, 0.84512,-0.050],'rot':[0,0,0]}
P6hole7={'trans':[0.44464, 0.87769,-0.050],'rot':[0,0,0]}
P6hole8={'trans':[0.001329, 0.4085,-0.015],'rot':[0,pi,0]}
P6hole9={'trans':[0.370, 0.39553,-0.050],'rot':[0,pi,0]}
P6hole10={'trans':[0.44198, 0.8614,-0.050],'rot':[0,pi,0]}

theta = acos( dot(array(P6hole7['trans'])-array(P6hole6['trans']),[0,1,0]) / norm(array(P6hole7['trans'])-array(P6hole6['trans'])))
P6hole5['rot'][2] = -theta
P6hole6['rot'][2] = -theta
P6hole7['rot'][2] = -theta
P6hole10['rot'][2] = -theta


P6hole_offset=[P6hole1,P6hole2,P6hole3,P6hole4,P6hole5,
				P6hole6,P6hole7,P6hole8,P6hole9,P6hole10]

hole_offset = [P1hole_offset,P2hole_offset,P3hole_offset,
				P4hole_offset,P5hole_offset,P6hole_offset]


pin122620_hole = {'trans':[0.0,-0.01,-0.01],'rot':[-pi/2,0,pi]}

pin_hole_offset = [{},{},pin122620_hole,{}]