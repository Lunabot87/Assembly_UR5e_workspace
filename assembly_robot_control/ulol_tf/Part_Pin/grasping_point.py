from math import pi

P1grasp={}

P2grasp1={'trans':[0.1293,0,0.02],'rot':[0,pi,pi/2]}
P2grasp2={'trans':[-0.1293,0,0.02],'rot':[0,pi,pi/2]}
P2grasp3={'trans':[0,0,0.01],'rot':[0,pi,pi/2]}
P2grasp = [P2grasp1,P2grasp2,P2grasp3]

P3grasp1={'trans':[0.1117,0,0.01],'rot':[0,pi,pi/2]}
P3grasp2={'trans':[-0.1117,0,0.01],'rot':[0,pi,pi/2]}
P3grasp3={'trans':[0,0,0.01],'rot':[0,pi,pi/2]}
# P3grasp3={'trans':[0.12,0,0.01],'rot':[0,pi,pi/2]}
# P3grasp4={'trans':[-0.12,0,0.01],'rot':[0,pi,pi/2]}
P3grasp = [P3grasp1,P3grasp2,P3grasp3]

P4grasp1={'trans':[-0.094, 0.000, 0.005],'rot':[0, 0,-pi/2]}
P4grasp2={'trans':[ 0.028,-0.295, 0.005],'rot':[0, 0, pi/2]}
P4grasp3={'trans':[ 0.094, 0.000, 0.005],'rot':[0, 0, pi/2]}
P4grasp4={'trans':[-0.094,-0.295, 0.005],'rot':[pi, 0,-pi/2]}
P4grasp = [P4grasp1,P4grasp2,P4grasp3,P4grasp4]

P5grasp1={'trans':[0.123621,0.405,0.011],'rot':[0.09,0,-pi/2]}
P5grasp2={'trans':[0.378777,0.435449,0.04],'rot':[0,0,-0.11]}
P5grasp3={'trans':[0.245913,0.4,0.027],'rot':[0.09,0,-pi/2]}
P5grasp4={'trans':[0.383863,0.26097,0.04],'rot':[0,0,0.11+pi]}
P5grasp5={'trans':[0.405777,0.735449,0.04],'rot':[0,0,-0.11]}
P5grasp = [P5grasp1,P5grasp2,P5grasp3,P5grasp4,P5grasp5]

P6grasp1={'trans':[0.388863,0.18097,-0.04],'rot':[0,0,0.08+pi]}
P6grasp2={'trans':[0.000000,0.06000, 0.00],'rot':[0,0,pi]}
P6grasp3={'trans':[0.123621,0.40500,-0.011],'rot':[0.09,0,pi/2]}
P6grasp4={'trans':[0.378777,0.45545,-0.04],'rot':[0,0,-0.11]}
P6grasp5={'trans':[0.245913,0.40000,-0.022],'rot':[0.09,0,pi/2]}
P6grasp = [P6grasp1,P6grasp2,P6grasp3,P6grasp4,P6grasp5]

grasping_pose = [P1grasp,P2grasp,P3grasp,P4grasp,P5grasp,P6grasp]
