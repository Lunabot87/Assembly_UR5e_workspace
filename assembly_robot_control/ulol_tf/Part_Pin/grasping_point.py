from math import pi

P1grasp={}

P2grasp1={'trans':[0.0793,0,0.01],'rot':[0,pi,pi/2]}
P2grasp2={'trans':[-0.0793,0,0.01],'rot':[0,pi,pi/2]}
P2grasp = [P2grasp1,P2grasp2]

P3grasp1={'trans':[0.0617,0,0.01],'rot':[0,pi,pi/2]}
P3grasp2={'trans':[-0.0617,0,0.01],'rot':[0,pi,pi/2]}
P3grasp = [P3grasp1,P3grasp2]

P4grasp={}

P5grasp1={'trans':[0.3865863,0.26097,0.035],'rot':[0,pi,0.11]}
P5grasp2={'trans':[0.398777,0.635449,0.035],'rot':[0,pi,-0.11]}
P5grasp3={'trans':[0.123621,0.405,0.011],'rot':[0.09,pi,pi/2]}
P5grasp4={'trans':[0.245913,0.4,0.022],'rot':[0.09,pi,pi/2]}
P5grasp = [P5grasp1,P5grasp2,P5grasp3,P5grasp4]

P6grasp1={'trans':[0.3885863,0.26097,-0.035],'rot':[0,0,0.11]}
P6grasp2={'trans':[0.397777,0.635449,-0.035],'rot':[0,0,-0.11]}
P6grasp3={'trans':[0.123621,0.405,-0.011],'rot':[0.09,0,pi/2]}
P6grasp4={'trans':[0.245913,0.4,-0.022],'rot':[0.09,0,pi/2]}
P6grasp = [P6grasp1,P6grasp2,P6grasp3,P6grasp4]

grasping_pose = [P1grasp,P2grasp,P3grasp,P4grasp,P5grasp,P6grasp]