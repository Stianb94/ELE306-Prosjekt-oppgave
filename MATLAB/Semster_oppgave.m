
clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%DH%%%%%%%%%%%%%%%%%%
D1 = 0.2;L2 = 0.5;L3 = 0.5;L4 = 0.25;D2 = 0.2;

L(1) = Link('revolute', 'd', D1, 'a', 0, 'alpha', -pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha',0);

robot = SerialLink(L,'name', 'open manipulator')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%FOREADR KINEMATIKKS%%

syms L1 L2 L3 L4  q1 q2 q3 q4 d1 Q1 a1 a2 a3 a4
q1 =0, q2=-0; q3=0; q4 = 0; d1 = 0.2; a2 = 0.5; a3 = 0.5; a4 = 0.25; 
T1 = [cos(q1) 0 -sin(q1) 0; sin(q1) 0 cos(q1) 0; 0 -1 0 (d1); 0 0 0 1]
T2 = [cos(q2) -sin(q2) 0 a2*cos(q2); sin(q2) cos(q2) 0 a2*sin(q2); 0 0 1 0; 0 0 0 1]
T3 = [cos(q3) -sin(q3) 0 a3*cos(q3); sin(q3) cos(q3) 0 a3*sin(q3); 0 0 1 0; 0 0 0 1]
T4 = [cos(q4) -sin(q4) 0 a4*cos(q4); sin(q4) cos(q4) 0 a4*sin(q4); 0 0 1 0; 0 0 0 1]
T = T1*T2*T3*T4

%%MED PETER CORCK TOOLBOX%%


D1 = 0.2;L2 = 0.5;L3 = 0.5;L4 = 0.25;

L(1) = Link('revolute', 'd', D1, 'a', 0, 'alpha', -pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha',0);

robot = SerialLink(L,'name', 'open manipulator');
Tn = robot.fkine([0 0 0 0]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%invers%%

M = [0 -1 0 0.6; 0 0 1 0; -1 0 0 -0.3; 0 0 0 1];

q0 = robot.ikine(M,'mask', [1 1 1 0 0 1]);
rad2deg(q0)


%%%%%%%%%%%%%%%%%%%%%%%%%%
%%jacobian%%
syms L1 L2 L3 L4 d1 a1 theta1 theta2 theta3 theta4 

theta1 =0;
theta2=-0.4684; 
theta3=1.7264; 
theta4 = 0.3128; 

d1 = 0.2; L2 = 0.5; L3 = 0.5; L4 = 0.25; a = -pi/2
% Define the links for the robot from the DH table parameters
L(1) = Link('revolute', 'd', d1, 'a', 0, 'alpha', a);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha', 0);



%Transformasjonsmatrise ymbolsk 
robot=SerialLink(L,'name','My Robot')
T1 = robot.fkine([theta1 theta2 theta3 theta4])



T_04 = robot.fkine([theta1 theta2 theta3 theta4])





J =robot.jacobe([theta1 theta2 theta3 theta4])
j_sub_1 = J([1,2,3,6],:)
det(j_sub_1)
rank(j_sub_1)
xdot = J*[0.1;0.2;0.3;0.4]


J0 =robot.jacob0([theta1 theta2 theta3 theta4])
j_sub = J0([1,2,3,5],:)
det(j_sub)
rank(j_sub)
jsingu(j_sub)
xdot0 = J0*[0.1;0.2;0.3;0.4]


qd = inv(j_sub)*[0.1,0.2,0.3,0.4]'


%velocity ellipsoids

robot.vellipse([theta1 theta2 theta3 theta4])
robot.teach([theta1 theta2 theta3 theta4], 'callback', @(r, q) r.vellipse(q), 'view', 'top')
m = robot.maniplty([theta1 theta2 theta3 theta4]);

%%force and tor

mdl_planar2
wo = [0;0;20;0;0;0];
q = robot.jacob0([theta1 theta2 theta3 theta4])'*wo
robot.fellipse([theta1 theta2 theta3 theta4]);
robot.teach([theta1 theta2 theta3 theta4], 'callback', @(r, q) r.vellipse(q), 'view', 'top')
