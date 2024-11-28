clc;
clear;
close all;
import ETS3.*
%%
% Defining the robotic arm
L1 = 0.2;
L2 = 0.5;
L3 = 0.5;
L4 = 0.2;


%% No offsets
L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', -pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha', 0);
robot = SerialLink(L,'name', 'Prosjekt_arm');

%robot.plot([0, 0, 0, 0])

%hold on

%Start posisjon
T_t = robot.fkine([deg2rad(0) deg2rad(-82.8) deg2rad(129.6) deg2rad(43)])
T0 = robot.fkine([deg2rad(-90) deg2rad(-82.8) deg2rad(129.6) deg2rad(43)])
%slutt posisjon i x,y,z kordinater med en 90 grader rotasjon på end-faktor
T_d = SE3(0.6, 0, -0.3) *SE3.Ry(90,'deg')
%invers kinematik for start
qt =robot.ikine(T_t,'mask',[1 1 1 0 0 1])

q0 = robot.ikine(T0,'mask',[1 1 1 0 0 1])
%invers kinematik av slutt
qd = robot.ikine(T_d,'mask',[1 1 1 0 0 1])
%bane til slutt
qj3 = jtraj(q0, qt , 50)
figure()
robot.plot(qj3)
hold on
qj = jtraj(qt, qd , 50)

robot.plot(qj)

%bane tilbake til start
qj4 = jtraj(qd, qt , 50)
robot.plot(qj4)

qj2 = jtraj(qt, q0 , 50)
robot.plot(qj2)

% q1 = [pi 0.3 -pi/2 -pi/2]
% T1 = robot.fkine(q1)
%robot.teach
