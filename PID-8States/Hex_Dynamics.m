function [dX]=Hex_Dynamics(t,X,U)
%% Mass of the Multirotor in Kilograms as taken from the CAD

M = 1.455882; 
g = 9.81;

%% Dimensions of Multirotor

L1 = 0.19; % along X-axis Distance from left and right motor pair to center of mass
L2 = 0.18; % along Y-axis Vertical Distance from left and right motor pair to center of mass
L3 = 0.30; % along Y-axis Distance from motor pair to center of mass

%%  Mass Moment of Inertia as Taken from the CAD
% Inertia Matrix and Diagolalisation CAD model coordinate system rotated 90 degrees about X

Ixx = 0.014;
Iyy = 0.028;
Izz = 0.038;

%% Motor Thrust and Torque Constants (To be determined experimentally)

Kw = 0.85;
Ktau =  7.708e-10;
Kthrust =  1.812e-07;
Kthrust2 = 0.0007326;
Mtau = (1/44.22);
Ku = 515.5*Mtau;

%% Air resistance damping coeeficients

Dxx = 0.01212;
Dyy = 0.01212;
Dzz = 0.0648;                          

%% X = [x xdot y ydot z zdot phi p theta q psi r w1 w2 w3 w4 w5 w6]

x = X(1);
xdot = X(2);
y = X(3);
ydot = X(4);
z = X(5);
zdot = X(6);

phi = X(7);
p = X(8);
theta = X(9);
q = X(10);
psi = X(11);
r = X(12);

w1 = X(13);
w2 = X(14);
w3 = X(15);
w4 = X(16);
w5 = X(17);
w6 = X(18);

%% Initialise Outputs

dX = zeros(18,1);
Y = zeros(4,1);

%% Motor Dynamics: dX = [w1dot w2dot w3dot w4dot w5dot w6dot], U = Pulse Width of the pwm signal 0-1000%

dX(13) = -(1/Mtau)*w1 + Ku/Mtau*U(1);
dX(14) = -(1/Mtau)*w2 + Ku/Mtau*U(2);
dX(15) = -(1/Mtau)*w3 + Ku/Mtau*U(3);
dX(16) = -(1/Mtau)*w4 + Ku/Mtau*U(4);
dX(17) = -(1/Mtau)*w5 + Ku/Mtau*U(5);
dX(18) = -(1/Mtau)*w6 + Ku/Mtau*U(6);

%% Motor Forces and Torques

F = zeros(6,1);
T = zeros(6,1);

F(1)= Kw*Kthrust*(w1^2) + Kthrust2*w1;
F(2)= Kthrust*(w2^2) + Kthrust2*w2;
F(3)= Kw*Kthrust*(w3^2)+ Kthrust2*w3;
F(4)= Kthrust*(w4^2) + Kthrust2*w4;
F(5)= Kw*Kthrust*(w5^2) + Kthrust2*w5;
F(6)= Kthrust*(w6^2) + Kthrust2*w6;

T(1)= -Ktau*(w1^2);
T(2)= Ktau*(w2^2);
T(3)= Ktau*(w3^2);
T(4)= -Ktau*(w4^2);
T(5)= -Ktau*(w5^2);
T(6)= Ktau*(w6^2);

Fn = sum(F);
Tn = sum(T);

%% First Order Direvatives dX = [xdot ydot zdot phidot thetadot psidot]

dX(1) = X(2);
dX(3) = X(4);
dX(5) = X(6);
dX(7) = p + q*(sin(phi)*tan(theta)) + r*(cos(phi)*tan(theta));
dX(9) = q*cos(phi) - r*sin(phi);
dX(11) = q*(sin(phi)/cos(theta)) + r*(cos(phi)/cos(theta));

%% Second Order Direvatives: dX = [xddot yddot zddot pdot qdot rdot]

dX(2) = Fn/M*(cos(phi)*sin(theta)*cos(psi)) + Fn/M*(sin(phi)*sin(psi)) - (Dxx/M)*xdot;
dX(4) = Fn/M*(cos(phi)*sin(theta)*sin(psi)) - Fn/M*(sin(phi)*cos(psi)) - (Dyy/M)*ydot;
dX(6) = Fn/M*(cos(phi)*cos(theta)) - g - (Dzz/M)*zdot;

dX(8) = (L1/Ixx)*((F(1)+F(2)) - (F(3)+F(4))) - ((Izz-Iyy)/Ixx)*(r*q); 
dX(10) = (L3/Iyy)*(F(5)+F(6)) - (L2/Iyy)*(F(1)+F(2)+F(3)+F(4)) - ((Izz-Ixx)/Iyy)*(p*r);
dX(12) = Tn/Izz - ((Iyy-Ixx)/Izz)*(p*q);

%% Measured States

Y(1) = z;
Y(2) = phi;
Y(3) = theta;
Y(4) = psi;

end