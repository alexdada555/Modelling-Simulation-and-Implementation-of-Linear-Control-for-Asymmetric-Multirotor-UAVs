close all; % close all figures
clear;     % clear workspace variables
clc;       % clear command window

%% Mass of the Multirotor in Kilograms as taken from the CAD

M = 2100/1000; 
g = 9.81;

%% Dimensions of Multirotor

L1 = 22/100; % along X-axis Distance from left and right motor pair to center of mass
L2 = 15/100; % along Y-axis Vertical Distance from left and right motor pair to center of mass
L3 = 30/100; % along Y-axis Distance from motor pair to center of mass

%%  Mass Moment of Inertia as Taken from the CAD

Ixx = 3.094E+05;
Ixy = 45.147;
Ixz = 2.309;
Iyx = 45.147;
Iyy = 4.204E+05;
Iyz = 832.833;
Izx = 2.309;
Izy = 832.833;
Izz = 1.464E+05;

%% Inertia Matrix and Diagolalisation CAD model coordinate system rotated 90 degrees about X

I = [Izx Izy Izz;
     Ixx Ixy Ixz;
     Iyx Iyy Iyz];
 
Idiag = diag(I)/10000;
 
Ixx = Idiag(1);
Iyy = Idiag(2);
Izz = Idiag(3);

%% Motor Thrust and Torque Constants (To be determined experimentally)

Kw = 0.85;
Ktau =  1.812e-08;%2.46*(10^-8);
Kthrust =  1.812e-07;%3.7155*(10^-7);
Kthrust2 = 0.0007326; %-3.7327*(10^-4);
Mtau = (1/44.22);
Ku = 515.5;

%% Air resistance damping coeeficients

Dxx = 0.02;
Dyy = 0.02;
Dzz = 0.1;                          

%% Equilibrium Input 

%W_e = sqrt(((M*g)/(3*(Kthrust+(Kw*Kthrust)))));
W_e = ((-6*Kthrust2) + sqrt((6*Kthrust2)^2 - (4*(-M*g)*(3*Kw*Kthrust + 3*Kthrust))))/(2*(3*Kw*Kthrust + 3*Kthrust));
U_e = (W_e/Ku)/Mtau;

%% Define Linear Continuous-Time Multirotor Dynamics in State Space: x_dot = Ax + Bu, y = Cx + Du         

% A =  14x14 matrix
A = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 2*Kthrust*W_e/M, 2*Kthrust*W_e/M, 2*Kthrust*W_e/M, 2*Kthrust*W_e/M, 2*Kthrust*W_e/M, 2*Kthrust*W_e/M;
     0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, L1*2*Kthrust*W_e/Ixx, L1*2*Kthrust*W_e/Ixx, -L1*2*Kthrust*W_e/Ixx, -L1*2*Kthrust*W_e/Ixx, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, -L2*2*Kthrust*W_e/Iyy, -L2*2*Kthrust*W_e/Iyy, -L2*2*Kthrust*W_e/Iyy, -L2*2*Kthrust*W_e/Iyy, L3*2*Kthrust*W_e/Iyy,L3*2*Kthrust*W_e/Iyy;
     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, -2*Ktau*W_e/Izz, 2*Ktau*W_e/Izz, 2*Ktau*W_e/Izz, -2*Ktau*W_e/Izz, -2*Ktau*W_e/Izz, 2*Ktau*W_e/Izz;
     0, 0, 0, 0, 0, 0, 0, 0, -1/Mtau, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, -1/Mtau, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/Mtau, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/Mtau, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/Mtau, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/Mtau];
 
% B = 14x6 matrix
B = [0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     Ku, 0, 0, 0, 0, 0;
     0, Ku, 0, 0, 0, 0;
     0, 0, Ku, 0, 0, 0;
     0, 0, 0, Ku, 0, 0;
     0, 0, 0, 0, Ku, 0;
     0, 0, 0, 0, 0, Ku];

% C = 4x14 matrix
C = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0];
     
% D = 4x6 matrix
D = zeros(4,6);

%% Converstion form State Space to Laplace Domain Transfer Functions

Gss = ss2tf(A,B,C,D,6);


G = tf(Gss);