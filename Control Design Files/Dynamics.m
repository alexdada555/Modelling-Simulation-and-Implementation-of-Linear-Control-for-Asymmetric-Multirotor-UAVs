close all; % close all figures
clear;     % clear workspace variables
clc;       % clear command window

%% Mass of the Multirotor in Kilograms as taken from the CAD

M = 1.455882; 
g = 9.81;

%% Dimensions of Multirotor

L1 = 22/100; % along X-axis Distance from left and right motor pair to center of mass
L2 = 15/100; % along Y-axis Vertical Distance from left and right motor pair to center of mass
L3 = 30/100; % along Y-axis Distance from motor pair to center of mass

%%  Mass Moment of Inertia as Taken from the CAD

Ixx = 2.820E+05;
Ixy = 19.798;
Ixz = 211.315;
Iyx = 19.798;
Iyy = 3.858E+05;
Iyz = -294.238;
Izx = 211.315;
Izy = -294.238;
Izz = 1.375E+05;

%% Inertia Matrix and Diagolalisation CAD model coordinate system rotated 90 degrees about X

I = [Izz Izy Izx;
     Ixy Ixx Ixz;
     Iyx Iyz Iyy];
 
Idiag = diag(I)/10000;
 
Ixx = Idiag(1);
Iyy = Idiag(2);
Izz = Idiag(3);

%% Motor Thrust and Torque Constants (To be determined experimentally)

Kw = 0.85;
Ktau =  4.46*(10^-8);
Kthrust =  3.7155*(10^-7);
Kthrust2 = -3.7327*(10^-4);
Mtau = (1/44.22);
Ku = 515.5;

%% Air resistance damping coeeficients

Dxx = 0.01212;
Dyy = 0.01212;
Dzz = 0.0648;                          

%% Equilibrium Input 

%U_e = sqrt(((M*g)/(3*(Kthrust+(Kw*Kthrust)))));
U_e = ((-6*Kthrust2) + sqrt((6*Kthrust2)^2 - (4*(-M*g)*(3*Kw*Kthrust + 3*Kthrust))))/(2*(3*Kw*Kthrust + 3*Kthrust));

