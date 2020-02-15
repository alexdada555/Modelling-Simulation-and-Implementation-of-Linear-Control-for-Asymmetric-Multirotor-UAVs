close all; % close all figures
clear;     % clear workspace variables
clc;       % clear command window
format short;

%% Mass of the Multirotor in Kilograms as taken from the CAD

M = 1.455882; 
g = 9.81;

%% Dimensions of Multirotor

L1 = 22/100; % along X-axis Distance from left and right motor pair to center of mass
L2 = 15/100; % along Y-axis Vertical Distance from left and right motor pair to center of mass
L3 = 30/100; % along Y-axis Distance from motor pair to center of mass

%%  Mass Moment of Inertia as Taken from the CAD
% Inertia Matrix and Diagolalisation CAD model coordinate system rotated 90 degrees about X

Ixx = 0.014;
Iyy = 0.028;
Izz = 0.038;

%% Motor Thrust and Torque Constants (To be determined experimentally)

Kw = 1;
Ktau =  7.708e-10;
Kthrust =  1.812e-07;%3.7155*(10^-7);
Kthrust2 = 0.0007326;%-3.7327*(10^-4);
Mtau = (1/44.22);
Ku = 515.5*Mtau;

%% Air resistance damping coeeficients

Dxx = 0.01212;
Dyy = 0.01212;
Dzz = 0.0648;                          

%% Equilibrium Input 

%W_e = sqrt(((M*g)/(3*(Kthrust+(Kw*Kthrust)))));
W_e = ((-6*Kthrust2) + sqrt((6*Kthrust2)^2 - (4*(-M*g)*(3*Kw*Kthrust + 3*Kthrust))))/(2*(3*Kw*Kthrust + 3*Kthrust));
U_e = (W_e/Ku);

%% Define Discrete-Time BeagleBone Dynamics

T = 0.01; % Sample period (s)- 100Hz

%% Define Linear Continuous-Time Multirotor Dynamics: x_dot = Ax + Bu, y = Cx + Du         

% A =  8x8 matrix
A = [0, 1, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 1, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, 0, 0, 0, 0];
 
% B = 8x6 matrix
B = [0, 0, 0, 0, 0, 0;
     2*Kthrust*W_e*((Ku))/M, 2*Kthrust*W_e*((Ku))/M, 2*Kthrust*W_e*((Ku))/M, 2*Kthrust*W_e*((Ku))/M, 2*Kthrust*W_e*((Ku))/M, 2*Kthrust*W_e*((Ku))/M;
     0, 0, 0, 0, 0, 0;
     2*L1*Kthrust*W_e*((Ku))/Ixx, 2*L1*Kthrust*W_e*((Ku))/Ixx, -2*L1*Kthrust*W_e*((Ku))/Ixx, -2*L1*Kthrust*W_e*((Ku))/Ixx, 0, 0;
     0, 0, 0, 0, 0, 0;
     -2*L2*Kthrust*W_e*((Ku))/Iyy, -2*L2*Kthrust*W_e*((Ku))/Iyy, -2*L2*Kthrust*W_e*((Ku))/Iyy, -2*L2*Kthrust*W_e*((Ku))/Iyy, 2*L3*Kthrust*W_e*((Ku))/Iyy,2*L3*Kthrust*W_e*((Ku))/Iyy;
     0, 0, 0, 0, 0, 0;
     -2*Ktau/Izz*W_e*((Ku)), 2*Ktau*W_e*((Ku))/Izz, 2*Ktau*W_e*((Ku))/Izz, -2*Ktau*W_e*((Ku))/Izz, -2*Ktau*W_e*((Ku))/Izz, 2*Ktau*W_e*((Ku))/Izz];

% C = 4x8 matrix
C = [1, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, 0];
     
% D = 4x6 matrix
D = zeros(4,6);


sysdt = c2d(ss(A,B,C,D),T,'zoh');  % Generate Discrete-Time System

Adt = sysdt.a; 
Bdt = sysdt.b; 
Cdt = sysdt.c; 
Ddt = sysdt.d;

%%
poles = eig(A);

Jpoles = jordan(A);

cntr = rank(ctrb(A,B));
% full controllable 

obs = rank(obsv(A,C));
% partially observable but fully detectable

%% Output and Motor Mix Matrix

Cr  = [1, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 1, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 1, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 1, 0];  
   
K = [1, 1, -1, 1;
     1, 1, -1, -1;
     1, -1, -1, -1; % Motor Mixer
     1, -1, -1, 1;
     1, 0, 1, 1;
     1, 0, 1, -1];

%% Discrete-Time Kalman Filter Design x_dot = A*x + B*u + G*w, y = C*x + D*u + H*w + v

n = size(A,2); 
Gdt = 1e-1*eye(n);
Hdt = zeros(size(C,1),size(Gdt,2)); % No process noise on measurements

Rw =diag([0,1,0,1,0,1,0,1]);   % Process noise covariance matrix
Rv = diag([0.0001,0.001,0.001,0.05]);     % Measurement noise covariance matrix Note: use low gausian noice for Rv
N = zeros(size(Rw,2),size(Rv,2));

sys4kf = ss(Adt,[Bdt Gdt],Cdt,[Ddt Hdt],T);

[kdfilt,Ldt] = kalman(sys4kf,Rw,Rv); 
