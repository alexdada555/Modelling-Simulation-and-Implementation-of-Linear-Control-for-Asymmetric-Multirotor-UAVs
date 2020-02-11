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
Ktau =  7.708e-10;
Kthrust =  1.812e-07;%3.7155*(10^-7);
Kthrust2 = 0.0007326;%-3.7327*(10^-4);
Mtau = (1/44.22);
Ku = 515.5;

%% Air resistance damping coeeficients

Dxx = 0.01212;
Dyy = 0.01212;
Dzz = 0.0648;                          

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

%% Define Discrete-Time BeagleBone Dynamics

T = 0.010; % Sample period (s)- 100Hz

%% Discrete-Time Full State-Feedback Control
% State feedback control design (integral control via state augmentation)
% Define augmented system matrices Z pitch roll and yaw are controlled outputs
% Define LQR weighting matrices

Cr  = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0];    

r = 4;                                % number of reference inputs
n = size(A,2);                        % number of states
q = size(Cr,1);                       % number of controlled outputs

Dr = zeros(q,6);

Aaug = [A zeros(n,r); 
       -Cr zeros(q,r)];
Baug = [B; -Dr];
Caug = [C zeros(r,r)];

sys = ss(A,B,C,0);
sysd = c2d(sys,T);

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;

%Qx = diag([1,1,0.001,5,1,1,1,1,0,0,0,0,0,0]);
%Qu = (1*10^-7)*eye(6);

Qx = diag([1,10^-3,10,100,10,100,10,100,0,0,0,0,0,0,0.1,10^-10,10^-10,10^-10]); % State penalty
Qu = 1*10^-7*eye(6,6);  % Control penalty

Kdtaug = lqrd(Aaug,Baug,Qx,Qu,T); % DT State-Feedback Controller Gains

Kdt = Kdtaug(:,1:n); 

Kpdt = [Kdt(:,1) Kdt(:,3) Kdt(:,5) Kdt(:,7)];
Kddt = [Kdt(:,2) Kdt(:,5) Kdt(:,6) Kdt(:,8)];
Kidt = -Kdtaug(:,n+1:end);

Kr = -1*((Cr*((A - B*Kdt)\B)).^-1)/6;%eye(6,4);

%% Output and Motor Mix Matrix

K = [1, 1, -1, 1;
     1, 1, -1, -1;
     1, -1, -1, -1; % Motor Mixer
     1, -1, -1, 1;
     1, 0, 1, 1;
     1, 0, 1, -1];

 %% Discrete-Time Kalman Filter Design x_dot = A*x + B*u + G*w, y = C*x + D*u + H*w + v

sysdt = c2d(ss(A,B,C,D),T,'zoh');  % Generate Discrete-Time System

Adt = sysdt.a; 
Bdt = sysdt.b; 
Cdt = sysdt.c; 
Ddt = sysdt.d;

n = size(A,2); 
Gdt = 1e-1*eye(n);

Hdt = zeros(size(C,1),size(Gdt,2)); % No process noise on measurements

Rw = eye(14,14);   % Process noise covariance matrix

Rv = eye(4)*1e-5;     % Measurement noise covariance matrix Note: use low gausian noice for Rv

N = zeros(size(Rw,2),size(Rv,2));

sys4kf = ss(Adt,[Bdt Gdt],Cdt,[Ddt Hdt],T);

[kdfilt,Ldt] = kalman(sys4kf,Rw,Rv); 
 