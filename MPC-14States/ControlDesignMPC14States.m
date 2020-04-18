close all; % close all figures
clear;     % clear workspace variables
clc;       % clear command window
format short;

%% Mass of the Multirotor in Kilograms as taken from the CAD

M = 1.455882; 
g = 9.81;

%% Dimensions of Multirotor

L1 = 0.19; % along X-axis Distance from left and right motor pair to center of mass
L2 = 0.18; % along Y-axis Vertical Distance from left and right motor pair to center of mass
L3 = 0.30; % along Y-axis Distance from motor pair to center of mass

%%  Mass Moment of Inertia as Taken from the CAD

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

%% Equilibrium Input 

%W_e = sqrt(((M*g)/(3*(Kthrust+(Kw*Kthrust)))));
%W_e = ((-6*Kthrust2) + sqrt((6*Kthrust2)^2 - (4*(-M*g)*(3*Kw*Kthrust + 3*Kthrust))))/(2*(3*Kw*Kthrust + 3*Kthrust));
%U_e = (W_e/Ku);
U_e = [176.1,178.5,177.2,177.6,202.2,204.5]';
W_e = U_e*Ku;

%% Define Discrete-Time BeagleBone Dynamics

T = 0.01; % Sample period (s)- 100Hz
ADC = 3.3/((2^12)-1); % 12-bit ADC Quantization
DAC =  3.3/((2^12)-1); % 12-bit DAC Quantization

%% Define Linear Continuous-Time Multirotor Dynamics: x_dot = Ax + Bu, y = Cx + Du         

% A =  14x14 matrix
A = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 2*Kthrust*W_e(1)/M, 2*Kw*Kthrust*W_e(2)/M, 2*Kthrust*W_e(3)/M, 2*Kw*Kthrust*W_e(4)/M, 2*Kthrust*W_e(5)/M, 2*Kw*Kthrust*W_e(6)/M;
     0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, L1*2*Kthrust*W_e(1)/Ixx, L1*2*Kw*Kthrust*W_e(2)/Ixx, -L1*2*Kthrust*W_e(3)/Ixx, -L1*2*Kw*Kthrust*W_e(4)/Ixx, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, -L2*2*Kthrust*W_e(1)/Iyy, -L2*2*Kw*Kthrust*W_e(2)/Iyy, -L2*2*Kthrust*W_e(3)/Iyy, -L2*2*Kw*Kthrust*W_e(4)/Iyy, L3*2*Kthrust*W_e(5)/Iyy,L3*2*Kw*Kthrust*W_e(6)/Iyy;
     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, -2*Ktau*W_e(1)/Izz, 2*Ktau*W_e(2)/Izz, 2*Ktau*W_e(3)/Izz, -2*Ktau*W_e(4)/Izz, -2*Ktau*W_e(5)/Izz, 2*Ktau*W_e(6)/Izz;
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
     Ku/Mtau, 0, 0, 0, 0, 0;
     0, Ku/Mtau, 0, 0, 0, 0;
     0, 0, Ku/Mtau, 0, 0, 0;
     0, 0, 0, Ku/Mtau, 0, 0;
     0, 0, 0, 0, Ku/Mtau, 0;
     0, 0, 0, 0, 0, Ku/Mtau];

% C = 4x14 matrix
C = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0];
     
% D = 4x6 matrix
D = 0;

%% Discrete-Time System

sysdt = c2d(ss(A,B,C,D),T,'zoh');  % Generate Discrete-Time System

Adt = sysdt.a; 
Bdt = sysdt.b; 
Cdt = sysdt.c; 
Ddt = sysdt.d;

%% Discrete-Time Increment Augmaneted System 
% xinc = Ainc*xinc + Binc*uinc, xinc = [x;u]
% y = Cinc*xinc + d

r = 4;                               % number of reference inputs
n = size(Adt,2);                     % number of states
q = size(C,1);                       % number of controlled outputs

% Ainc = [Adt Bdt; 
%         zeros(6,6) eye(6,6)];
%    
% Binc = [Bdt; eye(6,6)];
% 
% Cinc = [Cdt zeros(6,6)];

%% Discrete-Time Full State-Feedback Control
% State feedback control, Z Phi Theta Psi are controlled outputs

Q = diag([200,0,1000,100,1000,150,1000,100,0,0,0,0,0,0]); % State penalty
R = (1*10^-3)*eye(6,6);  % Control penalty

Kdt = dlqr(Adt,Bdt,Q,R,0); % DT State-Feedback Controller Gains

%% LQR Dynamic Simulation

Time = 100;
kT = round(Time/T);

X = zeros(14,kT);
U = zeros(6,kT);

X(:,1) = [0;0;0;0;0;0;0;0;0;0;0;0;0;0];

for k = 1:kT-1

    %Control    
    U(:,k) = -Kdt*X(:,k); %+ U_e;
    
    %Simulation
    X(:,k+1) = Adt*X(:,k) + Bdt*U(:,k);
end

%% Plots

t = (0:kT-1)*T;
Red2Deg = [180/pi,180/pi,180/pi]';

figure(1);
subplot(2,1,1);
plot(t,X(1,:));
legend('Alt');
title('LQR Real Output Altitude');
ylabel('Meters(m)')

subplot(2,1,2);
plot(t,X([3,5,6],:).*Red2Deg);
legend('\phi','\theta','\psi');
title('LQR Real Output Attitude');
ylabel('Radians(r)')

figure(3);
plot(t,U);
legend('U1','U2','U3','U4','U5','U6');
title('LQR Inputs PWM Pulse Width');
ylabel('Micro Seconds(ms)')


%%   Terminal State Penalty

Phi = (Adt - Bdt*Kdt);  % Closed Loopp System
S = (Q + Kdt' * R * Kdt);
P = dlyap(Phi',S);  % Terminal State Penalty from Lyapunov Function

%%  Definition of MPC Prediction, Cost and Constaint Matrices

N = 5;  % Prediction Horizon

[Fxcl,Gxcl,Fycl,Gycl,Fucl,Gucl] = predict_mats_cl(Phi,Bdt,Cdt,Kdt,N); % Parameter Matrices
[Sc1,Scx] = cost_mats_cl(Fxcl,Gxcl,Fycl,Gycl,Fucl,Gucl,Q,R,P); % Cost Function Matricies

% Px = [eye(14); -1*eye(14)];
% qx = [inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;
%       inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf;inf]; % State Constraints
% 
% Pxf = Px;
% qxf = qx; % Terminal State Constraints
% 
% Pu = [eye(6); -1*eye(6)];
% qu = [800*ones(6,1); zeros(6,1)]; % Input Constraints
% 
% [Pc, qc, Sc] = constraint_mats(F,G,Pu,qu,Px,qx,Pxf,qxf); % Constraints As Linear Inequality 

%% Discrete-Time Kalman Filter Design x_dot = A*x + B*u + G*w, y = C*x + D*u + H*w + v

Cy = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0];
 
Dy = zeros(6,6);

Gdt = 1e-1*eye(n);
Hdt = zeros(size(Cy,1),size(Gdt,2)); % No process noise on measurements

Rw = diag([0.001,1,0.00001,1,0.00001,1,0.00001,1,10^-10,10^-10,10^-10,10^-10,10^-10,10^-10]); % Process noise covariance matrix
Rv = diag([1,1,1,1,1,1])*10^-5;     % Measurement noise covariance matrix Note: use low gausian noice for Rv

sys4kf = ss(Adt,[Bdt Gdt],Cy,[Dy Hdt],T);

[kdfilt,Ldt] = kalman(sys4kf,Rw,Rv); 

%% LQ-MPC Dynamic Simulation

Time = 10;
kT = round(Time/T);

Xreal = zeros(18,kT);
Xmodel = zeros(14,kT);
X = zeros(14,kT);

U = zeros(6,kT);
c = zeros(30,kT);
Cost = zeros(1,kT);
Y = zeros(4,kT);
Xe = zeros(4,kT);
Ref = [0;0;0;0];

X(:,1) = [0;0;0;0;0;0;0;0;0;0;0;0;0;0];
X(:,2) = [0;0;0;0;0;0;0;0;0;0;0;0;0;0];
Xreal(:,1) = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
Xest = X;

for k = 2:kT-1
    
    %Estimation
    %Xest(:,k) = Adt*Xest(:,k-1) + Bdt*(U(:,k-1)-U_e);   % No KF Linear Prediction   
    Xest(:,k) = Xreal([5,6,7,8,9,10,11,12,13:18],k);   % No KF Non Linear Prediction
    Y(:,k) = Xreal([5,7,9,11],k);
%    t_span = [0,T];
%    xkf = [0;0;0;0;Xest(:,k-1)];  
%    xode = ode45(@(t,X) Hex_Dynamics(t,X,U(:,k-1)),t_span,xkf); % Nonlinear Prediction
%    Xest(:,k) = xode.y(5:18,end);
%    Y(:,k) = Xreal([5,7,9,11],k);
%    Pred_Error = [Y(:,k) - Xest([1,3,5,7],k); 0; 0];
%    Xest(:,k) = Xest(:,k) + Ldt*Pred_Error;

    %Control    
    [c(:,k),Cst] = quadprog(Sc1,Scx*X(:,k)); % Pc,qc + Sc*X(:,k) Solve Quadratic program
    U(:,k) =  -Kdt*Xest(:,k) + c(1:6,k);
    Cost(:,k) = Cst;
    
    %Simulation    
    t_span = [0,T];
    xode = ode45(@(t,X) Hex_Dynamics(t,X,U(:,k)),t_span,Xreal(:,k)); % Nonlinear Dynamics
    Xmodel(:,k+1) = Adt*Xmodel(:,k) + Bdt*(U(:,k));   % Model Prediction
    Xreal(:,k+1) = xode.y(:,end);
%    [dX] = Hex_Dynamics(t,Xreal(:,k),U(:,k)); % Forward Euler Nonlinear Dynamics 
%    Xreal(:,k+1) = Xreal(:,k) + T*dX;
%    X(:,k+1) = Adt*Xest(:,k) + Bdt*U(:,k);     % Linear Dynamics
end

%% Plots

t = (0:kT-1)*T;
Red2Deg = [180/pi,180/pi,180/pi]';

figure(4);
subplot(2,1,1);
plot(t,Xreal(5,:));
legend('Alt');
title('LQ-MPC Real Output Altitude');
ylabel('Meters(m)')

subplot(2,1,2);
plot(t,Xreal([7,9,11],:).*Red2Deg);
legend('\phi','\theta','\psi');
title('LQ-MPC Real Output Attitude');
ylabel('Radians(r)')

% subplot(2,1,1);
% plot(t,X(1,:));
% legend('Alt');
% title('LQ-MPC Real Output Altitude');
% ylabel('Meters(m)')
% 
% subplot(2,1,2);
% plot(t,X([3,5,7],:).*Red2Deg);
% legend('\phi','\theta','\psi');
% title('LQ-MPC Real Output Attitude');
% ylabel('Radians(r)')

figure(5);
subplot(2,1,1);
plot(t,Xest(1,:));
legend('Alt_e');
title('LQ-MPC Estimated Output Altitude');
ylabel('Meters(m)')

subplot(2,1,2);
plot(t,Xest([3,5,7],:).*Red2Deg);
legend('\phi_e','\theta_e','\psi_e');
title('LQ-MPC Estimated Output Attitude');
ylabel('Radians(r)')

figure(6);
plot(t,U);
legend('U1','U2','U3','U4','U5','U6');
title('LQ-MPC Inputs PWM Pulse Width');
ylabel('Micro Seconds(ms)')

figure(7);
plot(t,Cost);
title('LQ-MPC Cost');
