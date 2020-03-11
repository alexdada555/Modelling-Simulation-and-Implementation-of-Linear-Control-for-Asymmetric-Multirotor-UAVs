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
W_e = ((-6*Kthrust2) + sqrt((6*Kthrust2)^2 - (4*(-M*g)*(3*Kw*Kthrust + 3*Kthrust))))/(2*(3*Kw*Kthrust + 3*Kthrust));
U_e = (W_e/Ku);

%% Define Discrete-Time BeagleBone Dynamics

T = 0.01; % Sample period (s)- 100Hz
ADC = 3.3/((2^12)-1); % 12-bit ADC Quantization
DAC =  3.3/((2^12)-1); % 12-bit DAC Quantization

%% Define Linear Continuous-Time Multirotor Dynamics: x_dot = Ax + Bu, y = Cx + Du         

% A =  14x14 matrix
A = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 2*Kthrust*W_e/M, 2*Kw*Kthrust*W_e/M, 2*Kthrust*W_e/M, 2*Kw*Kthrust*W_e/M, 2*Kthrust*W_e/M, 2*Kw*Kthrust*W_e/M;
     0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, L1*2*Kthrust*W_e/Ixx, L1*2*Kw*Kthrust*W_e/Ixx, -L1*2*Kthrust*W_e/Ixx, -L1*2*Kw*Kthrust*W_e/Ixx, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, -L2*2*Kthrust*W_e/Iyy, -L2*2*Kw*Kthrust*W_e/Iyy, -L2*2*Kthrust*W_e/Iyy, -L2*2*Kw*Kthrust*W_e/Iyy, L3*2*Kthrust*W_e/Iyy,L3*2*Kw*Kthrust*W_e/Iyy;
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

%% System Characteristics

poles = eig(Adt);
Jpoles = jordan(Adt);
% System Unstable

cntr = rank(ctrb(Adt,Bdt));
% Fully Reachable

obs = rank(obsv(Adt,Cdt));
% Partially Observable but Detectable

%% Discrete-Time Full Integral Augmaneted System 

Cr  = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0];    

r = 4;                                % number of reference inputs
n = size(A,2);                        % number of states
q = size(Cr,1);                       % number of controlled outputs

Dr = zeros(q,6);

Adtaug = [Adt zeros(n,r); 
          -Cr*Adt eye(q,r)];
   
Bdtaug = [Bdt; -Cr*Bdt];

Cdtaug = [C zeros(r,r)];

%% Discrete-Time Full State-Feedback Control
% State feedback control design with integral control via state augmentation
% Z Phi Theta Psi are controlled outputs

Q = diag([50,200,250,500,250,500,1,1000,0,0,0,0,0,0,1,30,30,0.1]); % State penalty
R = (1*10^-3)*eye(6,6);  % Control penalty

Kdtaug = dlqr(Adtaug,Bdtaug,Q,R,0); % DT State-Feedback Controller Gains
Kdt = Kdtaug(:,1:n);        % LQR Gains
Kidt = Kdtaug(:,n+1:end);  % Integral Gains

Kxr = zeros(14,4);
Kxr(1,1) = 1;
Kxr(3,2) = 1;
Kxr(5,3) = 1;
Kxr(7,4) = 1;

%% Discrete-Time Kalman Filter Design x_dot = A*x + B*u + G*w, y = C*x + D*u + H*w + v

Cy = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0];
 
Dy = zeros(6,6);

Ky = zeros(6,4);
KwH = zeros(6,2);

Ky(1,1) = 1;
Ky(2,2) = 1;
Ky(3,3) = 1;
Ky(4,4) = 1;

KwH(5,1) = 1;
KwH(6,2) = 1;

Gdt = 1e-1*eye(n);
Hdt = zeros(size(Cy,1),size(Gdt,2)); % No process noise on measurements

Rw = diag([0.001,1,0.00001,1,0.00001,1,0.00001,1,10^-10,10^-10,10^-10,10^-10,10^-10,10^-10]);   % Process noise covariance matrix
Rv = diag([1,1,1,1,1,1])*10^-5;     % Measurement noise covariance matrix Note: use low gausian noice for Rv
N = zeros(size(Rw,2),size(Rv,2));

sys4kf = ss(Adt,[Bdt Gdt],Cy,[Dy Hdt],T);

[kdfilt,Ldt] = kalman(sys4kf,Rw,Rv); 

%%  Dynamic Simulation

Time = 100;
kT = round(Time/T);

X = zeros(14,kT);
Xreal = zeros(18,kT);

U = U_e*ones(6,kT);

Y = zeros(4,kT);
Xe = zeros(4,kT);

Ref = [0;0;0;0];
x_ini = [0;0;0;0;0;0;0;0;0;0;0;0;0;0];

X(:,2) = x_ini;
Xest = X;
Xest(:,1) = x_ini+0.001*randn(14,1);
Xreal(5:end,2) = x_ini;
U(:,1) = 0;

for k = 2:kT-1
    
    %Estimation
    %Xest(:,k) = Adt*Xest(:,k-1)+Bdt*(U(:,k-1)-U_e);       %Linear Prediction Phase    
    t_span = [0,T];
    xkf = [0;0;0;0;Xest(:,k-1)];              %Remapping    
    xode = ode45(@(t,X) Hex_Dynamics(t,X,U(:,k-1)),t_span,xkf);    %Nonlinear Prediction
    Xest(:,k) = xode.y(5:18,end);            %Remappping back
    Y(:,k) = Xreal([5,7,9,11],k);
    Pred_Error = [Y(:,k) - Xest([1,3,5,7],k); 0; 0];
    Xest(:,k) = Xest(:,k) + Ldt*Pred_Error;
    
    %Control    
    %Xest(:,k) = Xreal([5,6,7,10,8,11,9,12,13:18],k);          %No Kalman Filter
    Xe(:,k) = Xe(:,k-1) + (Ref - Xest([1,3,5,7],k));
    U(:,k) = U_e - [Kdt,Kidt]*[Xest(:,k) - [Ref(1);0;Ref(2);0;Ref(3);0;Ref(4);0;W_e*ones(6,1)]; Xe(:,k)];
    
    %Simulation    
    t_span = [0,T];
    xode = ode45(@(t,X) Hex_Dynamics(t,X,U(:,k)),t_span,Xreal(:,k));
    Xreal(:,k+1) = xode.y(:,end);
    
    %%%%% Forward Euler Nonlinear Dynamics %%%%%%%
%     [dX]=Hex_Dynamics(t,Xreal(:,k),U(:,k));
%     Xreal(:,k+1)=Xreal(:,k)+T*dX;

    %%%%% Fully Linear Dynamics %%%%%%
%     X(:,k+1)=Adt*X(:,k)+Bdt*U(:,k);

    if k == 10/T
        Ref(1) = 1;
    end
    if k == 15/T
        Ref(2) = 30*pi/180;
    end
    if k == 20/T
        Ref(2) = 0;
    end
    if k == 25/T
        Ref(3) = 30*pi/180;
    end
    if k == 30/T
        Ref(3) = 0;
    end
%     if k == 40/T
%         Ref(4) = 30*pi/180;
%     end
end

%Plots
t = (0:kT-1)*T;
figure(1);
subplot(3,1,1);
plot(t,Xreal([5,7,9,11],:))
legend('Alt','\phi','\theta','\psi')
subplot(3,1,2);
plot(t,Xest([1,3,5,7],:))
legend('Alt_e','\phi_e','\theta_e','\psi_e')
subplot(3,1,3);
plot(t,U);
