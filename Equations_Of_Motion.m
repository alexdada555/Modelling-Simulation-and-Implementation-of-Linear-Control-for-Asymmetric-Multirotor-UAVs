%% Mass of the Multirotor in Kilograms as taken from the CAD

M = 2100/1000; 
g = 9.81;
U_e = 1500;

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
Ktau = 0.00008;
Kthrust = 0.013;
Mtau = 0.09;
Ku = 0.014;

%% Air resistance damping coeeficients

Dxx = 0.01212;
Dyy = 0.01212;
Dzz = 0.0648;

%% X = [x xdot y ydot z zdot phi theta psi p q r w1 w2 w3 w4 w5 w6]

x = X(1);
xdot = X(2);
y = X(3);
ydot = X(4);
z = X(5);
zdot = X(6);

phi = X(7); 
theta = X(8);
psi = X(9);

p = X(10);
q = X(11);
r = X(12);

w1 = X(13);
w2 = X(14);
w3 = X(15);
w4 = X(16);
w5 = X(17);
w6 = X(18);

%%

dX = zeros(18,1);
Y = zeros(4,1);

%Forces and Torques
F = zeros(6,1);
T = zeros(6,1);

F(1)= Kw*Kthrust*(w1^2);
F(2)= Kthrust*(w2^2);
F(3)= Kw*Kthrust*(w3^2);
F(4)= Kthrust*(w4^2);
F(5)= Kw*Kthrust*(w5^2);
F(6)= Kthrust*(w6^2);

T(1)= Ktau*w1^2;
T(2)= -Ktau*w2^2;
T(3)= -Ktau*w3^2;
T(4)= Ktau*w4^2;
T(5)= Ktau*w5^2;
T(6)= -Ktau*w6^2;

Fn = sum(F);
Tn = sum(T);

%Straight forward integration....
dX(1) = X(2);
dX(3) = X(4);
dX(5) = X(6);
dX(7) = p + r*(cos(phi)*tan(theta)) + q*(sin(phi)*tan(theta));
dX(8) = q*cos(phi) - r*sin(phi);
dX(9) = r*(cos(phi)/cos(theta)) + q*(sin(phi)/cos(theta));

%Lower order direvatives
dX(2) = Fn/M*(cos(phi)*sin(theta)*cos(psi)) + Fn/M*(sin(phi)*sin(psi)) - Dxx/M*xdot;
dX(4) = Fn/M*(cos(phi)*sin(theta)*sin(psi)) - Fn/M*(sin(phi)*sin(psi)) - Dyy/M*ydot;
dX(6) = Fn/M*(cos(phi)*cos(theta)) - g - Dzz/M*zdot;

dX(10) = (L3/Ixx)*(F(5)+F(6)) - (L2/Ixx)*(F(1)+F(2)+F(3)+F(4)) - ((Izz-Iyy)/Izz)*(r*q);
dX(11) = (L1/Iyy)*((F(5)+F(6)) - (F(1)+F(2)+F(3)+F(4))) - ((Izz-Ixx)/Iyy)*(p*r);
dX(12) = Tn/Izz - ((Iyy-Ixx)/Izz)*(p*q);


Y(1) = z;
Y(2) = phi;
Y(3) = theta;
Y(4) = psi;

%Motor dynamics
dX(13) = -(1/Mtau)*w1 + (Ku/Mtau)*U(1);
dX(14) = -(1/Mtau)*w2 + (Ku/Mtau)*U(2);
dX(15) = -(1/Mtau)*w3 + (Ku/Mtau)*U(3);
dX(16) = -(1/Mtau)*w4 + (Ku/Mtau)*U(4);
dX(17) = -(1/Mtau)*w5 + (Ku/Mtau)*U(5);
dX(18) = -(1/Mtau)*w6 + (Ku/Mtau)*U(6);


% % % % % %Form Mass Matrix
% % % % % Mm=[m,0,0,0,m*dz,-m*dy;
% % % % %     0,m,0,-m*dz,0,m*dx;
% % % % %     0,0,m,m*dy,-m*dx,0;
% % % % %     0,-m*dz,m*dy,Ixx,-Ixy,-Ixz;
% % % % %     m*dz,0,-m*dx,-Ixy,Iyy,-Iyz;
% % % % %     -m*dy,m*dx,0,-Ixz,-Iyz,Izz];
% % % % % 
% % % % % %Form Ma Matrix
% % % % % Ma=[X+m*R*V-m*Q*W+m*dx*(Q^2+R^2)-m*dy*P*Q-m*dz*P*R;
% % % % %     Y-m*R*U+m*P*W-m*dx*P*Q+m*dy*(P^2+R^2)-m*dz*Q*R;
% % % % %     Z+m*Q*U-m*P*V-m*dx*P*R-m*dy*Q*R+m*dz*(P^2+Q^2);
% % % % %     L-Q*R*(Izz-Iyy)-P*R*Ixy+P*Q*Ixz-(R^2-Q^2)*Iyz+(Q*U-P*V)*m*dy+(R*U-P*W)*m*dz;
% % % % %     M-P*R*(Ixx-Izz)-P*Q*Iyz+Q*R*Ixy-(P^2-R^2)*Ixz+(R*V-Q*W)*m*dz-(Q*U-P*V)*m*dx;
% % % % %     N-P*Q*(Iyy-Ixx)-Q*R*Ixz+P*R*Iyz-(Q^2-P^2)*Ixy-(R*U-P*W)*m*dx+(-R*V+Q*W)*m*dy];