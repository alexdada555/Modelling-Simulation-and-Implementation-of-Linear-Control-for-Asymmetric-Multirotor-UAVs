%function Extract_States(X)
%% X = [x xdot y ydot z zdot phi phidot theta thetadot psi psidot w1 w2 w3 w4 w5 w6]

X = zeros(18,1);
x = X(1);
xdot = X(2);
y = X(3);
ydot = X(4);
z = X(5);
zdot = X(6);
phi = X(7);  
phidot = X(8);
theta = X(9);
thetadot = X(10);
psi = X(11);
psidot = X(12);
w1 = X(13);
w2 = X(14);
w3 = X(15);
w4 = X(16);
w5 = X(17);
w6 = X(18);

%end