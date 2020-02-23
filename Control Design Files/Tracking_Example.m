%Model
sys = ss(A,B,C,0);
sysd = c2d(sys,T);
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;

%LQR Design
Q = diag([0.01,2000,100,0,100,0,1,0,0,0,0,0,0,0]); % State penalty
R = (1*10^-3)*eye(6,6);  % Control penalty

Kx = dlqr(Ad,Bd,Q,R,0);

%Simulation Parameters
Time = 20;
dt = T;
kT = round(Time/dt);
X = zeros(14,kT);
Y = zeros(4,kT);
U = zeros(6,kT);
X(:,1) = [0;0;0;0;0;0;0;0;0;0;0;0;0;0];

for k = 1:kT-1
    %Control System
    Ref = [0;0;0;20*pi/180];
    U(:,k )= -Kx*(X(:,k)-[Ref(1);0;Ref(2);0;Ref(3);0;Ref(4);0;0;0;0;0;0;0]);
    
    %Simulate
    X(:,k+1) = Ad*X(:,k)+Bd*U(:,k);
    Y(:,k) = Cd*X(:,k);
end

%Plot
t = dt*[0:kT-1];

figure(3);
subplot(4,1,1);
plot(t,Y(1,:));
subplot(4,1,2);
plot(t,Y(2,:)*(180/pi));
subplot(4,1,3);
plot(t,Y(3,:)*(180/pi));
subplot(4,1,4);
plot(t,Y(4,:)*(180/pi));

figure(4);
plot(t,U');
