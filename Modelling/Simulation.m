close all
clc

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

%     if k == 10/T
%         Ref(1) = 1;
%     end
%     if k == 20/T
%         Ref(2) = 30*pi/180;
%     end
%     if k == 30/T
%         Ref(3) = 30*pi/180;
%     end
    if k == 40/T
        Ref(4) = 30*pi/180;
    end
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
