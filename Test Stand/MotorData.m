clear;
clc;

load("DATA.TXT");
load("LoadData.TXT");
load("ForceRPMdata.TXT");
load("TorqueData.TXT");

NoLoadtime = DATA(1:102,1);
NoLoadinput = DATA(1:102,2) + 300;
NoLoadRPM = DATA(1:102,3);

Loadtime = LoadData(1:102,1);
Loadinput = LoadData(1:102,2) + 300;
LoadRPM = LoadData(1:102,3);

RPM = ForceRPMdata(1:end,2)-2871.43;
Force = ForceRPMdata(1:end,3);

RPM2 =  TorqueData(1:end,2);
Torque = TorqueData(1:end,3)/1000;

figure(1)
plot(NoLoadtime,NoLoadinput);
hold on;
plot(NoLoadtime,NoLoadRPM,'*');

figure(2)
plot(Loadtime,Loadinput);
hold on;
plot(Loadtime,LoadRPM,'*');

figure(3)
plot(RPM,Force,'*');

figure(4)
plot(RPM2,Torque,'*');

% Linear model Poly2:
%      f(x) = p1*x^2 + p2*x + p3
% Coefficients (with 95% confidence bounds):
%        p1 =   1.812e-07  (1.756e-07, 1.867e-07)
%        p2 =   0.0007326  (0.0007006, 0.0007647)
%        p3 =       1.225  (1.183, 1.267)
% 
% Goodness of fit:
%   SSE: 8.957
%   R-square: 0.9974
%   Adjusted R-square: 0.9974
%   RMSE: 0.1452


% Linear model Poly2:
%      f(x) = p1*x^2 + p2*x + p3
% Coefficients (with 95% confidence bounds):
%        p1 =   7.708e-10  (5.24e-10, 1.018e-09)
%        p2 =  -6.623e-06  (-9.336e-06, -3.911e-06)
%        p3 =      0.0161  (0.008837, 0.02337)
% 
% Goodness of fit:
%   SSE: 0.0001328
%   R-square: 0.7607
%   Adjusted R-square: 0.7567
%   RMSE: 0.001043

  
NoLoadtf = tf(1506.1,[1 56.55]);
Loadtf = tf(515.5,[1 44.22]);

figure(5)
opt = stepDataOptions('InputOffset',300,'StepAmplitude',200);
step(Loadtf,opt);