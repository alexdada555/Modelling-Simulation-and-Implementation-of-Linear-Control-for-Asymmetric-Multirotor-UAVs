clear;
clc;

load("DATA.TXT");
load("LoadData.TXT");
load("ForceRPMdata.TXT");

NoLoadtime = DATA(1:102,1);
NoLoadinput = DATA(1:102,2) + 300;
NoloadRPM = DATA(1:102,3);

Loadtime = LoadData(1:102,1);
Loadinput = LoadData(1:102,2) + 300;
LoadRPM = LoadData(1:102,3);

RPM = ForceRPMdata(1:end,2)-2871.43;
Force = ForceRPMdata(1:end,3);

%figure(1)
%plot(time,NoLoadinput);
%hold on;
%plot(time,NoLoadRPM,'*');

%figure(2)
%plot(time,Loadinput);
%hold on;
%plot(time,LoadRPM,'*');

figure(3)
plot(RPM,Force,'*');

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
  
%NoLoadtf = tf(1506.1,[1 56.55]);
%Loadtf = tf(515.5,[1 44.22]);

%figure(4)
%opt = stepDataOptions('InputOffset',300,'StepAmplitude',200);
%step(Loadtf,opt);